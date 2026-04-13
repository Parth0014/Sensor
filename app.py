import argparse
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, render_template
from flask_cors import CORS

# ── Config ────────────────────────────────────────────────────
# REMOVED: adc_to_ma() conversion — the firmware now does it.
#          Python receives milliamps directly on the DATA line.
#
# CHANGED: NOISE_FLOOR_MA 150 → 50
#          Must match firmware MIN_CURRENT_MA (default 50).
#          The old 150 mA floor was silently zeroing real small loads.
ALARM_MA        = 50     # stolen mA threshold to trigger alarm
ALARM_HITS      = 3       # consecutive reads above threshold to arm alarm
CLEAR_HITS      = 5       # consecutive reads below threshold to clear alarm
NOISE_FLOOR_MA  = 50.0    # ignore readings below this (matches firmware MIN_CURRENT_MA)
BAUD_RATE       = 115200

# ── Shared state ──────────────────────────────────────────────
state = {
    "s1": 0, "s2": 0, "delta": 0,
    "s1_W": 0.0, "s2_W": 0.0,
    "status": "INIT",
    "connected": False,
    "last_update": 0.0,
    "history": [],
}
lock = threading.Lock()
_ser = None

# ── Moving average buffers ─────────────────────────────────────
_ma1 = deque([0.0] * 8, maxlen=8)
_ma2 = deque([0.0] * 8, maxlen=8)
_s2_zero_start_time = None  # Track when S2 first becomes 0
# ─────────────────────────────────────────────────────────────
def process(s1_ma: float, s2_ma: float, s1_w: float = 0.0, s2_w: float = 0.0):
    """
    Accept already-converted milliamp values from the DATA line.
    Moving average still applied here for stability.
    """
    global _s2_zero_start_time
    
    _ma1.append(s1_ma)
    _ma2.append(s2_ma)

    s1 = sum(_ma1) / len(_ma1)
    s2 = sum(_ma2) / len(_ma2)

    # Soft floor matches firmware MIN_CURRENT_MA — anything below is noise
    s1 = s1 if s1 >= NOISE_FLOOR_MA else 0.0
    s2 = s2 if s2 >= NOISE_FLOOR_MA else 0.0

    # Sanity check: if S1 (total current) is 0, S2 (legal current) must also be 0
    # This prevents noise / sensor touches from showing fake readings
    if s1 == 0.0:
        s2 = 0.0

    # Track if S2 = 0 for 7+ seconds — if so, zero S1 too (sensor malfunction)
    if s2 == 0.0:
        if _s2_zero_start_time is None:
            _s2_zero_start_time = time.time()  # Mark when S2 first became 0
        elif time.time() - _s2_zero_start_time >= 7.0:
            s1 = 0.0  # If S2 has been 0 for 7+ seconds, something is wrong
    else:
        _s2_zero_start_time = None  # Reset when S2 is non-zero

    delta = max(0, int(s1) - int(s2))
    theft = (delta >= ALARM_MA) and (s1 >= NOISE_FLOOR_MA)

    with lock:
        state["s1"]          = int(s1)
        state["s2"]          = int(s2)
        state["s1_W"]        = round(s1_w, 1)
        state["s2_W"]        = round(s2_w, 1)
        state["delta"]       = delta
        state["last_update"] = time.time()
        state["history"].append({
            "t":     round(time.time(), 1),
            "s1":    int(s1),
            "s2":    int(s2),
            "delta": delta,
        })
        if len(state["history"]) > 60:
            state["history"].pop(0)

        if state["status"] != "ALARM":
            state.setdefault("_ac", 0)
            if theft:
                state["_ac"] += 1
                if state["_ac"] >= ALARM_HITS:
                    state["status"] = "ALARM"
                    state["_ac"] = 0
                    _buzz(True)
            else:
                state["_ac"] = 0
        else:
            state.setdefault("_oc", 0)
            if not theft:
                state["_oc"] += 1
                if state["_oc"] >= CLEAR_HITS:
                    state["status"] = "OK"
                    state["_oc"] = 0
                    _buzz(False)
            else:
                state["_oc"] = 0

    print(f"S1={int(s1):5d}mA ({s1_w:6.1f}W)  S2={int(s2):5d}mA ({s2_w:6.1f}W)  Δ={delta:5d}mA  [{state['status']}]")


def _buzz(on: bool):
    try:
        if _ser and _ser.is_open:
            _ser.write(b"BUZZ:1\n" if on else b"BUZZ:0\n")
    except Exception as e:
        print(f"[WARN] buzz: {e}")


def _parse_data_line(line: str):
    """
    Parse new-format DATA line from firmware:
        DATA,s1_mA=435.0,s2_mA=0.0,s1_W=100.1,s2_W=0.0

    Returns (s1_mA, s2_mA, s1_W, s2_W) floats or None on parse error.
    """
    try:
        kv = {}
        for part in line[5:].split(","):   # skip leading "DATA,"
            k, v = part.split("=")
            kv[k.strip()] = float(v.strip())
        return kv["s1_mA"], kv["s2_mA"], kv.get("s1_W", 0.0), kv.get("s2_W", 0.0)
    except Exception as e:
        print(f"[PARSE_ERROR] Failed to parse line: {repr(line)} — {e}")
        return None


# ── Serial reader thread ──────────────────────────────────────
def serial_thread(port):
    global _ser
    calibrated = False

    while True:
        try:
            print(f"[SERIAL] Connecting to {port} ...")
            ser = serial.Serial(
                port,
                BAUD_RATE,
                dsrdtr  = False,
                rtscts  = False,
                timeout = 30,   # must cover 5s countdown + ~8s calibration silence
            )
            ser.dtr = False
            ser.rts = False

            _ser = ser
            calibrated = False

            with lock:
                state["connected"] = True
                state["status"]    = "CAL"

            # Flush ESP32 boot ROM garbage
            time.sleep(0.5)
            ser.reset_input_buffer()
            print("[SERIAL] Connected — waiting for CAL...")

            while ser.is_open:
                raw = ser.readline()
                if not raw:
                    continue    # readline timed out — keep waiting

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                print(f"[ESP] {line}")

                # ── Calibration handshake ──────────────────────
                if line.startswith("CAL:BASELINE,"):
                    parts = line.split(",")
                    if len(parts) == 3:
                        calibrated = True
                        print("[CAL] Baseline received — ready.")

                elif line == "INFO:READY":
                    with lock:
                        state["status"] = "OK"

                # ── Primary: new DATA line (mA + W already computed) ──
                elif line.startswith("DATA,") and calibrated:
                    result = _parse_data_line(line)
                    if result:
                        process(*result)
                        if state["status"] == "CAL":
                            with lock:
                                state["status"] = "OK"
                
                elif line.startswith("DATA,") and not calibrated:
                    print("[DEBUG] DATA line received but not calibrated yet — ignoring")

                # ── Fallback: legacy RAW line (ADC counts) ─────────────
                # Kept so the server still works if you flash older firmware.
                # If the firmware is up-to-date, this branch is never hit
                # because the DATA line always appears first on the same loop.
                elif line.startswith("RAW,") and calibrated:
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            # Convert ADC counts → mA the old way
                            ADC_LSB_mV  = 0.125
                            SENSITIVITY = 0.185
                            def _adc_to_ma(c):
                                return (c * ADC_LSB_mV / 1000.0 / SENSITIVITY) * 1000.0
                            s1 = _adc_to_ma(float(parts[1]))
                            s2 = _adc_to_ma(float(parts[2]))
                            process(s1, s2)
                            if state["status"] == "CAL":
                                with lock:
                                    state["status"] = "OK"
                        except ValueError:
                            pass

        except serial.SerialException as e:
            with lock:
                state["connected"] = False
                state["status"]    = "DISCONNECTED"
            print(f"[SERIAL] Lost: {e}. Retrying in 5 s...")
            time.sleep(5)

        finally:
            try:
                if _ser and _ser.is_open:
                    _ser.close()
            except Exception:
                pass
            _ser = None


# ── Flask app ─────────────────────────────────────────────────
app = Flask(__name__, template_folder="templates")
CORS(app)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/state")
def api_state():
    with lock:
        return jsonify(state)

@app.route("/data")
def data():
    with lock:
        return jsonify({k: state[k] for k in ("s1", "s2", "delta", "status")})

@app.route("/api/recal", methods=["POST"])
def api_recal():
    try:
        if _ser and _ser.is_open:
            _ser.write(b"RECAL\n")
            with lock:
                state["status"] = "CAL"
            return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})
    return jsonify({"ok": False, "error": "not connected"})

@app.route("/api/buzz/<int:on>", methods=["POST"])
def api_buzz(on):
    _buzz(bool(on))
    return jsonify({"ok": True})


# ── Serial port auto-detection ─────────────────────────────────
def auto_port():
    keywords = ["CH340", "CP210", "USB Serial", "UART", "ESP", "usbserial", "ttyUSB"]
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "") + " " + (p.manufacturer or "")
        if any(k.lower() in desc.lower() for k in keywords):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


# ── Entry point ───────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PowerGuard — current theft monitor")
    parser.add_argument("--port",     default=None, help="Serial port e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--web-port", default=8765, type=int, help="HTTP dashboard port")
    args = parser.parse_args()

    port = args.port or auto_port()
    if not port:
        print("No serial port found. Plug in the ESP32 or use --port COM3")
        exit(1)

    print(f"Starting on {port}  |  Dashboard → http://localhost:{args.web_port}")
    print(f"Config: NOISE_FLOOR={NOISE_FLOOR_MA}mA  ALARM_THRESHOLD={ALARM_MA}mA")

    threading.Thread(target=serial_thread, args=(port,), daemon=True).start()
    app.run(host="0.0.0.0", port=args.web_port, debug=False, threaded=True)