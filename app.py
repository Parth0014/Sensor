import argparse
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, render_template
from flask_cors import CORS

# ── Config ────────────────────────────────────────────────────
SENSITIVITY     = 0.185   # ACS712 5A  (V/A)
ADC_LSB_mV      = 0.125   # ADS1115 GAIN_ONE (mV/LSB)
ALARM_MA        = 200     # stolen mA threshold to trigger alarm
ALARM_HITS      = 3       # consecutive reads above threshold to arm alarm
CLEAR_HITS      = 5       # consecutive reads below threshold to clear alarm
NOISE_FLOOR_MA  = 150.0   # deadzone — below this is sensor noise, not real current
BAUD_RATE       = 115200

def adc_to_ma(rms_adc):
    v = rms_adc * ADC_LSB_mV / 1000.0
    return (v / SENSITIVITY) * 1000.0

# ── Shared state ──────────────────────────────────────────────
state = {
    "s1": 0, "s2": 0, "delta": 0,
    "status": "INIT",
    "connected": False,
    "last_update": 0.0,
    "history": [],
}
lock = threading.Lock()
_ser = None

# ── Moving average buffers ────────────────────────────────────
_ma1 = deque([0.0] * 8, maxlen=8)
_ma2 = deque([0.0] * 8, maxlen=8)

# ─────────────────────────────────────────────────────────────
def process(rms1, rms2):
    _ma1.append(rms1)
    _ma2.append(rms2)

    s1 = adc_to_ma(sum(_ma1) / len(_ma1))
    s2 = adc_to_ma(sum(_ma2) / len(_ma2))

    s1 = s1 if s1 >= NOISE_FLOOR_MA else 0.0
    s2 = s2 if s2 >= NOISE_FLOOR_MA else 0.0

    delta = max(0, int(s1) - int(s2))
    theft = (delta >= ALARM_MA) and (s1 >= NOISE_FLOOR_MA)

    with lock:
        state["s1"]          = int(s1)
        state["s2"]          = int(s2)
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

    print(f"S1={int(s1):5d}mA  S2={int(s2):5d}mA  Δ={delta:5d}mA  [{state['status']}]")

def _buzz(on: bool):
    try:
        if _ser and _ser.is_open:
            _ser.write(b"BUZZ:1\n" if on else b"BUZZ:0\n")
    except Exception as e:
        print(f"[WARN] buzz: {e}")

# ── Serial reader thread ──────────────────────────────────────
def serial_thread(port):
    global _ser
    calibrated = False

    while True:
        try:
            print(f"[SERIAL] Connecting to {port} ...")

            # dsrdtr/rtscts=False → prevents DTR asserting and resetting the ESP32
            ser = serial.Serial(
                port,
                BAUD_RATE,
                dsrdtr  = False,
                rtscts  = False,
                # FIX: timeout raised from 2s → 30s
                # During calibration the ESP32 is silent for up to ~8 seconds
                # (5s countdown + warmup + 400 samples).
                # With timeout=2, pyserial's readline() returned b'' after 2s,
                # causing the for-loop iterator to exit and triggering a
                # reconnect/reset loop. 30s comfortably covers any silence.
                timeout = 30,
            )
            ser.dtr = False
            ser.rts = False

            _ser = ser
            calibrated = False

            with lock:
                state["connected"] = True
                state["status"]    = "CAL"

            # Flush ESP32 boot ROM garbage (prints at 74880 baud → mojibake at 115200)
            time.sleep(0.5)
            ser.reset_input_buffer()
            print("[SERIAL] Connected. Boot garbage flushed — waiting for CAL...")

            # FIX: replaced  `for line in ser`  with a manual while loop.
            # The for-loop iterator calls readline() and raises StopIteration
            # on any empty return (timeout). Our manual loop just continues,
            # keeping the connection alive through long calibration silences.
            while ser.is_open:
                raw = ser.readline()

                # Empty bytes = readline timed out — just keep waiting
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                print(f"[ESP] {line}")

                if line.startswith("CAL:BASELINE,"):
                    parts = line.split(",")
                    if len(parts) == 3:
                        calibrated = True
                        print("[CAL] Baseline received — ready.")

                elif line == "INFO:READY":
                    with lock:
                        state["status"] = "OK"

                elif line.startswith("RAW,") and calibrated:
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            process(float(parts[1]), float(parts[2]))
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

# ── Serial port auto-detection ────────────────────────────────
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