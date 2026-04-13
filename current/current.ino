#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define BUZZER_PIN      14
#define BAUD_RATE       115200

// ── Sampling ──────────────────────────────────────────────────
// 80 samples × 800 µs ≈ 64 ms = 3.2 cycles at 50 Hz
#define SAMPLES         80
#define CAL_SAMPLES     400

// ── Sensor calibration ────────────────────────────────────────
// ACS712-5A output sensitivity: 185 mV/A
// ADS1115 GAIN_ONE:            0.125 mV per count
// Net counts per Ampere:       185 / 0.125 = 1480.0
#define COUNTS_PER_AMP  1480.0f

// ── Mains voltage (RMS) ───────────────────────────────────────
// India / EU: 230 V
#define MAINS_VOLTS     230.0f

// ── Noise floor ───────────────────────────────────────────────
// Minimum current (mA) to be reported as a real load.
#define MIN_CURRENT_MA  50.0f

// ── S2 trim ──────────────────────────────────────────────────
// Compensates for baseline mismatch between the two ACS712 sensors.
// If S2 > S1 in normal (no-theft) state, increase in steps of 10
// until S1 ≈ S2 with no theft load connected. Start at 20.0.
#define S2_TRIM_MA      20.0f

// ── Channel assignment ────────────────────────────────────────
// ACS712 #1 (TOTAL current)  → ADS1115 A0
// ACS712 #2 (LEGAL current)  → ADS1115 A1
#define CH_TOTAL        0
#define CH_LEGAL        1

// ── WDT yield interval ───────────────────────────────────────
// Call yield() every N samples inside long loops to feed the
// ESP8266 watchdog. 50 samples × ~2.3 ms ≈ 115 ms — well under
// the ~3.2 s WDT timeout and safe for WiFi background tasks.
#define YIELD_EVERY     50

Adafruit_ADS1115 ads;
float baseline_total = 0.0f;
float baseline_legal = 0.0f;

// ─────────────────────────────────────────────────────────────
void calibrateBaseline() {
  Serial.println("CAL:STARTED");

  // Warmup — let ADC internal reference settle.
  // yield() every 10 iterations keeps WDT happy.
  for (int i = 0; i < 50; i++) {
    ads.readADC_SingleEnded(CH_TOTAL);
    ads.readADC_SingleEnded(CH_LEGAL);
    if (i % 10 == 0) {
      Serial.println("CAL:WARMUP");
      yield();   // ← FIX: feed WDT during warmup
    }
    delay(5);
  }

  // Print raw samples to verify ADC midpoint (~18960 counts = 2.5 V at GAIN_ONE)
  Serial.println("DBG:RAW_SAMPLES_BEFORE_CAL");
  for (int i = 0; i < 5; i++) {
    int16_t r0 = ads.readADC_SingleEnded(CH_TOTAL);
    int16_t r1 = ads.readADC_SingleEnded(CH_LEGAL);
    Serial.print("DBG:RAW,ch_total="); Serial.print(r0);
    Serial.print(",ch_legal=");        Serial.println(r1);
    delay(10);
  }

  // Calibrate CH_TOTAL in its own loop (separate loops prevent
  // phase-offset error that made S2 > S1 in normal state).
  // yield() every YIELD_EVERY samples feeds the WDT.
  double sum_total = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum_total += ads.readADC_SingleEnded(CH_TOTAL);
    delayMicroseconds(1200);
    if (i % YIELD_EVERY == 0) yield();   // ← FIX: feed WDT
  }
  baseline_total = (float)(sum_total / CAL_SAMPLES);

  // Calibrate CH_LEGAL in its own loop
  double sum_legal = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum_legal += ads.readADC_SingleEnded(CH_LEGAL);
    delayMicroseconds(1200);
    if (i % YIELD_EVERY == 0) yield();   // ← FIX: feed WDT
  }
  baseline_legal = (float)(sum_legal / CAL_SAMPLES);

  Serial.print("DBG:BASELINE_COUNTS,total=");
  Serial.print(baseline_total, 1);
  Serial.print(",legal=");
  Serial.println(baseline_legal, 1);

  float v_total = baseline_total * 0.125f / 1000.0f;
  float v_legal = baseline_legal * 0.125f / 1000.0f;
  Serial.print("DBG:BASELINE_VOLTS,total=");
  Serial.print(v_total, 3);
  Serial.print("V,legal=");
  Serial.print(v_legal, 3);
  Serial.println("V  (expected ~2.500 V each)");

  Serial.print("CAL:BASELINE,total=");
  Serial.print(baseline_total, 2);
  Serial.print(",legal=");
  Serial.println(baseline_legal, 2);
}

// ─────────────────────────────────────────────────────────────
void readAndSend() {
  double sq_total = 0, sq_legal = 0;
  float maxDev_total = 0, maxDev_legal = 0;

  for (int i = 0; i < SAMPLES; i++) {
    float v_total = (float)ads.readADC_SingleEnded(CH_TOTAL) - baseline_total;
    float v_legal = (float)ads.readADC_SingleEnded(CH_LEGAL) - baseline_legal;

    if (fabsf(v_total) > maxDev_total) maxDev_total = fabsf(v_total);
    if (fabsf(v_legal) > maxDev_legal) maxDev_legal = fabsf(v_legal);

    sq_total += (double)v_total * v_total;
    sq_legal += (double)v_legal * v_legal;

    delayMicroseconds(800);
    // readAndSend() runs in ~64 ms total — no yield needed here.
    // If you raise SAMPLES above ~200, add yield() here too.
  }

  // RMS counts → Amps → milliamps
  float rms_counts_total = sqrtf((float)(sq_total / SAMPLES));
  float rms_counts_legal = sqrtf((float)(sq_legal / SAMPLES));

  float current_total_mA = (rms_counts_total / COUNTS_PER_AMP) * 1000.0f;
  float current_legal_mA = (rms_counts_legal / COUNTS_PER_AMP) * 1000.0f;

  // Noise floor
  if (current_total_mA < MIN_CURRENT_MA) current_total_mA = 0.0f;
  if (current_legal_mA < MIN_CURRENT_MA) current_legal_mA = 0.0f;

  // S2 trim — cancel residual baseline offset
  current_legal_mA = max(0.0f, current_legal_mA - S2_TRIM_MA);

  // Apparent power (unity power factor assumed)
  float watts_total = current_total_mA / 1000.0f * MAINS_VOLTS;
  float watts_legal = current_legal_mA / 1000.0f * MAINS_VOLTS;

  // Theft current = total − legal (clamped to 0)
  float theft_mA = max(0.0f, current_total_mA - current_legal_mA);
  float theft_W  = theft_mA / 1000.0f * MAINS_VOLTS;

  Serial.print("DATA,s1_mA=");   Serial.print(current_total_mA, 1);
  Serial.print(",s2_mA=");       Serial.print(current_legal_mA, 1);
  Serial.print(",s1_W=");        Serial.print(watts_total, 1);
  Serial.print(",s2_W=");        Serial.print(watts_legal, 1);
  Serial.print(",theft_mA=");    Serial.print(theft_mA, 1);
  Serial.print(",theft_W=");     Serial.println(theft_W, 1);

  // Legacy RAW line for backward compatibility
  Serial.print("RAW,");
  Serial.print(rms_counts_total, 2);
  Serial.print(",");
  Serial.println(rms_counts_legal, 2);

  // Debug peak deviation every 10 readings
  static int dbgCount = 0;
  if (++dbgCount >= 10) {
    dbgCount = 0;
    Serial.print("DBG:MAX_DEV,total="); Serial.print(maxDev_total, 0);
    Serial.print(",legal=");            Serial.print(maxDev_legal, 0);
    Serial.print("  MIN_CURRENT_MA=");  Serial.print(MIN_CURRENT_MA);
    Serial.print("  S2_TRIM_MA=");      Serial.println(S2_TRIM_MA);
  }
}

// ─────────────────────────────────────────────────────────────
void handleCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if      (cmd == "BUZZ:1") { digitalWrite(BUZZER_PIN, HIGH); }
  else if (cmd == "BUZZ:0") { digitalWrite(BUZZER_PIN, LOW);  }
  else if (cmd == "RECAL")  {
    digitalWrite(BUZZER_PIN, LOW);
    calibrateBaseline();
    Serial.println("INFO:READY");
  }
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  if (!ads.begin()) {
    Serial.println("ERR:ADS1115_NOT_FOUND");
    while (1) yield();
  }

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("CAL:WAITING_FOR_RAILS");
  for (int i = 5; i > 0; i--) {
    Serial.print("CAL:COUNTDOWN,");
    Serial.println(i);
    delay(1000);   // delay() feeds WDT internally — safe
  }

  calibrateBaseline();
  Serial.println("INFO:READY");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  handleCommands();
  readAndSend();
  yield();   // ← give ESP8266 background tasks a slice every loop
}
