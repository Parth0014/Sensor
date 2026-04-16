#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define BAUD_RATE       115200

// ── Sampling ─────────────────────────────────────────────
#define SAMPLES         80
#define CAL_SAMPLES     400

// ── ADC + Sensor Config ─────────────────────────────────
#define ADC_LSB_MV      0.0625f    // GAIN_TWO → 0.0625 mV per count
#define SENSITIVITY     70.0f     // ACS712-20A → 100 mV/A
#define DIVIDER_FACTOR  2.0f       // 10k + 10k divider

// ── Power ───────────────────────────────────────────────
#define MAINS_VOLTS     230.0f

// ── Noise + tuning ─────────────────────────────────────
#define MIN_CURRENT_MA  20.0f
#define S2_TRIM_MA      0.0f

#define CH_TOTAL        0
#define CH_LEGAL        1
#define YIELD_EVERY     50

Adafruit_ADS1115 ads;

float baseline_total = 0.0f;
float baseline_legal = 0.0f;

// ───────────────────────────────────────────────────────
void calibrateBaseline() {
  Serial.println("CAL:STARTED");

  for (int i = 0; i < 50; i++) {
    ads.readADC_SingleEnded(CH_TOTAL);
    ads.readADC_SingleEnded(CH_LEGAL);
    if (i % 10 == 0) yield();
    delay(5);
  }

  double sum_total = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum_total += ads.readADC_SingleEnded(CH_TOTAL);
    delayMicroseconds(1200);
    if (i % YIELD_EVERY == 0) yield();
  }
  baseline_total = sum_total / CAL_SAMPLES;

  double sum_legal = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum_legal += ads.readADC_SingleEnded(CH_LEGAL);
    delayMicroseconds(1200);
    if (i % YIELD_EVERY == 0) yield();
  }
  baseline_legal = sum_legal / CAL_SAMPLES;

  Serial.print("CAL:BASELINE,total=");
  Serial.print(baseline_total, 2);
  Serial.print(",legal=");
  Serial.println(baseline_legal, 2);
}

// ───────────────────────────────────────────────────────
void readAndSend() {

  double sq_total = 0, sq_legal = 0;

  for (int i = 0; i < SAMPLES; i++) {
    float v_total = ads.readADC_SingleEnded(CH_TOTAL) - baseline_total;
    float v_legal = ads.readADC_SingleEnded(CH_LEGAL) - baseline_legal;

    sq_total += v_total * v_total;
    sq_legal += v_legal * v_legal;

    delayMicroseconds(800);
  }

  float rms_counts_total = sqrt(sq_total / SAMPLES);
  float rms_counts_legal = sqrt(sq_legal / SAMPLES);

  // ── Convert counts → mV ───────────────────────────────
  float rms_mV_total = rms_counts_total * ADC_LSB_MV;
  float rms_mV_legal = rms_counts_legal * ADC_LSB_MV;

  // ── Convert mV → Current ──────────────────────────────
  float current_total_mA = (rms_mV_total / SENSITIVITY) * 1000.0f * DIVIDER_FACTOR;
  float current_legal_mA = (rms_mV_legal / SENSITIVITY) * 1000.0f * DIVIDER_FACTOR;

  // ── Noise filter ──────────────────────────────────────
  if (current_total_mA < MIN_CURRENT_MA) current_total_mA = 0;
  if (current_legal_mA < MIN_CURRENT_MA) current_legal_mA = 0;

  // ── Trim mismatch ─────────────────────────────────────
  current_legal_mA = max(0.0f, current_legal_mA - S2_TRIM_MA);

  // ── Power calculation ─────────────────────────────────
  float watts_total = (current_total_mA / 1000.0f) * MAINS_VOLTS;
  float watts_legal = (current_legal_mA / 1000.0f) * MAINS_VOLTS;

  float theft_mA = max(0.0f, current_total_mA - current_legal_mA);
  float theft_W  = (theft_mA / 1000.0f) * MAINS_VOLTS;

  // ── Output ────────────────────────────────────────────
  Serial.print("DATA,s1_mA="); Serial.print(current_total_mA, 1);
  Serial.print(",s2_mA=");     Serial.print(current_legal_mA, 1);
  Serial.print(",s1_W=");      Serial.print(watts_total, 1);
  Serial.print(",s2_W=");      Serial.print(watts_legal, 1);
  Serial.print(",theft_mA=");  Serial.print(theft_mA, 1);
  Serial.print(",theft_W=");   Serial.println(theft_W, 1);
}

// ───────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  ads.setGain(GAIN_TWO);   // 🔥 correct for small signal
  ads.setDataRate(RATE_ADS1115_860SPS);

  if (!ads.begin()) {
    Serial.println("ERR:ADS1115_NOT_FOUND");
    while (1) yield();
  }

  Serial.println("CAL:WAITING_FOR_RAILS");

  for (int i = 5; i > 0; i--) {
    Serial.print("CAL:COUNTDOWN,");
    Serial.println(i);
    delay(1000);
  }

  calibrateBaseline();
  Serial.println("INFO:READY");
}

// ───────────────────────────────────────────────────────
void loop() {
  readAndSend();
  yield();
}