// ─────────────────────────────────────────────────────────────
//  PowerGuard ESP32 — DEBUG VERSION
//  Prints raw ADC counts, baseline, and offset so we can see
//  exactly why S1/S2 are non-zero with no load.
// ─────────────────────────────────────────────────────────────

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define BUZZER_PIN   14
#define BAUD_RATE    115200
#define SAMPLES      80
#define CAL_SAMPLES  400

// ── Noise clamp in ADC counts ─────────────────────────────────
// ACS712 5A + ADS1115 GAIN_ONE: 1 count = 0.125 mV
// Real signal from a 100W bulb at 230V:
//   I_rms = 100W/230V = 435 mA
//   Sensor output swing = 435mA × 185mV/A = 80mV RMS
//   In counts = 80mV / 0.125mV = 640 counts RMS
//   Peak amplitude = 640 × √2 ≈ 905 counts
// So NOISE_CLAMP must be < 905 to let real signal through,
// but high enough to kill noise. Start at 200 and tune up if
// you still see false readings with no load.
#define NOISE_CLAMP  200

Adafruit_ADS1115 ads;
float baseline1 = 0.0;
float baseline2 = 0.0;

// ─────────────────────────────────────────────────────────────
void calibrateBaseline() {
  Serial.println("CAL:STARTED");

  // Warmup
  for (int i = 0; i < 50; i++) {
    ads.readADC_SingleEnded(0);
    ads.readADC_SingleEnded(1);
    if (i % 10 == 0) Serial.println("CAL:WARMUP");
    delay(5);
  }

  // Print a few raw samples so we can see the actual ADC values
  Serial.println("DBG:RAW_SAMPLES_BEFORE_CAL");
  for (int i = 0; i < 5; i++) {
    int16_t r0 = ads.readADC_SingleEnded(0);
    int16_t r1 = ads.readADC_SingleEnded(1);
    Serial.print("DBG:RAW,ch0="); Serial.print(r0);
    Serial.print(",ch1="); Serial.println(r1);
    delay(10);
  }

  double sum1 = 0, sum2 = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum1 += ads.readADC_SingleEnded(1);
    sum2 += ads.readADC_SingleEnded(0);
    delay(2);
  }

  baseline1 = sum1 / CAL_SAMPLES;
  baseline2 = sum2 / CAL_SAMPLES;

  // ── DEBUG: print baseline counts and their voltage equivalents ──
  // Expected: baseline should be near 20000 counts (= 2.5V at GAIN_ONE)
  // If you see very different values (e.g. 16000 or 24000), the sensor
  // DC output is shifted — check 5V supply and ACS712 wiring.
  Serial.print("DBG:BASELINE_COUNTS,b1=");
  Serial.print(baseline1, 1);
  Serial.print(",b2=");
  Serial.println(baseline2, 1);

  float v1 = baseline1 * 0.125 / 1000.0;
  float v2 = baseline2 * 0.125 / 1000.0;
  Serial.print("DBG:BASELINE_VOLTS,v1=");
  Serial.print(v1, 3);
  Serial.print("V,v2=");
  Serial.print(v2, 3);
  Serial.println("V  (expected ~2.500V each)");

  Serial.print("CAL:BASELINE,");
  Serial.print(baseline1, 2);
  Serial.print(",");
  Serial.println(baseline2, 2);
}

// ─────────────────────────────────────────────────────────────
void readAndSend() {
  double sq1 = 0, sq2 = 0;

  // Also track max deviation seen this batch for debug
  float maxDev1 = 0, maxDev2 = 0;

  for (int i = 0; i < SAMPLES; i++) {
    float v1 = ads.readADC_SingleEnded(1) - baseline1;
    float v2 = ads.readADC_SingleEnded(0) - baseline2;

    // Track max deviation (before clamp) to help tune NOISE_CLAMP
    if (abs(v1) > maxDev1) maxDev1 = abs(v1);
    if (abs(v2) > maxDev2) maxDev2 = abs(v2);

    if (abs(v1) < NOISE_CLAMP) v1 = 0;
    if (abs(v2) < NOISE_CLAMP) v2 = 0;

    sq1 += v1 * v1;
    sq2 += v2 * v2;
    delayMicroseconds(800);
  }

  float rms1 = sqrt(sq1 / SAMPLES);
  float rms2 = sqrt(sq2 / SAMPLES);

  Serial.print("RAW,");
  Serial.print(rms1, 2);
  Serial.print(",");
  Serial.println(rms2, 2);

  // Every 10 readings print a debug line showing max deviations
  // This tells you the ACTUAL noise amplitude vs NOISE_CLAMP=200
  // If maxDev1 > 200 with no load, raise NOISE_CLAMP
  // If maxDev1 < 200 with load on, lower NOISE_CLAMP
  static int dbgCount = 0;
  if (++dbgCount >= 10) {
    dbgCount = 0;
    Serial.print("DBG:MAX_DEV,d1=");
    Serial.print(maxDev1, 0);
    Serial.print(",d2=");
    Serial.print(maxDev2, 0);
    Serial.print("  NOISE_CLAMP=");
    Serial.println(NOISE_CLAMP);
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
    delay(1000);
  }

  calibrateBaseline();
  Serial.println("INFO:READY");
}

// ─────────────────────────────────────────────────────────────
void loop() {
  handleCommands();
  readAndSend();
}
