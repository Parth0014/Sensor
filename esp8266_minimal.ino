#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ─── Pin & Comms ───────────────────────────────────────────
#define BUZZER_PIN  14
#define BAUD_RATE   115200

// ─── Sampling ──────────────────────────────────────────────
#define SAMPLES       80      // slightly reduced (faster + stable)
#define CAL_SAMPLES   400     // better baseline accuracy

// ─── Micro Noise Clamp ─────────────────────────────────────
#define NOISE_CLAMP   3       // ADC units (~0.4mV)

// ─── Timing ───────────────────────────────────────────────
#define SAMPLE_DELAY_US 800   // stabilizes ADS1115 sampling
#define CAL_DELAY_MS    2

Adafruit_ADS1115 ads;

float baseline1 = 0.0;
float baseline2 = 0.0;

// ──────────────────────────────────────────────────────────
// 🔹 Calibration
// ──────────────────────────────────────────────────────────
void calibrateBaseline() {
  Serial.println("CAL:STARTED");

  // Warmup (VERY important)
  for (int i = 0; i < 100; i++) {
    ads.readADC_SingleEnded(1);
    ads.readADC_SingleEnded(0);
    delay(2);
    yield();
  }

  double sum1 = 0.0, sum2 = 0.0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum1 += ads.readADC_SingleEnded(1);
    sum2 += ads.readADC_SingleEnded(0);
    delay(CAL_DELAY_MS);
    yield();
  }

  baseline1 = sum1 / CAL_SAMPLES;
  baseline2 = sum2 / CAL_SAMPLES;

  Serial.print("CAL:BASELINE,");
  Serial.print(baseline1, 2);
  Serial.print(",");
  Serial.println(baseline2, 2);
}

// ──────────────────────────────────────────────────────────
// 🔹 RMS Reading
// ──────────────────────────────────────────────────────────
void readAndSend() {
  double sq1 = 0.0, sq2 = 0.0;

  for (int i = 0; i < SAMPLES; i++) {
    float raw1 = ads.readADC_SingleEnded(1);
    float raw2 = ads.readADC_SingleEnded(0);

    float v1 = raw1 - baseline1;
    float v2 = raw2 - baseline2;

    // ✅ micro-noise removal BEFORE squaring
    if (abs(v1) < NOISE_CLAMP) v1 = 0;
    if (abs(v2) < NOISE_CLAMP) v2 = 0;

    sq1 += v1 * v1;
    sq2 += v2 * v2;

    delayMicroseconds(SAMPLE_DELAY_US);
    yield();
  }

  float rms1 = sqrt(sq1 / SAMPLES);
  float rms2 = sqrt(sq2 / SAMPLES);

  Serial.print("RAW,");
  Serial.print(rms1, 2);
  Serial.print(",");
  Serial.println(rms2, 2);
}

// ──────────────────────────────────────────────────────────
// 🔹 Serial Commands
// ──────────────────────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "BUZZ:1") {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("ACK:BUZZ:1");

  } else if (cmd == "BUZZ:0") {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("ACK:BUZZ:0");

  } else if (cmd == "RECAL") {
    baseline1 = 0;
    baseline2 = 0;
    calibrateBaseline();
    Serial.println("INFO:RECAL_DONE");
  }
}

// ──────────────────────────────────────────────────────────
// 🔹 Setup
// ──────────────────────────────────────────────────────────
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

  delay(3000);  // full stabilization

  calibrateBaseline();
  Serial.println("INFO:READY");
}

// ──────────────────────────────────────────────────────────
// 🔹 Loop
// ──────────────────────────────────────────────────────────
void loop() {
  handleSerialCommands();
  readAndSend();
}