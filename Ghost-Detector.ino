#include <Wire.h>
#define SSD1306_NO_SPLASH
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MMC56x3.h>

Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

// SSD1306 display (I2C)
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr int8_t OLED_RESET = -1;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- TUNING ----------------
// Startup calibration time
const uint16_t CAL_SAMPLES = 250;        // 250 * 20ms = 5s

// Filtering
const float ALPHA_MAG   = 0.25f;         // smooth raw magnitude
const float ALPHA_BASE  = 0.008f;        // baseline drift rate (only when calm)
const float ALPHA_NOISE = 0.02f;         // noise estimator smoothing (only when calm)

// Trigger behavior
const float MIN_TRIP_uT   = 10.0f;       // hard minimum trigger threshold
const float K_NOISE       = 6.0f;        // dynamic threshold multiplier (noise * K)
const float RELEASE_FRAC  = 0.45f;       // re-arm when below RELEASE_FRAC * trip
const uint8_t HIT_COUNT   = 3;           // consecutive samples needed to trigger
const uint16_t PEAK_HOLD_MS = 350;       // record peak for this long after trigger
const uint16_t COOLDOWN_MS  = 900;       // ignore new triggers during cooldown

// Strong magnet detection (special event)
const float SAT_uT = 900.0f;             // if magnitude exceeds this, call it "MAGNET"
const uint16_t SAT_COOLDOWN_MS = 1200;   // prevent SAT spam while magnet stays nearby

// Loop timing
const uint16_t LOOP_MS = 40;

// -------------- STATE -------------------
float baselineMag = 0.0f;
float filtMag = 0.0f;

// "Noise" is typical absolute delta in calm conditions (uT)
float noise_uT = 2.0f;

uint8_t hits = 0;
bool armed = true;

unsigned long t_trigger = 0;
unsigned long t_lastEvent = 0;
unsigned long t_lastSat = 0;

// Peak hold values
float peakMag = 0.0f;
float peakDelta = 0.0f;

// Utility
static float mag_uT(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

static int levelFromScore(float score) {
  if (score < 0.0f) return 0;
  if (score < 10.0f) return 1;
  if (score < 25.0f) return 2;
  if (score < 45.0f) return 3;
  if (score < 70.0f) return 4;
  return 5;
}

static void drawGauge(int level, bool armed, bool inCooldown, bool inMagnet) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("EMF DETECTOR"));

  display.setCursor(0, 12);
  if (inMagnet) {
    display.print(F("ALERT: MAGNET"));
  } else if (inCooldown) {
    display.print(F("STATE: COOLDOWN"));
  } else if (armed) {
    display.print(F("STATE: ARMED"));
  } else {
    display.print(F("STATE: HOLD"));
  }

  display.setCursor(0, 24);
  display.print(F("LEVEL "));
  display.print(level);

  const int barX = 0;
  const int barY = 40;
  const int barW = 128;
  const int barH = 20;
  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);

  const int fill = map(level, 0, 5, 0, barW - 2);
  if (fill > 0) {
    display.fillRect(barX + 1, barY + 1, fill, barH - 2, SSD1306_WHITE);
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed."));
    while (1) {}
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("EMF GHOST DETECTOR"));
  display.setCursor(0, 12);
  display.println(F("Calibrating..."));
  display.display();

  if (!mmc.begin()) {
    Serial.println(F("MMC5603 not found."));
    while (1) {}
  }

  // Calibration: hold still
  float sum = 0.0f;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sensors_event_t e;
    mmc.getEvent(&e);
    float m = mag_uT(e.magnetic.x, e.magnetic.y, e.magnetic.z);
    sum += m;
    delay(20);
  }

  baselineMag = sum / CAL_SAMPLES;
  filtMag = baselineMag;
  noise_uT = 2.0f;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibration done");
  display.display();
  delay(800);

  // Silent startup: prints only on events
}

void loop() {
  const unsigned long now = millis();

  sensors_event_t e;
  mmc.getEvent(&e);

  // Smooth magnitude
  const float m = mag_uT(e.magnetic.x, e.magnetic.y, e.magnetic.z);
  filtMag = (1.0f - ALPHA_MAG) * filtMag + ALPHA_MAG * m;

  // Delta vs baseline
  const float delta = filtMag - baselineMag;
  const float adelta = fabsf(delta);

  // Compute dynamic thresholds using current noise estimate
  float trip = noise_uT * K_NOISE;
  if (trip < MIN_TRIP_uT) trip = MIN_TRIP_uT;
  const float release = trip * RELEASE_FRAC;

  // ---------------- STRONG MAGNET EVENT ----------------
  // If you shove a proper magnet near it, we WANT an event, not silence.
  // We also do NOT update baseline/noise during this condition.
  const bool inMagnet = (filtMag >= SAT_uT);
  if (inMagnet) {
    // prevent spamming while the magnet is held in place
    if (now - t_lastSat >= SAT_COOLDOWN_MS) {
      Serial.print(F("MAGNET t="));
      Serial.print(now);
      Serial.print(F("ms  mag="));
      Serial.print(filtMag, 2);
      Serial.print(F("uT  delta="));
      Serial.print(delta, 2);
      Serial.println(F("uT"));

      t_lastSat = now;

      // Latch into a cooldown-like state so it doesn't re-trigger instantly
      armed = false;
      hits = 0;
      t_lastEvent = now;

      // Peak tracking for this "event"
      peakMag = filtMag;
      peakDelta = delta;
      t_trigger = now;
    }

    // Still allow peak hold accumulation below
    // (but do not learn baseline/noise)
  } else {
    // Learn noise only when calm (prevents spikes inflating noise)
    if (adelta < release) {
      noise_uT = (1.0f - ALPHA_NOISE) * noise_uT + ALPHA_NOISE * adelta;
      if (noise_uT < 0.5f) noise_uT = 0.5f;
    }

    // Update baseline only when calm (prevents spikes poisoning baseline)
    if (adelta < release) {
      baselineMag = (1.0f - ALPHA_BASE) * baselineMag + ALPHA_BASE * filtMag;
    }
  }

  // Cooldown window after an event
  const bool inCooldown = (now - t_lastEvent) < COOLDOWN_MS;

  // Peak-hold collection after trigger
  const bool inPeakHold = (t_trigger != 0) && ((now - t_trigger) < PEAK_HOLD_MS);
  if (inPeakHold) {
    if (filtMag > peakMag) peakMag = filtMag;
    if (fabsf(delta) > fabsf(peakDelta)) peakDelta = delta;
  }

  // Re-arm logic (only when calm and not in cooldown/peakhold)
  if (!armed) {
    if (adelta < release && !inCooldown && !inPeakHold) {
      armed = true;
      hits = 0;
      t_trigger = 0;
    }
  }

  // Detection logic (normal anomalies, only when armed and not in cooldown)
  if (armed && !inCooldown && filtMag < SAT_uT) {
    if (adelta >= trip) {
      if (hits < 255) hits++;
    } else {
      if (hits > 0) hits--;
    }

    if (hits >= HIT_COUNT) {
      // Trigger an event
      armed = false;
      t_lastEvent = now;
      t_trigger = now;

      // Seed peak-hold
      peakMag = filtMag;
      peakDelta = delta;

      // "Spook score": how far beyond threshold we went
      const float score = adelta - trip;
      const int level = levelFromScore(score);

      Serial.print(F("ANOMALY t="));
      Serial.print(now);
      Serial.print(F("ms  level="));
      Serial.print(level);
      Serial.print(F("  mag="));
      Serial.print(filtMag, 2);
      Serial.print(F("uT  delta="));
      Serial.print(delta, 2);
      Serial.print(F("uT  trip="));
      Serial.print(trip, 2);
      Serial.print(F("uT  noise="));
      Serial.print(noise_uT, 2);
      Serial.println(F("uT"));
    }
  }

  // After peak-hold finishes, print a single follow-up peak line
  if (t_trigger != 0 && (now - t_trigger) >= PEAK_HOLD_MS) {
    Serial.print(F("PEAK   t="));
    Serial.print(now);
    Serial.print(F("ms  mag="));
    Serial.print(peakMag, 2);
    Serial.print(F("uT  delta="));
    Serial.print(peakDelta, 2);
    Serial.println(F("uT"));
    t_trigger = 0;
  }

  const float score = adelta - trip;
  const int level = levelFromScore(score);
  drawGauge(level, armed, inCooldown, inMagnet);

  delay(LOOP_MS);
}
