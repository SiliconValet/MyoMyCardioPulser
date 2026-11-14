/**
 * MyoPulser ESP32-C3 Controller Firmware
 * Adapted for ESP32-C3 + L298N H-Bridge Driver
 *
 * Pinout:
 *   GPIO 2 -> L298N IN1 (Polarity Direction A)
 *   GPIO 3 -> L298N IN2 (Polarity Direction B)
 *   GPIO 4 -> L298N ENA (H-Bridge Enable / High-Z Disconnect)
 *   GPIO 8 -> Onboard Status LED (Active LOW on most ESP32-C3 boards)
 *
 * Communication:
 *   USB CDC Serial @ 115200 baud (JSON commands in, JSON telemetry out)
 *   NOTE: In Arduino IDE, set Tools -> "USB CDC On Boot" to "Enabled"!
 */

#include <Arduino.h>

// --- Hardware Pin Definitions ---
const int PIN_IN1 = 2;
const int PIN_IN2 = 3;
const int PIN_ENA = 4;

// Most ESP32-C3 boards (SuperMini / DevKitM) have an active-LOW LED on GPIO 8
int pinLed = 8;
bool ledActiveLow = true;

// --- Pacing Configuration State ---
enum PacingMode { MODE_STOPPED = 0, MODE_NORMAL = 1, MODE_RANDOM = 2 };

enum PulseType {
  TYPE_BIPHASIC = 0,    // Anodic (+V) first half, Cathodic (-V) second half
  TYPE_POLYPHASIC = 1,  // High-frequency AC polarity flipping during pulse
  TYPE_ALTERNATING = 2, // Alternates +V and -V on consecutive pulses
  TYPE_MONOPHASIC =
      3 // Single polarity (+V) - use with caution (electrolysis risk)
};

struct PacerConfig {
  PacingMode mode = MODE_STOPPED;
  PulseType type = TYPE_BIPHASIC;
  uint32_t pulseWidthUs = 10000; // Default: 10 ms (10,000 us)
  uint32_t gapMs = 990; // Default: 990 ms (total period = 1000 ms = 1.0 Hz)
  uint32_t polyHalfPeriodUs =
      500; // Default: 500 us half-period (1 kHz AC in polyphasic mode)
  uint32_t randPulseMinMs = 5;
  uint32_t randPulseMaxMs = 20;
  uint32_t randGapMinMs = 400;
  uint32_t randGapMaxMs = 1200;
};

PacerConfig cfg;

// --- Runtime Counters & State ---
uint64_t totalPulseCount = 0;
uint64_t sessionStartTimeMs = 0;
uint32_t lastPulseTimestampMs = 0;
uint32_t currentTargetGapMs = 990;
uint32_t ledOffTimeMs = 0;
bool alternatingPolarityState = true;
String serialBuffer = "";

// --- LED Visual Indicator Helpers ---
inline void setStatusLed(bool on) {
  if (ledActiveLow) {
    digitalWrite(pinLed, on ? LOW : HIGH);
  } else {
    digitalWrite(pinLed, on ? HIGH : LOW);
  }
}

void triggerLedFlash(uint32_t holdMs = 80) {
  setStatusLed(true);
  ledOffTimeMs = millis() + holdMs;
}

// --- Low-Level H-Bridge Hardware Control ---
inline void setBridgeHighZ() {
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

inline void setBridgeAnodic() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, HIGH);
}

inline void setBridgeCathodic() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_ENA, HIGH);
}

/**
 * Executes a single blocking stimulation pulse with microsecond accuracy.
 * Blocking during the 1-200 ms pulse guarantees zero USB/OS jitter on pulse
 * width.
 */
void fireSinglePulse(uint32_t durationUs) {
  uint32_t startUs = micros();
  uint32_t startMs = millis();

  // Turn LED on at start of pulse
  setStatusLed(true);

  if (cfg.type == TYPE_BIPHASIC) {
    uint32_t halfUs = durationUs / 2;

    // Phase 1: Anodic (+V)
    setBridgeAnodic();
    while ((uint32_t)(micros() - startUs) < halfUs) {
      // Tight microsecond wait
    }

    // Phase 2: Cathodic (-V)
    setBridgeCathodic();
    while ((uint32_t)(micros() - startUs) < durationUs) {
      // Tight microsecond wait
    }
  } else if (cfg.type == TYPE_POLYPHASIC) {
    bool phasePositive = true;
    uint32_t nextFlipUs = cfg.polyHalfPeriodUs;
    setBridgeAnodic();

    while (true) {
      uint32_t elapsedUs = (uint32_t)(micros() - startUs);
      if (elapsedUs >= durationUs)
        break;

      if (elapsedUs >= nextFlipUs) {
        phasePositive = !phasePositive;
        if (phasePositive) {
          setBridgeAnodic();
        } else {
          setBridgeCathodic();
        }
        nextFlipUs += cfg.polyHalfPeriodUs;
      }
    }
  } else if (cfg.type == TYPE_ALTERNATING) {
    if (alternatingPolarityState) {
      setBridgeAnodic();
    } else {
      setBridgeCathodic();
    }
    alternatingPolarityState = !alternatingPolarityState;

    while ((uint32_t)(micros() - startUs) < durationUs) {
      // Tight microsecond wait
    }
  } else if (cfg.type == TYPE_MONOPHASIC) {
    setBridgeAnodic();
    while ((uint32_t)(micros() - startUs) < durationUs) {
      // Tight microsecond wait
    }
  }

  // Immediately disconnect H-bridge to High-Z resting state
  setBridgeHighZ();
  uint32_t actualDurationUs = (uint32_t)(micros() - startUs);

  totalPulseCount++;
  int8_t firedPolarity =
      (cfg.type == TYPE_ALTERNATING) ? (alternatingPolarityState ? -1 : 1) : 1;

  // Emit lightweight JSON telemetry event over USB Serial
  Serial.print("{\"event\":\"pulse\",\"id\":");
  Serial.print((unsigned long)totalPulseCount);
  Serial.print(",\"t_ms\":");
  Serial.print(startMs - (uint32_t)sessionStartTimeMs);
  Serial.print(",\"dur_us\":");
  Serial.print(actualDurationUs);
  Serial.print(",\"gap_ms\":");
  Serial.print(currentTargetGapMs);
  Serial.print(",\"type\":");
  Serial.print((int)cfg.type);
  Serial.print(",\"pol\":");
  Serial.print(firedPolarity);
  Serial.print(",\"poly_us\":");
  Serial.print(cfg.polyHalfPeriodUs);
  Serial.println("}");
  Serial.flush();

  // Hold LED on for 80 ms AFTER the pulse completes so human eye clearly
  // perceives the flash
  ledOffTimeMs = millis() + 80;
}

/**
 * Simple key-value parser for incoming JSON commands over USB Serial.
 * Avoids external JSON library dependencies so it compiles out of the box.
 */
long extractJsonInt(const String &json, const String &key, long fallback) {
  String searchKey = "\"" + key + "\":";
  int idx = json.indexOf(searchKey);
  if (idx == -1)
    return fallback;
  int valStart = idx + searchKey.length();
  while (valStart < (int)json.length() &&
         (json[valStart] == ' ' || json[valStart] == '"')) {
    valStart++;
  }
  return json.substring(valStart).toInt();
}

void sendStatus() {
  Serial.print("{\"event\":\"status\",\"mode\":");
  Serial.print((int)cfg.mode);
  Serial.print(",\"type\":");
  Serial.print((int)cfg.type);
  Serial.print(",\"pulse_us\":");
  Serial.print(cfg.pulseWidthUs);
  Serial.print(",\"gap_ms\":");
  Serial.print(cfg.gapMs);
  Serial.print(",\"poly_us\":");
  Serial.print(cfg.polyHalfPeriodUs);
  Serial.print(",\"count\":");
  Serial.print((unsigned long)totalPulseCount);
  Serial.println("}");
  Serial.flush();
}

void handleCommand(const String &cmd) {
  Serial.println("{\"event\":\"ack\"}");

  if (cmd.indexOf("\"cmd\":\"ping\"") != -1) {
    Serial.print("{\"event\":\"pong\",\"uptime_ms\":");
    Serial.print(millis());
    Serial.print(",\"mode\":");
    Serial.print((int)cfg.mode);
    Serial.print(",\"pulses\":");
    Serial.print((unsigned long)totalPulseCount);
    Serial.println("}");
    Serial.flush();
    return;
  }

  if (cmd.indexOf("\"cmd\":\"led_test\"") != -1) {
    setStatusLed(true);
    ledOffTimeMs = millis() + 1500; // Hold on for 1.5 seconds
    Serial.print("{\"event\":\"led_status\",\"pin\":");
    Serial.print(pinLed);
    Serial.print(",\"active_low\":");
    Serial.print(ledActiveLow ? 1 : 0);
    Serial.println("}");
    Serial.flush();
    return;
  }

  if (cmd.indexOf("\"cmd\":\"led_config\"") != -1) {
    int newPin = extractJsonInt(cmd, "pin", pinLed);
    int newLow = extractJsonInt(cmd, "active_low", ledActiveLow ? 1 : 0);
    setStatusLed(false);
    pinLed = newPin;
    ledActiveLow = (newLow != 0);
    pinMode(pinLed, OUTPUT);
    for (int i = 0; i < 2; i++) {
      setStatusLed(true);
      delay(80);
      setStatusLed(false);
      delay(80);
    }
    Serial.print("{\"event\":\"led_status\",\"pin\":");
    Serial.print(pinLed);
    Serial.print(",\"active_low\":");
    Serial.print(ledActiveLow ? 1 : 0);
    Serial.println("}");
    Serial.flush();
    return;
  }

  if (cmd.indexOf("\"cmd\":\"manual\"") != -1) {
    uint32_t pUs = extractJsonInt(cmd, "pulse_us", cfg.pulseWidthUs);
    pUs = constrain(pUs, 500, 200000); // 0.5 ms to 200 ms safety bounds
    if (sessionStartTimeMs == 0)
      sessionStartTimeMs = millis();
    fireSinglePulse(pUs);
    return;
  }

  if (cmd.indexOf("\"cmd\":\"reset\"") != -1) {
    cfg.mode = MODE_STOPPED;
    setBridgeHighZ();
    totalPulseCount = 0;
    sessionStartTimeMs = millis();
    sendStatus();
    return;
  }

  if (cmd.indexOf("\"cmd\":\"config\"") != -1) {
    int newMode = extractJsonInt(cmd, "mode", (int)cfg.mode);
    int newType = extractJsonInt(cmd, "type", (int)cfg.type);
    long pUs = extractJsonInt(cmd, "pulse_us", cfg.pulseWidthUs);
    long gMs = extractJsonInt(cmd, "gap_ms", cfg.gapMs);
    long polyUs = extractJsonInt(cmd, "poly_us", cfg.polyHalfPeriodUs);
    long rPMin = extractJsonInt(cmd, "rp_min", cfg.randPulseMinMs);
    long rPMax = extractJsonInt(cmd, "rp_max", cfg.randPulseMaxMs);
    long rGMin = extractJsonInt(cmd, "rg_min", cfg.randGapMinMs);
    long rGMax = extractJsonInt(cmd, "rg_max", cfg.randGapMaxMs);

    cfg.type = (PulseType)constrain(newType, 0, 3);
    cfg.pulseWidthUs = constrain(pUs, 500, 200000);
    cfg.gapMs = constrain(gMs, 20, 10000);
    cfg.polyHalfPeriodUs = constrain(polyUs, 30, 5000);
    cfg.randPulseMinMs = constrain(rPMin, 1, 200);
    cfg.randPulseMaxMs = constrain(rPMax, cfg.randPulseMinMs, 200);
    cfg.randGapMinMs = constrain(rGMin, 20, 5000);
    cfg.randGapMaxMs = constrain(rGMax, cfg.randGapMinMs, 5000);

    PacingMode targetMode = (PacingMode)constrain(newMode, 0, 2);
    if (targetMode != MODE_STOPPED && cfg.mode == MODE_STOPPED) {
      if (totalPulseCount == 0)
        sessionStartTimeMs = millis();
      lastPulseTimestampMs =
          millis() - cfg.gapMs; // Trigger first pulse immediately
      currentTargetGapMs = cfg.gapMs;
    }
    cfg.mode = targetMode;

    if (cfg.mode == MODE_STOPPED) {
      setBridgeHighZ();
    }
    sendStatus();
    return;
  }

  if (cmd.indexOf("\"cmd\":\"status\"") != -1) {
    sendStatus();
  }
}

void setup() {
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(pinLed, OUTPUT);
  setBridgeHighZ();
  setStatusLed(false);

  // Triple-blink on boot to verify board power & LED pin clearly (120ms each)
  for (int i = 0; i < 3; i++) {
    setStatusLed(true);
    delay(120);
    setStatusLed(false);
    delay(120);
  }

  // Seed RNG from floating analog pin noise for arrhythmia simulation
  randomSeed(analogRead(0) ^ micros());

  Serial.begin(115200);
  // Brief wait for USB CDC enumeration on ESP32-C3
  delay(500);
  sendStatus();
}

void loop() {
  // Turn off visual LED indicator once hold duration expires
  if (ledOffTimeMs > 0 && (int32_t)(millis() - ledOffTimeMs) >= 0) {
    setStatusLed(false);
    ledOffTimeMs = 0;
  }

  // 1. Read incoming USB Serial commands without blocking
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
      }
      serialBuffer = "";
    } else if (serialBuffer.length() < 256) {
      serialBuffer += c;
    }
  }

  // 2. Execute scheduled pacing if active
  if (cfg.mode == MODE_NORMAL) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastPulseTimestampMs) >= currentTargetGapMs) {
      fireSinglePulse(cfg.pulseWidthUs);
      lastPulseTimestampMs = millis();
      currentTargetGapMs = cfg.gapMs;
    }
  } else if (cfg.mode == MODE_RANDOM) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastPulseTimestampMs) >= currentTargetGapMs) {
      uint32_t randPulseUs =
          random(cfg.randPulseMinMs, cfg.randPulseMaxMs + 1) * 1000UL;
      fireSinglePulse(randPulseUs);
      lastPulseTimestampMs = millis();
      currentTargetGapMs = random(cfg.randGapMinMs, cfg.randGapMaxMs + 1);
    }
  } else {
    // In stopped / idle mode, emit a periodic heartbeat every 3 seconds to
    // confirm loop activity
    static uint32_t lastIdleHeartbeatMs = 0;
    if ((uint32_t)(millis() - lastIdleHeartbeatMs) >= 3000) {
      lastIdleHeartbeatMs = millis();
      Serial.print("{\"event\":\"heartbeat\",\"uptime_ms\":");
      Serial.print(millis());
      Serial.println("}");
      Serial.flush();
    }
  }
}