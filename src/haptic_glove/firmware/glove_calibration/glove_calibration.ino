// =============================================================================
// glove_calibration.ino
//
// Standalone calibration sketch for the haptic glove.
// For each finger, sweeps the servo to 0° then 180°, reads the pot at each
// end, and prints the results to Serial so they can be hardcoded into firmware.
//
// Wiring assumptions — edit the pin arrays below to match your board:
//   SERVO_PIN[i]  — PWM output to servo i
//   POT_PIN[i]    — analog input from pot i
//
// Flash this sketch, open Serial Monitor at 115200 baud, and follow prompts.
// =============================================================================

#include <Servo.h>

// ── Pin configuration ─────────────────────────────────────────────────────────
// Edit these to match your glove's wiring.
const int SERVO_PIN[5] = { 2,  4,  6,  8, 10 };  // thumb → pinky
const int POT_PIN[5]   = {A0, A1, A2, A3, A4 };  // thumb → pinky

const char* FINGER_NAME[5] = { "thumb", "index", "middle", "ring", "pinky" };

// ── Timing ────────────────────────────────────────────────────────────────────
#define SETTLE_MS  600    // time to wait after writing servo before reading pot
#define SAMPLE_N    20    // number of ADC samples to average per reading
#define SAMPLE_MS   10    // delay between samples (ms)

// ── Globals ───────────────────────────────────────────────────────────────────
Servo servos[5];

int potAt0[5];
int potAt180[5];

// ── Helpers ───────────────────────────────────────────────────────────────────

int readPotAvg(int pin) {
    long sum = 0;
    for (int i = 0; i < SAMPLE_N; i++) {
        sum += analogRead(pin);
        delay(SAMPLE_MS);
    }
    return (int)(sum / SAMPLE_N);
}

void writeAndSettle(Servo& s, int angle) {
    s.write(angle);
    delay(SETTLE_MS);
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    for (int i = 0; i < 5; i++) {
        servos[i].attach(SERVO_PIN[i]);
    }

    Serial.println("==============================================");
    Serial.println("  HAPTIC GLOVE SERVO-POT CALIBRATION");
    Serial.println("==============================================");
    Serial.println("  Remove glove from hand before proceeding.");
    Serial.println("  Servos will sweep 0 -> 180 -> 0 for each finger.");
    Serial.println("  Pot values are printed when servo has settled.");
    Serial.println();
    Serial.println("  Starting in 3 seconds...");
    delay(3000);

    // ── Calibrate each finger ─────────────────────────────────────────────────
    for (int i = 0; i < 5; i++) {
        Serial.print("  Finger ");
        Serial.print(i);
        Serial.print(" (");
        Serial.print(FINGER_NAME[i]);
        Serial.println("):");

        // 0°
        writeAndSettle(servos[i], 0);
        potAt0[i] = readPotAvg(POT_PIN[i]);
        Serial.print("    servo=0    pot=");
        Serial.println(potAt0[i]);

        // 180°
        writeAndSettle(servos[i], 180);
        potAt180[i] = readPotAvg(POT_PIN[i]);
        Serial.print("    servo=180  pot=");
        Serial.println(potAt180[i]);

        // Return to 0°
        writeAndSettle(servos[i], 0);
        Serial.println("    returned to 0°");
        Serial.println();
    }

    // ── Print copy-paste block ────────────────────────────────────────────────
    Serial.println("==============================================");
    Serial.println("  CALIBRATION COMPLETE — copy into firmware:");
    Serial.println("==============================================");
    Serial.println();

    Serial.print("const int POT_AT_0[5]   = { ");
    for (int i = 0; i < 5; i++) {
        Serial.print(potAt0[i]);
        if (i < 4) Serial.print(", ");
    }
    Serial.println(" };  // thumb, index, middle, ring, pinky");

    Serial.print("const int POT_AT_180[5] = { ");
    for (int i = 0; i < 5; i++) {
        Serial.print(potAt180[i]);
        if (i < 4) Serial.print(", ");
    }
    Serial.println(" };  // thumb, index, middle, ring, pinky");

    Serial.println();
    Serial.println("  Done. You can now flash the main firmware.");
}

void loop() {
    // nothing — calibration runs once in setup()
}
