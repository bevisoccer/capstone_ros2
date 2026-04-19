// =============================================================================
// lock_at_position.ino  —  snippet to add to the main haptic glove firmware
//
// Paste the constants (from calibration output) and the function below
// into your main sketch.  Call lockAtCurrentPosition() whenever a
// haptic "lock" command arrives (e.g. servo_cmd value == 0).
// =============================================================================

// ── Calibration constants — replace with values from glove_calibration output ─
// Order: thumb, index, middle, ring, pinky
const int POT_AT_0[5]   = { 0, 0, 0, 0, 0 };    // pot reading when servo=0°
const int POT_AT_180[5] = { 1023, 1023, 1023, 1023, 1023 };  // pot reading when servo=180°

// ── Pin configuration — must match main firmware ──────────────────────────────
extern Servo servos[5];        // already defined in main sketch
extern const int POT_PIN[5];   // already defined in main sketch

// ── Lock each finger at its current physical position ─────────────────────────
//
// currentPotReadings[i]  — live analogRead() for finger i (call just before
//                          invoking this function so readings are fresh)
//
// For each finger, maps the current pot value back to the servo angle that
// corresponds to that same physical position and writes it.  The spool stops
// moving and the string holds without yanking tighter.
//
void lockAtCurrentPosition(int currentPotReadings[5]) {
    for (int i = 0; i < 5; i++) {
        int pot0   = POT_AT_0[i];
        int pot180 = POT_AT_180[i];

        // Guard: if calibration range is too small, skip (sensor not wired)
        if (abs(pot180 - pot0) < 20) continue;

        // Map current pot reading → servo angle, clamped to 0–180
        int angle = (int)map(currentPotReadings[i], pot0, pot180, 0, 180);
        angle = constrain(angle, 0, 180);

        servos[i].write(angle);
    }
}

// ── Example call site ─────────────────────────────────────────────────────────
//
// In your BLE receive handler, when a lock command arrives:
//
//   int currentPot[5];
//   for (int i = 0; i < 5; i++) currentPot[i] = analogRead(POT_PIN[i]);
//   lockAtCurrentPosition(currentPot);
