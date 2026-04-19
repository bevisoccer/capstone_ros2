// =============================================================================
// hand_control_v7.ino — Robotic Hand Controller
// Teensy 4.1 · PCA9685 (14x MG90S servos) · 5x INA219 · 4x FSR (5th pending)
// =============================================================================
//
// LIBRARIES REQUIRED:
//   "Adafruit PWM Servo Driver Library"
//   "Adafruit INA219"
//   "Adafruit BusIO"
//
// HARDWARE / I2C WIRING
// ─────────────────────────────────────────────────────────────────────────────
//  Bus    Teensy pins        Devices
//  Wire1  SDA=18, SCL=19    4x INA219 current sensors (INA ch 0–3)
//  Wire   SDA=17, SCL=16    1x INA219 (INA ch 4)  +  PCA9685 servo driver
//
//  INA219 address table:
//    INA ch | finger | bus   | addr | A1    A0
//    -------+--------+-------+------+-------+------
//       0   | ring   | Wire1 | 0x40 | GND   GND
//       1   | middle | Wire1 | 0x41 | GND   VS+
//       2   | index  | Wire1 | 0x44 | VS+   GND
//       3   | pinky  | Wire1 | 0x45 | VS+   VS+
//       4   | thumb  | Wire  | 0x41 | GND   VS+
//
//  PCA9685 on Wire at 0x40
//
// SERVO CHANNEL MAP
// ─────────────────────────────────────────────────────────────────────────────
//  CH  | Finger | Joint           | INA
//   0  | —      | UNUSED          | —
//   1  | —      | UNUSED          | —
//   2  | Pinky  | 2nd curvature   | —
//   3  | Pinky  | rotation        | —
//   4  | Ring   | 2nd curvature   | —
//   5  | Ring   | rotation        | —
//   6  | Middle | 2nd curvature   | —
//   7  | Middle | rotation        | —
//   8  | Index  | 2nd curvature   | —
//   9  | Index  | rotation        | —
//  10  | Thumb  | rotation        | —
//  11  | Thumb  | 1st curvature   | INA #4
//  12  | Index  | 1st curvature   | INA #2
//  13  | Middle | 1st curvature   | INA #1
//  14  | Ring   | 1st curvature   | INA #0
//  15  | Pinky  | 1st curvature   | INA #3
//
// FSR PIN MAP
// ─────────────────────────────────────────────────────────────────────────────
//  FSR | Finger | Pin       | Status
//   0  | thumb  | 15 (A1)   | active
//   1  | index  | 14 (A0)   | active
//   2  | middle | 41 (A17)  | active
//   3  | ring   | 40 (A16)  | active
//   4  | pinky  | — (TBD)   | PENDING — not yet wired
//
//  Wiring per FSR: 3V3 → FSR → analog pin → 10kΩ → GND
//
// SERIAL PROTOCOL (115200 baud, newline-terminated)
// ─────────────────────────────────────────────────────────────────────────────
// DESIGN PRINCIPLES
//   All response lines follow one of two formats:
//     Machine lines:  TYPE key=val key=val ...   (no spaces in values)
//     Human lines:    INFO free text
//   Machine lines always start with a TYPE token (all-caps, no spaces).
//   Values never contain spaces — safe to split on whitespace.
//   INFO lines are for humans only; parsers should skip them.
//
// ── COMMANDS (host → Teensy) ─────────────────────────────────────────────────
//   MOVE <ch> <angle>     gradual move ch 2–15 to angle 0–180
//   HOME                  move ALL active channels to INIT_ANGLE gradually
//   HOME <ch>             move one channel to its INIT_ANGLE gradually
//   STOP <ch>             disable one channel immediately (holds position)
//   ESTOP                 disable ALL channels immediately
//   RESUME <ch>           re-enable stopped channel; resume last target
//   RESUME_ALL            re-enable all channels; clear e-stop
//   CURVATURE <angle>     move all curvature joints one by one (thumb→index→middle→ring→pinky)
//   ROTATE <angle>        move all rotation joints one by one (thumb→index→middle→ring→pinky)
//   CURVATURE_SYNC <angle> move all curvature joints simultaneously
//   ROTATE_SYNC <angle>    move all rotation joints simultaneously
//   MOVEALL <angle>        move ALL joints (curvature + rotation) simultaneously
//   MOVESET ch:a [ch:a ...] move named channels simultaneously, each to its angle
//                           e.g.  MOVESET 11:90 12:45 8:30 13:60  (up to 14 pairs)
//   SPEED <dps>           slew rate deg/sec (5–720, default 60)
//   READ                  one-shot read all 5 INA219 sensors
//   STREAM <hz>           continuous INA read (1–100 Hz, default 10)
//   INOSTOP               stop INA stream
//   FSRREAD               one-shot scaled read all FSRs
//   FSRRAW                one-shot raw ADC read all FSRs
//   FSRSTREAM <hz>        continuous FSR read (1–50 Hz, default 10)
//   FSRSTOP               stop FSR stream
//   FSRFORMAT PCT|SCALE   FSR output format (default SCALE 0–1000)
//   SCAN                  I2C bus scan
//   STATUS                full snapshot: all servo + INA + FSR state
//   HELP                  re-print command list
//
// ── RESPONSES (Teensy → host) ────────────────────────────────────────────────
//   READY       ina=N/5 servos=14 fsr=N/5
//   MOVE_START  ch=N label=L from=F to=T
//   MOVE_DONE   ch=N label=L angle=A
//   HOME_START  channels=N
//   HOME_DONE
//   SEQ_START  cmd=CURVATURE|ROTATE angle=A joints=N
//   SEQ_DONE   cmd=CURVATURE|ROTATE angle=A
//   SYNC_START cmd=CURVATURE_SYNC|ROTATE_SYNC|MOVEALL joints=N
//   MOVESET_START joints=N
//   MOVESET_DONE
//   STOP        ch=N label=L angle=A
//   ESTOP       channels=N
//   RESUME      ch=N label=L target=T
//   RESUME_ALL
//   SPEED       dps=D
//   INA         ch=N label=L addr=0xAA bus=B servo_ch=S bus_v=V shunt_mv=M current_ma=C power_mw=P
//   INA_MISSING ch=N label=L addr=0xAA bus=B
//   INA_STREAM  hz=N
//   INA_STREAM_STOP
//   FSR         ch=N label=L finger=F raw=R scaled=S pct=P pressed=0|1 pending=0|1
//   FSR_STREAM  hz=N
//   FSR_STREAM_STOP
//   SCAN        bus=B found=N addr=0xAA,...
//   STATUS_SERVO   ch=N label=L angle=A target=T init=I min=X max=Y enabled=0|1 stopped=0|1 ina_ch=N
//   STATUS_INA     ch=N label=L addr=0xAA bus=B servo_ch=S found=0|1
//   STATUS_FSR     ch=N label=L finger=F pin=P raw=R scaled=S pct=P pressed=0|1 pending=0|1
//   STATUS_SYSTEM  estop=0|1 speed_dps=D ina_stream=0|1 ina_hz=N fsr_stream=0|1 fsr_hz=N
//   ERR         msg=<text>
//   INFO        <free text — for humans, ignore in parsers>
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>

// =============================================================================
// ── SERVO CONFIGURATION ──────────────────────────────────────────────────────
// =============================================================================

#define NUM_CHANNELS     16
#define SERVO_FREQ       50
#define SERVO_MIN_US     600
#define SERVO_MAX_US     2400
#define PCA9685_OSC_FREQ 27000000

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire);

const int8_t SERVO_INA_CH[NUM_CHANNELS] = {
  -1, -1,   //  0–1  unused
  -1, -1,   //  2–3  pinky   curve2, rotate
  -1, -1,   //  4–5  ring    curve2, rotate
  -1, -1,   //  6–7  middle  curve2, rotate
  -1, -1,   //  8–9  index   curve2, rotate
  -1,       // 10    thumb   rotate
   4,       // 11    thumb   curve1 → INA #4
   2,       // 12    index   curve1 → INA #2
   1,       // 13    middle  curve1 → INA #1
   0,       // 14    ring    curve1 → INA #0
   3,       // 15    pinky   curve1 → INA #3
};

const char* CH_LABEL[NUM_CHANNELS] = {
  "unused", "unused",
  "pinky_curve2", "pinky_rotate",
  "ring_curve2",  "ring_rotate",
  "middle_curve2","middle_rotate",
  "index_curve2", "index_rotate",
  "thumb_rotate",
  "thumb_curve1_mon",
  "index_curve1_mon",
  "middle_curve1_mon",
  "ring_curve1_mon",
  "pinky_curve1_mon",
};

const float INIT_ANGLE[NUM_CHANNELS] = {
  90.0f,   //  0  unused   — locked
  90.0f,   //  1  unused   — locked
 150.0f,   //  2  pinky    2nd curvature  — open position
  60.0f,   //  3  pinky    rotation
 150.0f,   //  4  ring     2nd curvature  — open position
  30.0f,   //  5  ring     rotation
 150.0f,   //  6  middle   2nd curvature  — open position
  90.0f,   //  7  middle   rotation
 150.0f,   //  8  index    2nd curvature  — open position
  90.0f,   //  9  index    rotation
  45.0f,   // 10  thumb    rotation
 150.0f,   // 11  thumb    1st curvature  — open position
 150.0f,   // 12  index    1st curvature  — open position
 150.0f,   // 13  middle   1st curvature  — open position
 150.0f,   // 14  ring     1st curvature  — open position
 150.0f,   // 15  pinky    1st curvature  — open position
};

const float JOINT_MIN[NUM_CHANNELS] = {
  0.0f,   //  0  unused
  0.0f,   //  1  unused
  0.0f,   //  2  pinky    2nd curvature
  0.0f,   //  3  pinky    rotation
  0.0f,   //  4  ring     2nd curvature
  0.0f,   //  5  ring     rotation
  0.0f,   //  6  middle   2nd curvature
  60.0f,  //  7  middle   rotation
  0.0f,   //  8  index    2nd curvature
  30.0f,  //  9  index    rotation
  0.0f,   // 10  thumb    rotation
  0.0f,   // 11  thumb    1st curvature
  0.0f,   // 12  index    1st curvature
  0.0f,   // 13  middle   1st curvature
  0.0f,   // 14  ring     1st curvature
  0.0f,   // 15  pinky    1st curvature
};

const float JOINT_MAX[NUM_CHANNELS] = {
   90.0f,  //  0  unused
   90.0f,  //  1  unused
  180.0f,  //  2  pinky    2nd curvature
  180.0f,  //  3  pinky    rotation
  180.0f,  //  4  ring     2nd curvature
  105.0f,  //  5  ring     rotation
  180.0f,  //  6  middle   2nd curvature
  120.0f,  //  7  middle   rotation
  180.0f,  //  8  index    2nd curvature
  180.0f,  //  9  index    rotation
  135.0f,  // 10  thumb    rotation
  180.0f,  // 11  thumb    1st curvature
  180.0f,  // 12  index    1st curvature
  180.0f,  // 13  middle   1st curvature
  180.0f,  // 14  ring     1st curvature
  180.0f,  // 15  pinky    1st curvature
};


#define STEP_DEG      0.5f
#define DEFAULT_DPS   60.0f
#define MIN_DPS       5.0f
#define MAX_DPS       720.0f
#define ARRIVE_THRESH 0.6f

struct ServoState {
  float         currentAngle;
  float         targetAngle;
  float         commandedAngle;
  bool          enabled;
  bool          stopped;
  bool          moveDoneSent;
  unsigned long lastStepMs;
};

ServoState sv[NUM_CHANNELS];
float globalDPS   = DEFAULT_DPS;
bool  globalEstop = false;
bool  homingAll   = false;
int   homeRemaining = 0;

// ── Sequential move queue (used by CURVATURE and ROTATE) ──────────────────
#define SEQ_QUEUE_SIZE 16

struct SeqMove {
  int   ch;
  float angle;
};

SeqMove  seqQueue[SEQ_QUEUE_SIZE];
int      seqHead    = 0;
int      seqTail    = 0;
int      seqActive  = -1;
bool     seqRunning = false;
float    seqAngle   = 0.0f;
char     seqCmd[16] = "";

// =============================================================================
// ── INA219 CONFIGURATION ─────────────────────────────────────────────────────
// =============================================================================

#define NUM_SENSORS 5

const uint8_t INA_ADDR[NUM_SENSORS]     = { 0x40,   0x41,     0x44,    0x45,    0x41   };
const char*   INA_LABEL[NUM_SENSORS]    = { "ring", "middle", "index", "pinky", "thumb" };
const char*   INA_BUS_NAME[NUM_SENSORS] = { "Wire1","Wire1",  "Wire1", "Wire1", "Wire"  };
const uint8_t INA_SERVO_CH[NUM_SENSORS] = { 14, 13, 12, 15, 11 };

Adafruit_INA219 ina[NUM_SENSORS] = {
  Adafruit_INA219(0x40),  // ring   → CH14
  Adafruit_INA219(0x41),  // middle → CH13
  Adafruit_INA219(0x44),  // index  → CH12
  Adafruit_INA219(0x45),  // pinky  → CH15
  Adafruit_INA219(0x41),  // thumb  → CH11
};

TwoWire* INA_BUS[NUM_SENSORS] = { &Wire1, &Wire1, &Wire1, &Wire1, &Wire };

bool     sensorFound[NUM_SENSORS];
bool     inaStreaming = false;
uint32_t inaStreamHz = 10;
uint32_t lastInaMs   = 0;

// =============================================================================
// ── FSR CONFIGURATION ────────────────────────────────────────────────────────
// =============================================================================
//
// HOW TO CALIBRATE:
//   1. Upload, open Serial Monitor
//   2. Send FSRRAW with no finger touching → note raw values → set as FSRx_MIN
//   3. Send FSRRAW while pressing firmly   → note raw values → set as FSRx_MAX
//   4. Set FSRx_THRESHOLD to scaled 0–1000 value at which press should trigger
//   5. Re-upload
//
// FSR 4 is PENDING — pin not yet assigned. Set FSR4_PIN to the actual Teensy
// analog pin once wired and change FSR4_PENDING to 0 to enable it.
// ─────────────────────────────────────────────────────────────────────────────

#define FSR4_PENDING  1      // set to 0 once FSR4 is wired and calibrated

// FSR 0 — pin 15 (A1) — thumb
#define FSR0_PIN        15
#define FSR0_MIN        2
#define FSR0_MAX        20
#define FSR0_THRESHOLD  1

// FSR 1 — pin 14 (A0) — index
#define FSR1_PIN        14
#define FSR1_MIN        5
#define FSR1_MAX        600
#define FSR1_THRESHOLD  50

// FSR 2 — pin 41 (A17) — middle
#define FSR2_PIN        41
#define FSR2_MIN        5
#define FSR2_MAX        100
#define FSR2_THRESHOLD  1

// FSR 3 — pin 40 (A16) — ring
#define FSR3_PIN        40
#define FSR3_MIN        5
#define FSR3_MAX        3800
#define FSR3_THRESHOLD  200

// FSR 4 — pin TBD — pinky  ← PENDING: assign pin and calibrate before enabling
#define FSR4_PIN        0      // TODO: replace 0 with actual Teensy analog pin
#define FSR4_MIN        20
#define FSR4_MAX        3800
#define FSR4_THRESHOLD  100

// ─────────────────────────────────────────────────────────────────────────────

#define NUM_FSR   5
#define ADC_BITS  12
#define ADC_MAX   4095

const uint8_t FSR_PIN[NUM_FSR]       = { FSR0_PIN, FSR1_PIN, FSR2_PIN, FSR3_PIN, FSR4_PIN };
const char*   FSR_LABEL[NUM_FSR]     = { "F1_A1",  "F2_A0",  "F3_A17", "F4_A16", "F5_PEND" };
const int     FSR_MIN_VAL[NUM_FSR]   = { FSR0_MIN, FSR1_MIN, FSR2_MIN, FSR3_MIN, FSR4_MIN };
const int     FSR_MAX_VAL[NUM_FSR]   = { FSR0_MAX, FSR1_MAX, FSR2_MAX, FSR3_MAX, FSR4_MAX };
const int     FSR_THRESH[NUM_FSR]    = { FSR0_THRESHOLD, FSR1_THRESHOLD, FSR2_THRESHOLD,
                                         FSR3_THRESHOLD, FSR4_THRESHOLD };

const char* FSR_FINGER[NUM_FSR] = { "thumb", "index", "middle", "ring", "pinky" };

enum FsrFormat { FSR_SCALE, FSR_PCT };
FsrFormat    fsrFormat    = FSR_SCALE;
bool         fsrStreaming  = false;
uint32_t     fsrStreamHz  = 10;
uint32_t     lastFsrMs    = 0;

// =============================================================================
// ── SETUP ────────────────────────────────────────────────────────────────────
// =============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  analogReadResolution(ADC_BITS);

  for (int i = 0; i < NUM_FSR; i++) {
    if (i == 4 && FSR4_PENDING) continue;
    pinMode(FSR_PIN[i], INPUT);
  }
  Serial.println("INFO FSR pins configured");
  if (FSR4_PENDING) Serial.println("INFO FSR4 PENDING — not yet wired");

  Wire1.begin();
  Wire.begin();

  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERR msg=PCA9685_not_found_on_Wire_at_0x40_check_wiring");
    while (1);
  }
  pca.begin();
  pca.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (INIT_ANGLE[ch] < JOINT_MIN[ch] || INIT_ANGLE[ch] > JOINT_MAX[ch]) {
      Serial.print("ERR msg=INIT_ANGLE_ch"); Serial.print(ch);
      Serial.print("_="); Serial.print(INIT_ANGLE[ch], 1);
      Serial.print("_outside_["); Serial.print(JOINT_MIN[ch], 1);
      Serial.print(","); Serial.print(JOINT_MAX[ch], 1);
      Serial.println("]_halting");
      while (1);
    }
  }

  Serial.println("INFO homing servos...");
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float ang = INIT_ANGLE[ch];
    sv[ch].currentAngle   = ang;
    sv[ch].targetAngle    = ang;
    sv[ch].commandedAngle = ang;
    sv[ch].moveDoneSent   = true;
    sv[ch].lastStepMs     = 0;
    if (ch == 0 || ch == 1) {
      sv[ch].enabled = false;
      sv[ch].stopped = true;
      continue;
    }
    sv[ch].enabled = true;
    sv[ch].stopped = false;
    writePulse(ch, ang);
    Serial.print("INFO homed ch="); Serial.print(ch);
    Serial.print(" label="); Serial.print(CH_LABEL[ch]);
    Serial.print(" angle="); Serial.println(ang, 1);
    delay(300);
  }

  Serial.println("INFO initialising INA219...");
  int found = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (ina[i].begin(INA_BUS[i])) {
      ina[i].setCalibration_32V_2A();
      sensorFound[i] = true;
      found++;
      Serial.print("INFO INA ch=");   Serial.print(i);
      Serial.print(" label=");        Serial.print(INA_LABEL[i]);
      Serial.print(" addr=0x");       Serial.print(INA_ADDR[i], HEX);
      Serial.print(" servo_ch=");     Serial.print(INA_SERVO_CH[i]);
      Serial.println(" OK");
    } else {
      sensorFound[i] = false;
      Serial.print("INFO INA ch=");   Serial.print(i);
      Serial.print(" label=");        Serial.print(INA_LABEL[i]);
      Serial.println(" NOT FOUND");
    }
  }

  int activeFsr = NUM_FSR - (FSR4_PENDING ? 1 : 0);
  Serial.print("READY ina="); Serial.print(found); Serial.print("/"); Serial.print(NUM_SENSORS);
  Serial.print(" servos="); Serial.print(NUM_CHANNELS - 2);
  Serial.print(" fsr="); Serial.print(activeFsr); Serial.print("/"); Serial.println(NUM_FSR);

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    Serial.print("INFO init ch="); Serial.print(ch);
    Serial.print(" label=");       Serial.print(CH_LABEL[ch]);
    Serial.print(" angle=");       Serial.print(INIT_ANGLE[ch], 1);
    Serial.print(" min=");         Serial.print(JOINT_MIN[ch], 1);
    Serial.print(" max=");         Serial.println(JOINT_MAX[ch], 1);
  }

  printFsrConfig();
  printHelp();
}

// =============================================================================
// ── MAIN LOOP ────────────────────────────────────────────────────────────────
// =============================================================================

void loop() {
  handleSerial();
  if (!globalEstop) stepServos();

  if (inaStreaming) {
    uint32_t iv = 1000UL / inaStreamHz;
    if ((millis() - lastInaMs) >= iv) { lastInaMs = millis(); readAllSensors(); }
  }

  stepSeqQueue();

  if (fsrStreaming) {
    uint32_t iv = 1000UL / fsrStreamHz;
    if ((millis() - lastFsrMs) >= iv) { lastFsrMs = millis(); readAllFSR(false); }
  }
}

// =============================================================================
// ── SERIAL COMMAND PARSER ────────────────────────────────────────────────────
// =============================================================================

void handleSerial() {
  if (!Serial.available()) return;
  String raw = Serial.readStringUntil('\n');
  raw.trim();
  if (raw.length() == 0) return;

  String cmd = "";
  String args[14];
  int    argc = 0;
  bool   gotCmd = false;
  int    start  = 0;

  for (int i = 0; i <= (int)raw.length(); i++) {
    if (i == (int)raw.length() || raw[i] == ' ') {
      String tok = raw.substring(start, i);
      if (tok.length() == 0) { start = i + 1; continue; }
      if (!gotCmd) { cmd = tok; gotCmd = true; }
      else if (argc < 14) { args[argc++] = tok; }
      start = i + 1;
    }
  }
  cmd.toUpperCase();

  // ── MOVE ───────────────────────────────────────────────────────────────────
  if (cmd == "MOVE") {
    if (argc < 2) { Serial.println("ERR msg=MOVE_requires_channel_and_angle"); return; }
    int   ch  = args[0].toInt();
    float ang = args[1].toFloat();
    if (ch < 0 || ch >= NUM_CHANNELS) { Serial.print("ERR msg=invalid_channel_"); Serial.println(ch); return; }
    if (ch == 0 || ch == 1)           { Serial.println("ERR msg=CH0_CH1_unused_use_2_to_15"); return; }
    if (ang < 0 || ang > 180)         { Serial.print("ERR msg=invalid_angle_"); Serial.println(ang, 1); return; }
    if (globalEstop)                  { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float clamped = constrain(ang, JOINT_MIN[ch], JOINT_MAX[ch]);
    if (clamped != ang) {
      Serial.print("INFO ch="); Serial.print(ch);
      Serial.print(" clamped "); Serial.print(ang, 1); Serial.print(" to "); Serial.println(clamped, 1);
      ang = clamped;
    }
    float fromAngle = sv[ch].currentAngle;
    sv[ch].commandedAngle = ang;
    sv[ch].targetAngle    = ang;
    sv[ch].stopped        = false;
    sv[ch].enabled        = true;
    sv[ch].moveDoneSent   = (fabsf(ang - sv[ch].currentAngle) < ARRIVE_THRESH);
    Serial.print("MOVE_START ch="); Serial.print(ch);
    Serial.print(" label=");        Serial.print(CH_LABEL[ch]);
    Serial.print(" from=");         Serial.print(fromAngle, 1);
    Serial.print(" to=");           Serial.println(ang, 1);
  }

  // ── STOP ───────────────────────────────────────────────────────────────────
  else if (cmd == "STOP") {
    if (argc < 1) { Serial.println("ERR msg=STOP_needs_channel_use_INOSTOP_FSRSTOP_for_streams"); return; }
    int ch = args[0].toInt();
    if (ch < 0 || ch >= NUM_CHANNELS) { Serial.print("ERR msg=invalid_channel_"); Serial.println(ch); return; }
    sv[ch].stopped     = true;
    sv[ch].targetAngle = sv[ch].currentAngle;
    disableChannel(ch);
    Serial.print("STOP ch=");  Serial.print(ch);
    Serial.print(" label=");   Serial.print(CH_LABEL[ch]);
    Serial.print(" angle=");   Serial.println(sv[ch].currentAngle, 1);
  }

  // ── ESTOP ──────────────────────────────────────────────────────────────────
  else if (cmd == "ESTOP") {
    globalEstop = true;
    int count = 0;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      sv[ch].stopped = true; sv[ch].targetAngle = sv[ch].currentAngle; disableChannel(ch);
      if (ch >= 2) count++;
    }
    Serial.print("ESTOP channels="); Serial.println(count);
  }

  // ── RESUME ─────────────────────────────────────────────────────────────────
  else if (cmd == "RESUME") {
    if (argc < 1) { Serial.println("ERR msg=RESUME_needs_channel"); return; }
    int ch = args[0].toInt();
    if (ch < 0 || ch >= NUM_CHANNELS) { Serial.print("ERR msg=invalid_channel_"); Serial.println(ch); return; }
    if (ch == 0 || ch == 1) { Serial.println("ERR msg=CH0_CH1_unused"); return; }
    sv[ch].stopped = false; sv[ch].enabled = true;
    sv[ch].targetAngle  = sv[ch].commandedAngle;
    sv[ch].moveDoneSent = (fabsf(sv[ch].commandedAngle - sv[ch].currentAngle) < ARRIVE_THRESH);
    writePulse(ch, sv[ch].currentAngle);
    Serial.print("RESUME ch="); Serial.print(ch);
    Serial.print(" label=");    Serial.print(CH_LABEL[ch]);
    Serial.print(" target=");   Serial.println(sv[ch].commandedAngle, 1);
  }

  // ── RESUME_ALL ─────────────────────────────────────────────────────────────
  else if (cmd == "RESUME_ALL") {
    globalEstop = false;
    for (int ch = 2; ch < NUM_CHANNELS; ch++) {
      sv[ch].stopped = false; sv[ch].enabled = true;
      sv[ch].targetAngle  = sv[ch].commandedAngle;
      sv[ch].moveDoneSent = (fabsf(sv[ch].commandedAngle - sv[ch].currentAngle) < ARRIVE_THRESH);
      writePulse(ch, sv[ch].currentAngle);
    }
    Serial.println("RESUME_ALL");
  }

  // ── HOME / HOME <ch> ───────────────────────────────────────────────────────
  else if (cmd == "HOME") {
    if (globalEstop) { Serial.println("ERR msg=cannot_HOME_during_ESTOP_send_RESUME_ALL_first"); return; }
    if (argc >= 1) {
      int ch = args[0].toInt();
      if (ch < 0 || ch >= NUM_CHANNELS) { Serial.print("ERR msg=invalid_channel_"); Serial.println(ch); return; }
      if (ch == 0 || ch == 1) { Serial.println("ERR msg=CH0_CH1_unused"); return; }
      float target = INIT_ANGLE[ch];
      sv[ch].commandedAngle = target;
      sv[ch].targetAngle    = target;
      sv[ch].stopped        = false;
      sv[ch].enabled        = true;
      sv[ch].moveDoneSent   = (fabsf(target - sv[ch].currentAngle) < ARRIVE_THRESH);
      Serial.print("MOVE_START ch="); Serial.print(ch);
      Serial.print(" label=");        Serial.print(CH_LABEL[ch]);
      Serial.print(" from=");         Serial.print(sv[ch].currentAngle, 1);
      Serial.print(" to=");           Serial.println(target, 1);
    } else {
      int count = 0;
      for (int ch = 2; ch < NUM_CHANNELS; ch++) {
        float target = INIT_ANGLE[ch];
        sv[ch].commandedAngle = target;
        sv[ch].targetAngle    = target;
        sv[ch].stopped        = false;
        sv[ch].enabled        = true;
        sv[ch].moveDoneSent   = (fabsf(target - sv[ch].currentAngle) < ARRIVE_THRESH);
        count++;
      }
      homingAll = true;
      homeRemaining = count;
      Serial.print("HOME_START channels="); Serial.println(count);
    }
  }

  // ── CURVATURE <angle> ─────────────────────────────────────────────────────
  else if (cmd == "CURVATURE") {
    if (argc < 1) { Serial.println("ERR msg=CURVATURE_requires_angle"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float ang = args[0].toFloat();
    if (ang < 0 || ang > 180) { Serial.println("ERR msg=angle_must_be_0_to_180"); return; }
    // thumb curve1, index curve1+curve2, middle curve1+curve2, ring curve1+curve2, pinky curve1+curve2
    const int curveChs[] = { 11, 12, 8, 13, 6, 14, 4, 15, 2 };
    const int nCurve = 9;
    seqHead = 0; seqTail = 0; seqActive = -1;
    for (int i = 0; i < nCurve; i++) {
      seqQueue[seqTail] = { curveChs[i], ang };
      seqTail = (seqTail + 1) % SEQ_QUEUE_SIZE;
    }
    seqAngle   = ang;
    seqRunning = true;
    strncpy(seqCmd, "CURVATURE", sizeof(seqCmd));
    Serial.print("SEQ_START cmd=CURVATURE angle="); Serial.print(ang, 1);
    Serial.println(" joints=9");
    dispatchNextSeq();
  }

  // ── ROTATE <angle> ─────────────────────────────────────────────────────────
  else if (cmd == "ROTATE") {
    if (argc < 1) { Serial.println("ERR msg=ROTATE_requires_angle"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float ang = args[0].toFloat();
    if (ang < 0 || ang > 180) { Serial.println("ERR msg=angle_must_be_0_to_180"); return; }
    const int rotateChs[] = { 10, 9, 7, 5, 3 };
    const int nRotate = 5;
    seqHead = 0; seqTail = 0; seqActive = -1;
    for (int i = 0; i < nRotate; i++) {
      seqQueue[seqTail] = { rotateChs[i], ang };
      seqTail = (seqTail + 1) % SEQ_QUEUE_SIZE;
    }
    seqAngle   = ang;
    seqRunning = true;
    strncpy(seqCmd, "ROTATE", sizeof(seqCmd));
    Serial.print("SEQ_START cmd=ROTATE angle="); Serial.print(ang, 1);
    Serial.println(" joints=5");
    dispatchNextSeq();
  }

  // ── CURVATURE_SYNC <angle> ────────────────────────────────────────────────
  else if (cmd == "CURVATURE_SYNC") {
    if (argc < 1) { Serial.println("ERR msg=CURVATURE_SYNC_requires_angle"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float ang = args[0].toFloat();
    if (ang < 0 || ang > 180) { Serial.println("ERR msg=angle_must_be_0_to_180"); return; }
    const int chs[] = { 11, 12, 8, 13, 6, 14, 4, 15, 2 };
    const int n = 9;
    Serial.print("SYNC_START cmd=CURVATURE_SYNC angle="); Serial.print(ang, 1);
    Serial.print(" joints="); Serial.println(n);
    for (int i = 0; i < n; i++) startMoveQuiet(chs[i], ang);
  }

  // ── ROTATE_SYNC <angle> ───────────────────────────────────────────────────
  else if (cmd == "ROTATE_SYNC") {
    if (argc < 1) { Serial.println("ERR msg=ROTATE_SYNC_requires_angle"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float ang = args[0].toFloat();
    if (ang < 0 || ang > 180) { Serial.println("ERR msg=angle_must_be_0_to_180"); return; }
    const int chs[] = { 10, 9, 7, 5, 3 };
    const int n = 5;
    Serial.print("SYNC_START cmd=ROTATE_SYNC angle="); Serial.print(ang, 1);
    Serial.print(" joints="); Serial.println(n);
    for (int i = 0; i < n; i++) startMoveQuiet(chs[i], ang);
  }

  // ── MOVEALL <angle> ───────────────────────────────────────────────────────
  else if (cmd == "MOVEALL") {
    if (argc < 1) { Serial.println("ERR msg=MOVEALL_requires_angle"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    float ang = args[0].toFloat();
    if (ang < 0 || ang > 180) { Serial.println("ERR msg=angle_must_be_0_to_180"); return; }
    int n = 0;
    for (int ch = 2; ch < NUM_CHANNELS; ch++) { startMoveQuiet(ch, ang); n++; }
    Serial.print("SYNC_START cmd=MOVEALL angle="); Serial.print(ang, 1);
    Serial.print(" joints="); Serial.println(n);
  }

  // ── MOVESET ch:angle [ch:angle ...] ───────────────────────────────────────
  // Up to 14 channel:angle pairs per command.
  else if (cmd == "MOVESET") {
    if (argc < 1) { Serial.println("ERR msg=MOVESET_requires_at_least_one_ch:angle_pair"); return; }
    if (globalEstop) { Serial.println("ERR msg=in_ESTOP_send_RESUME_ALL_first"); return; }
    int applied = 0;
    for (int i = 0; i < argc; i++) {
      int colonIdx = args[i].indexOf(':');
      if (colonIdx < 0) {
        Serial.print("ERR msg=bad_token_"); Serial.print(args[i]); Serial.println("_expected_ch:angle");
        continue;
      }
      int   ch  = args[i].substring(0, colonIdx).toInt();
      float ang = args[i].substring(colonIdx + 1).toFloat();
      if (ch < 2 || ch >= NUM_CHANNELS) {
        Serial.print("ERR msg=invalid_channel_"); Serial.println(ch); continue;
      }
      startMoveQuiet(ch, ang);
      applied++;
    }
    Serial.print("MOVESET_START joints="); Serial.println(applied);
  }

  // ── SPEED ──────────────────────────────────────────────────────────────────
  else if (cmd == "SPEED") {
    if (argc < 1) { Serial.println("ERR msg=SPEED_needs_value"); return; }
    float dps = args[0].toFloat();
    if (dps < MIN_DPS || dps > MAX_DPS) {
      Serial.print("ERR msg=SPEED_out_of_range_"); Serial.print((int)MIN_DPS);
      Serial.print("_to_"); Serial.println((int)MAX_DPS); return;
    }
    globalDPS = dps;
    Serial.print("SPEED dps="); Serial.println(globalDPS, 1);
  }

  // ── READ (INA) ─────────────────────────────────────────────────────────────
  else if (cmd == "READ") {
    readAllSensors();
  }

  // ── STREAM <hz> (INA) ──────────────────────────────────────────────────────
  else if (cmd == "STREAM") {
    uint32_t hz = (argc > 0) ? (uint32_t)args[0].toInt() : 10;
    if (hz < 1 || hz > 100) { Serial.println("ERR msg=STREAM_rate_must_be_1_to_100_Hz"); return; }
    inaStreamHz = hz; inaStreaming = true; lastInaMs = millis();
    Serial.print("INA_STREAM hz="); Serial.println(inaStreamHz);
  }

  // ── INOSTOP ────────────────────────────────────────────────────────────────
  else if (cmd == "INOSTOP") {
    inaStreaming = false;
    Serial.println("INA_STREAM_STOP");
  }

  // ── FSRREAD ────────────────────────────────────────────────────────────────
  else if (cmd == "FSRREAD") {
    readAllFSR(false);
  }

  // ── FSRRAW ─────────────────────────────────────────────────────────────────
  else if (cmd == "FSRRAW") {
    readAllFSR(true);
  }

  // ── FSRSTREAM <hz> ─────────────────────────────────────────────────────────
  else if (cmd == "FSRSTREAM") {
    uint32_t hz = (argc > 0) ? (uint32_t)args[0].toInt() : 10;
    if (hz < 1 || hz > 50) { Serial.println("ERR msg=FSRSTREAM_rate_must_be_1_to_50_Hz"); return; }
    fsrStreamHz = hz; fsrStreaming = true; lastFsrMs = millis();
    Serial.print("FSR_STREAM hz="); Serial.println(fsrStreamHz);
  }

  // ── FSRSTOP ────────────────────────────────────────────────────────────────
  else if (cmd == "FSRSTOP") {
    fsrStreaming = false;
    Serial.println("FSR_STREAM_STOP");
  }

  // ── FSRFORMAT ──────────────────────────────────────────────────────────────
  else if (cmd == "FSRFORMAT") {
    if (argc < 1) { Serial.println("ERR msg=FSRFORMAT_needs_PCT_or_SCALE"); return; }
    args[0].toUpperCase();
    if      (args[0] == "PCT")   { fsrFormat = FSR_PCT;   Serial.println("INFO FSR format: 0.0-100.0%"); }
    else if (args[0] == "SCALE") { fsrFormat = FSR_SCALE; Serial.println("INFO FSR format: 0-1000"); }
    else { Serial.println("ERR msg=FSRFORMAT_use_PCT_or_SCALE"); }
  }

  // ── SCAN ───────────────────────────────────────────────────────────────────
  else if (cmd == "SCAN") {
    doI2CScan();
  }

  // ── STATUS ─────────────────────────────────────────────────────────────────
  else if (cmd == "STATUS") {
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
      int8_t ic = SERVO_INA_CH[ch];
      Serial.print("STATUS_SERVO");
      Serial.print(" ch=");      Serial.print(ch);
      Serial.print(" label=");   Serial.print(CH_LABEL[ch]);
      Serial.print(" angle=");   Serial.print(sv[ch].currentAngle, 1);
      Serial.print(" target=");  Serial.print(sv[ch].targetAngle, 1);
      Serial.print(" init=");    Serial.print(INIT_ANGLE[ch], 1);
      Serial.print(" min=");     Serial.print(JOINT_MIN[ch], 1);
      Serial.print(" max=");     Serial.print(JOINT_MAX[ch], 1);
      Serial.print(" enabled="); Serial.print(sv[ch].enabled ? 1 : 0);
      Serial.print(" stopped="); Serial.print(sv[ch].stopped ? 1 : 0);
      Serial.print(" ina_ch=");  Serial.println(ic);
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print("STATUS_INA");
      Serial.print(" ch=");       Serial.print(i);
      Serial.print(" label=");    Serial.print(INA_LABEL[i]);
      Serial.print(" addr=0x");   Serial.print(INA_ADDR[i], HEX);
      Serial.print(" bus=");      Serial.print(INA_BUS_NAME[i]);
      Serial.print(" servo_ch="); Serial.print(INA_SERVO_CH[i]);
      Serial.print(" found=");    Serial.println(sensorFound[i] ? 1 : 0);
    }
    for (int i = 0; i < NUM_FSR; i++) {
      bool pending = (i == 4 && FSR4_PENDING);
      int  rawVal  = pending ? 0 : analogRead(FSR_PIN[i]);
      int  scaled  = pending ? 0 : scaleFSR(rawVal, i);
      bool pressed = (!pending) && (scaled >= FSR_THRESH[i]);
      Serial.print("STATUS_FSR");
      Serial.print(" ch=");      Serial.print(i);
      Serial.print(" label=");   Serial.print(FSR_LABEL[i]);
      Serial.print(" finger=");  Serial.print(FSR_FINGER[i]);
      Serial.print(" pin=");     Serial.print(FSR_PIN[i]);
      Serial.print(" raw=");     Serial.print(rawVal);
      Serial.print(" scaled=");  Serial.print(scaled);
      Serial.print(" pct=");     Serial.print(scaled / 10.0f, 1);
      Serial.print(" pressed="); Serial.print(pressed ? 1 : 0);
      Serial.print(" pending="); Serial.println(pending ? 1 : 0);
    }
    Serial.print("STATUS_SYSTEM");
    Serial.print(" estop=");       Serial.print(globalEstop ? 1 : 0);
    Serial.print(" speed_dps=");   Serial.print(globalDPS, 1);
    Serial.print(" ina_stream=");  Serial.print(inaStreaming ? 1 : 0);
    Serial.print(" ina_hz=");      Serial.print(inaStreamHz);
    Serial.print(" fsr_stream=");  Serial.print(fsrStreaming ? 1 : 0);
    Serial.print(" fsr_hz=");      Serial.println(fsrStreamHz);
  }

  // ── HELP ───────────────────────────────────────────────────────────────────
  else if (cmd == "HELP") {
    printHelp();
  }

  else {
    Serial.print("ERR msg=unknown_command_"); Serial.println(cmd);
    Serial.println("INFO type HELP for commands");
  }
}

// =============================================================================
// ── SYNC MOVE HELPER ─────────────────────────────────────────────────────────
// =============================================================================

void startMoveQuiet(int ch, float ang) {
  ang = constrain(ang, JOINT_MIN[ch], JOINT_MAX[ch]);
  sv[ch].commandedAngle = ang;
  sv[ch].targetAngle    = ang;
  sv[ch].stopped        = false;
  sv[ch].enabled        = true;
  sv[ch].moveDoneSent   = (fabsf(ang - sv[ch].currentAngle) < ARRIVE_THRESH);
}

// =============================================================================
// ── SEQUENTIAL QUEUE HELPER ──────────────────────────────────────────────────
// =============================================================================

void dispatchNextSeq() {
  while (seqHead != seqTail) {
    int   ch  = seqQueue[seqHead].ch;
    float ang = seqQueue[seqHead].angle;
    seqHead = (seqHead + 1) % SEQ_QUEUE_SIZE;
    ang = constrain(ang, JOINT_MIN[ch], JOINT_MAX[ch]);
    if (fabsf(ang - sv[ch].currentAngle) < ARRIVE_THRESH) {
      Serial.print("MOVE_DONE ch="); Serial.print(ch);
      Serial.print(" label=");       Serial.print(CH_LABEL[ch]);
      Serial.print(" angle=");       Serial.println(sv[ch].currentAngle, 1);
      if (seqHead == seqTail) {
        seqActive  = -1;
        seqRunning = false;
        Serial.print("SEQ_DONE cmd="); Serial.print(seqCmd);
        Serial.print(" angle=");       Serial.println(seqAngle, 1);
      }
      continue;
    }
    seqActive             = ch;
    sv[ch].commandedAngle = ang;
    sv[ch].targetAngle    = ang;
    sv[ch].stopped        = false;
    sv[ch].enabled        = true;
    sv[ch].moveDoneSent   = false;
    Serial.print("MOVE_START ch="); Serial.print(ch);
    Serial.print(" label=");        Serial.print(CH_LABEL[ch]);
    Serial.print(" from=");         Serial.print(sv[ch].currentAngle, 1);
    Serial.print(" to=");           Serial.println(ang, 1);
    return;
  }
  seqActive  = -1;
  seqRunning = false;
  Serial.print("SEQ_DONE cmd="); Serial.print(seqCmd);
  Serial.print(" angle=");       Serial.println(seqAngle, 1);
}

void stepSeqQueue() {
  if (!seqRunning || seqActive < 0) return;
  if (sv[seqActive].moveDoneSent) {
    seqActive = -1;
    dispatchNextSeq();
  }
}

// =============================================================================
// ── SERVO HELPERS ────────────────────────────────────────────────────────────
// =============================================================================

void stepServos() {
  unsigned long stepMs = (unsigned long)((STEP_DEG / globalDPS) * 1000.0f);
  if (stepMs < 1) stepMs = 1;
  unsigned long now = millis();
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    if (!sv[ch].enabled || sv[ch].stopped) continue;
    float diff = sv[ch].targetAngle - sv[ch].currentAngle;
    if (fabsf(diff) < ARRIVE_THRESH) {
      if (!sv[ch].moveDoneSent) {
        sv[ch].currentAngle = sv[ch].targetAngle;
        writePulse(ch, sv[ch].currentAngle);
        Serial.print("MOVE_DONE ch="); Serial.print(ch);
        Serial.print(" label=");       Serial.print(CH_LABEL[ch]);
        Serial.print(" angle=");       Serial.println(sv[ch].currentAngle, 1);
        sv[ch].moveDoneSent = true;
        if (homingAll) {
          homeRemaining--;
          if (homeRemaining <= 0) {
            homingAll = false;
            Serial.println("HOME_DONE");
          }
        }
      }
      continue;
    }
    if ((now - sv[ch].lastStepMs) < stepMs) continue;
    sv[ch].lastStepMs = now;
    float step = (diff > 0) ? STEP_DEG : -STEP_DEG;
    sv[ch].currentAngle += step;
    sv[ch].currentAngle  = constrain(sv[ch].currentAngle, 0.0f, 180.0f);
    writePulse(ch, sv[ch].currentAngle);
  }
}

void writePulse(int ch, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  uint16_t us = (uint16_t)mapFloat(angle, 0.0f, 180.0f, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  pca.writeMicroseconds(ch, us);
}

void disableChannel(int ch) {
  pca.setPWM(ch, 0, 0);
  sv[ch].enabled = false;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =============================================================================
// ── INA219 HELPERS ───────────────────────────────────────────────────────────
// =============================================================================

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorFound[i]) {
      Serial.print("INA_MISSING");
      Serial.print(" ch=");     Serial.print(i);
      Serial.print(" label=");  Serial.print(INA_LABEL[i]);
      Serial.print(" addr=0x"); Serial.print(INA_ADDR[i], HEX);
      Serial.print(" bus=");    Serial.println(INA_BUS_NAME[i]);
      continue;
    }
    Serial.print("INA");
    Serial.print(" ch=");         Serial.print(i);
    Serial.print(" label=");      Serial.print(INA_LABEL[i]);
    Serial.print(" addr=0x");     Serial.print(INA_ADDR[i], HEX);
    Serial.print(" bus=");        Serial.print(INA_BUS_NAME[i]);
    Serial.print(" servo_ch=");   Serial.print(INA_SERVO_CH[i]);
    Serial.print(" bus_v=");      Serial.print(ina[i].getBusVoltage_V(),    2);
    Serial.print(" shunt_mv=");   Serial.print(ina[i].getShuntVoltage_mV(), 3);
    Serial.print(" current_ma="); Serial.print(ina[i].getCurrent_mA(),      1);
    Serial.print(" power_mw=");   Serial.println(ina[i].getPower_mW(),      1);
  }
}

void doI2CScan() {
  TwoWire*    buses[2] = { &Wire1, &Wire };
  const char* names[2] = { "Wire1","Wire" };
  for (int b = 0; b < 2; b++) {
    int found = 0; String lst = "";
    for (uint8_t a = 1; a < 127; a++) {
      buses[b]->beginTransmission(a);
      if (buses[b]->endTransmission() == 0) {
        if (found > 0) lst += ",";
        lst += "0x"; if (a < 16) lst += "0"; lst += String(a, HEX); found++;
      }
    }
    Serial.print("SCAN");
    Serial.print(" bus=");   Serial.print(names[b]);
    Serial.print(" found="); Serial.print(found);
    if (found > 0) { Serial.print(" addr="); Serial.print(lst); }
    Serial.println();
  }
}

// =============================================================================
// ── FSR HELPERS ──────────────────────────────────────────────────────────────
// =============================================================================

int scaleFSR(int raw, int i) {
  if (FSR_MAX_VAL[i] <= FSR_MIN_VAL[i]) return 0;
  long s = (long)(raw - FSR_MIN_VAL[i]) * 1000L / (FSR_MAX_VAL[i] - FSR_MIN_VAL[i]);
  if (s < 0)    s = 0;
  if (s > 1000) s = 1000;
  return (int)s;
}

void readAllFSR(bool rawOnly) {
  for (int i = 0; i < NUM_FSR; i++) {
    bool pending = (i == 4 && FSR4_PENDING);
    Serial.print("FSR");
    Serial.print(" ch=");      Serial.print(i);
    Serial.print(" label=");   Serial.print(FSR_LABEL[i]);
    Serial.print(" finger=");  Serial.print(FSR_FINGER[i]);
    Serial.print(" pending="); Serial.print(pending ? 1 : 0);

    if (pending) { Serial.println(); continue; }

    int  rawVal = analogRead(FSR_PIN[i]);
    int  scaled = scaleFSR(rawVal, i);
    bool pressed = (scaled >= FSR_THRESH[i]);

    Serial.print(" raw=");     Serial.print(rawVal);
    Serial.print(" scaled=");  Serial.print(scaled);
    Serial.print(" pct=");     Serial.print(scaled / 10.0f, 1);
    Serial.print(" pressed="); Serial.println(pressed ? 1 : 0);
  }
}

void printFsrConfig() {
  Serial.println("INFO ── FSR Config ──────────────────────────────────");
  for (int i = 0; i < NUM_FSR; i++) {
    bool pending = (i == 4 && FSR4_PENDING);
    Serial.print("INFO FSR "); Serial.print(i);
    Serial.print(" ("); Serial.print(FSR_LABEL[i]); Serial.print(")");
    Serial.print(" finger="); Serial.print(FSR_FINGER[i]);
    if (pending) {
      Serial.println(" PENDING (not yet wired)");
    } else {
      Serial.print("  min=");    Serial.print(FSR_MIN_VAL[i]);
      Serial.print("  max=");    Serial.print(FSR_MAX_VAL[i]);
      Serial.print("  thresh="); Serial.print(FSR_THRESH[i]);
      Serial.print("/1000 (");   Serial.print(FSR_THRESH[i] / 10.0f, 1);
      Serial.println("%)");
    }
  }
  Serial.println("INFO ────────────────────────────────────────────────");
}

// =============================================================================
// ── HELP ─────────────────────────────────────────────────────────────────────
// =============================================================================

void printHelp() {
  Serial.println("INFO ============================================================");
  Serial.println("INFO  hand_control_v7");
  Serial.println("INFO  Protocol: TYPE key=val key=val ... (machine-parseable)");
  Serial.println("INFO  INFO lines are human-only — skip in parsers.");
  Serial.println("INFO ============================================================");
  Serial.println("INFO  SERVO");
  Serial.println("INFO    MOVE <ch> <angle>       move ch 2-15 to angle 0-180");
  Serial.println("INFO    CURVATURE <angle>       all curvature joints, one by one");
  Serial.println("INFO    CURVATURE_SYNC <angle>  all curvature joints simultaneously");
  Serial.println("INFO    ROTATE <angle>          all rotation joints, one by one");
  Serial.println("INFO    ROTATE_SYNC <angle>     all rotation joints simultaneously");
  Serial.println("INFO    MOVEALL <angle>         ALL joints simultaneously");
  Serial.println("INFO    MOVESET ch:a [ch:a..]   named channels simultaneously (up to 14)");
  Serial.println("INFO    HOME                    move ALL servos to init angles");
  Serial.println("INFO    HOME <ch>               move one servo to its init angle");
  Serial.println("INFO    STOP <ch>               disable channel (holds position)");
  Serial.println("INFO    ESTOP                   disable ALL channels");
  Serial.println("INFO    RESUME <ch>             re-enable, resume last target");
  Serial.println("INFO    RESUME_ALL              re-enable all, clear estop");
  Serial.println("INFO    SPEED <dps>             slew rate 5-720 deg/sec");
  Serial.println("INFO  INA219 CURRENT SENSORS");
  Serial.println("INFO    READ                    one-shot → INA lines");
  Serial.println("INFO    STREAM <hz>             continuous INA (1-100 Hz)");
  Serial.println("INFO    INOSTOP                 stop → INA_STREAM_STOP");
  Serial.println("INFO    SCAN                    I2C scan → SCAN lines");
  Serial.println("INFO  FSR PRESSURE SENSORS");
  Serial.println("INFO    FSRREAD                 one-shot → FSR lines");
  Serial.println("INFO    FSRRAW                  one-shot raw ADC → FSR lines");
  Serial.println("INFO    FSRSTREAM <hz>          continuous FSR (1-50 Hz)");
  Serial.println("INFO    FSRSTOP                 stop → FSR_STREAM_STOP");
  Serial.println("INFO    FSRFORMAT SCALE|PCT     output 0-1000 or 0.0-100.0%");
  Serial.println("INFO  GENERAL");
  Serial.println("INFO    STATUS  → STATUS_SERVO/STATUS_INA/STATUS_FSR/STATUS_SYSTEM");
  Serial.println("INFO    HELP    this list");
  Serial.println("INFO ============================================================");
}
