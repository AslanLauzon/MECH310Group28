#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Encoder.h>

// -------------------- Pins --------------------
#define STEP_PIN 9
#define DIR_PIN 10
#define ENCODER_PIN_A A2
#define ENCODER_PIN_B A1

// -------------------- Mechanics --------------------
#define STEPS_PER_REV 3200L   // 200 steps * 16 microstepping
#define ENCODER_CPR   2048L   // e.g., 512 PPR * 4 (x4 decoding)

// -------------------- Motion limits (sane caps) --------------------
const float MAX_SPEED_LIMIT_SPS = 10000.0f;     // steps/s
const float MAX_ACCEL_LIMIT_SPS2 = 50000.0f;    // steps/s^2

// Defaults on boot
float cfg_speed_sps = 1500.0f;   // steps/s
float cfg_accel_sps2 = 10000.0f; // steps/s^2

// -------------------- CSV timing --------------------
const uint16_t SAMPLE_PERIOD_MS = 20; // 50 Hz
const bool USE_TIME_MS = true;
const bool USE_TIME_S  = false;

// -------------------- Objects --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);

// Serial command buffer
String inputLine;

// Timekeepers
elapsedMillis csvTimer;
elapsedMillis runTime;

// -------------------- Helpers --------------------
inline float stepsToDeg(long steps) {
  return (steps * 360.0f) / (float)STEPS_PER_REV;
}
inline float encCountsToDeg(long counts) {
  return (counts * 360.0f) / (float)ENCODER_CPR;
}

void printCsvHeader() {
  if (USE_TIME_MS) {
    Serial.println(F("time_ms,position_step,position_deg,target_step"));
  } else if (USE_TIME_S) {
    Serial.println(F("time_s,position_step,position_deg,target_step"));
  } else {
    Serial.println(F("time,position_step,position_deg,target_step"));
  }
}

void printStatus() {
  Serial.print(F("Speed (steps/s): ")); Serial.println(cfg_speed_sps, 2);
  Serial.print(F("Accel (steps/s^2): ")); Serial.println(cfg_accel_sps2, 2);
  Serial.print(F("Current pos (steps): ")); Serial.println(stepper.currentPosition());
  Serial.print(F("Target  pos (steps): ")); Serial.println(stepper.targetPosition());
}

void clampAndApplyMotionConfig() {
  if (cfg_speed_sps < 1) cfg_speed_sps = 1;
  if (cfg_speed_sps > MAX_SPEED_LIMIT_SPS) cfg_speed_sps = MAX_SPEED_LIMIT_SPS;
  if (cfg_accel_sps2 < 1) cfg_accel_sps2 = 1;
  if (cfg_accel_sps2 > MAX_ACCEL_LIMIT_SPS2) cfg_accel_sps2 = MAX_ACCEL_LIMIT_SPS2;

  stepper.setMaxSpeed(cfg_speed_sps);
  stepper.setAcceleration(cfg_accel_sps2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  clampAndApplyMotionConfig();
  stepper.setCurrentPosition(0);

  encoder.write(0);

  Serial.println(F("Stepper Profile Mode (speed/accel/distance)"));
  Serial.println(F("Commands: v <sps>, a <sps2>, m <steps>, g <step>, p v=<sps> a=<sps2> d=<steps>, z, status, help"));
  printCsvHeader();

  csvTimer = 0;
  runTime = 0;
}

void loop() {
  // Keep executing motion profile
  delay(5000)
  stepper.run();

  // Handle serial input
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }

  // CSV streaming
  if (csvTimer >= SAMPLE_PERIOD_MS) {
    csvTimer = 0;

    unsigned long t_ms = runTime;
    long stepPos = stepper.currentPosition();
    long target  = stepper.targetPosition();
    long encCnt  = encoder.read();
    float angle_deg = encCountsToDeg(encCnt);

    if (USE_TIME_MS) {
      Serial.print(t_ms);
    } else if (USE_TIME_S) {
      Serial.print(t_ms / 1000.0f, 3);
    } else {
      Serial.print(t_ms);
    }
    Serial.print(',');
    Serial.print(stepPos);
    Serial.print(',');
    Serial.print(angle_deg, 3);
    Serial.print(',');
    Serial.println(target);
  }
}

// -------------------- Command Handling --------------------
// trim helpers
String ltrim(const String &s) {
  int i = 0; while (i < (int)s.length() && isspace(s[i])) i++;
  return s.substring(i);
}
String rtrim(const String &s) {
  int i = s.length() - 1; while (i >= 0 && isspace(s[i])) i--;
  return s.substring(0, i + 1);
}
String trim(const String &s) { return rtrim(ltrim(s)); }

long parseLongSafe(const String &tok, bool &ok) {
  String t = trim(tok);
  if (t.length() == 0) { ok = false; return 0; }
  bool neg = (t[0] == '-');
  int start = (neg || t[0] == '+') ? 1 : 0;
  for (int i = start; i < (int)t.length(); i++) {
    if (!isDigit(t[i])) { ok = false; return 0; }
  }
  ok = true; return t.toInt();
}

float parseFloatSafe(const String &tok, bool &ok) {
  String t = trim(tok);
  if (t.length() == 0) { ok = false; return 0; }
  bool dotSeen = false;
  int start = (t[0] == '-' || t[0] == '+') ? 1 : 0;
  for (int i = start; i < (int)t.length(); i++) {
    if (t[i] == '.') {
      if (dotSeen) { ok = false; return 0; }
      dotSeen = true;
    } else if (!isDigit(t[i])) { ok = false; return 0; }
  }
  ok = true; return t.toFloat();
}

void handleCommand(String cmd) {
  cmd = trim(cmd);
  if (cmd.length() == 0) return;

  // help
  if (cmd.equalsIgnoreCase("help")) {
    Serial.println(F("v <sps>          : set max speed (steps/s)"));
    Serial.println(F("a <sps2>         : set acceleration (steps/s^2)"));
    Serial.println(F("m <steps>        : move relative (steps)"));
    Serial.println(F("g <step>         : go absolute to step position"));
    Serial.println(F("p v=<sps> a=<sps2> d=<steps> : set parameters and run relative move"));
    Serial.println(F("z                : zero stepper and encoder"));
    Serial.println(F("status           : show settings and positions"));
    return;
  }

  if (cmd.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("z")) {
    stepper.setCurrentPosition(0);
    encoder.write(0);
    Serial.println(F("Zeroed stepper and encoder."));
    return;
  }

  // tokenized form: leading verb + args
  // e.g., "v 1500", "a 8000", "m 2000", "g 0"
  if (cmd.length() >= 2 && (cmd[1] == ' ' || cmd[1] == '\t')) {
    char verb = tolower(cmd[0]);
    String arg = trim(cmd.substring(2));

    if (verb == 'v') {
      bool ok = false; float sps = parseFloatSafe(arg, ok);
      if (!ok || sps <= 0) { Serial.println(F("Bad speed.")); return; }
      cfg_speed_sps = sps;
      clampAndApplyMotionConfig();
      Serial.print(F("Speed set to ")); Serial.print(cfg_speed_sps, 2); Serial.println(F(" steps/s"));
      return;
    }

    if (verb == 'a') {
      bool ok = false; float sps2 = parseFloatSafe(arg, ok);
      if (!ok || sps2 <= 0) { Serial.println(F("Bad accel.")); return; }
      cfg_accel_sps2 = sps2;
      clampAndApplyMotionConfig();
      Serial.print(F("Accel set to ")); Serial.print(cfg_accel_sps2, 2); Serial.println(F(" steps/s^2"));
      return;
    }

    if (verb == 'm') {
      bool ok = false; long d = parseLongSafe(arg, ok);
      if (!ok) { Serial.println(F("Bad distance.")); return; }
      stepper.move(d);
      Serial.print(F("Moving relative ")); Serial.print(d); Serial.println(F(" steps"));
      return;
    }

    if (verb == 'g') {
      bool ok = false; long tgt = parseLongSafe(arg, ok);
      if (!ok) { Serial.println(F("Bad target.")); return; }
      stepper.moveTo(tgt);
      Serial.print(F("Going absolute to ")); Serial.print(tgt); Serial.println(F(" steps"));
      return;
    }
  }

  // profile form: p v=<..> a=<..> d=<..>
  if (cmd.length() >= 1 && (tolower(cmd[0]) == 'p')) {
    float ns = cfg_speed_sps;
    float na = cfg_accel_sps2;
    long  nd = 0;
    bool haveV=false, haveA=false, haveD=false;

    // simple parser for key=value separated by spaces/commas
    int i = 1;
    while (i < (int)cmd.length() && isspace(cmd[i])) i++;
    String rest = cmd.substring(i);

    // split by spaces
    int start = 0;
    while (start < (int)rest.length()) {
      // find token end
      int end = rest.indexOf(' ', start);
      if (end < 0) end = rest.length();
      String tok = rest.substring(start, end);
      tok = trim(tok);
      if (tok.length() > 0) {
        tok.replace(",", ""); // allow commas
        int eq = tok.indexOf('=');
        if (eq > 0) {
          String key = tok.substring(0, eq); key.toLowerCase(); key = trim(key);
          String val = tok.substring(eq + 1); val = trim(val);
          if (key == "v") {
            bool ok=false; float s = parseFloatSafe(val, ok);
            if (ok && s > 0) { ns = s; haveV = true; }
          } else if (key == "a") {
            bool ok=false; float a = parseFloatSafe(val, ok);
            if (ok && a > 0) { na = a; haveA = true; }
          } else if (key == "d") {
            bool ok=false; long d = parseLongSafe(val, ok);
            if (ok) { nd = d; haveD = true; }
          }
        }
      }
      start = end + 1;
    }

    if (!haveD) { Serial.println(F("Profile missing d=<steps>")); return; }

    cfg_speed_sps = ns;
    cfg_accel_sps2 = na;
    clampAndApplyMotionConfig();

    stepper.move(nd);

    Serial.print(F("Profile: v=")); Serial.print(cfg_speed_sps, 2);
    Serial.print(F(" steps/s, a=")); Serial.print(cfg_accel_sps2, 2);
    Serial.print(F(" steps/s^2, d=")); Serial.print(nd);
    Serial.println(F(" steps (relative)"));
    return;
  }

  Serial.println(F("Unknown command. Type 'help' for options."));
}
