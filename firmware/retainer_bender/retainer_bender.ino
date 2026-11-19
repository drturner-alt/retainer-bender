// Retainer bender firmware: two steppers, line commands over USB serial.
//
// Board: Teensy 4.1 or Arduino Mega 2560. Library: AccelStepper (Library
// Manager). Drivers: anything with STEP/DIR/ENABLE inputs.
//
//   HOME          home feed stage (to its rear switch) and arm (to its switch)
//   FEED <mm>     move the feed stage forward by mm (relative)
//   ARM <deg>     move the arm to an absolute angle; 0 = neutral
//   JOG <mm>      like FEED but allowed backwards, for loading wire
//   STATUS        print positions
//   OFF           disable the motors (so the arm can be turned by hand)
// Every command answers "ok" or "error: ...". The host waits for each reply.
//
// The E-stop must also cut motor power in hardware. The input here only
// stops the program and makes the firmware refuse moves until HOME.

#include <AccelStepper.h>

// ---- pins (change to match your wiring) ----
const int FEED_STEP = 2, FEED_DIR = 3;
const int ARM_STEP = 4, ARM_DIR = 5;
const int ENABLE_PIN = 8;          // shared, active LOW
const int FEED_HOME_SW = 9;        // rear end of the stage, to GND when pressed
const int ARM_HOME_SW = 10;        // to GND when pressed
const int ESTOP_PIN = 11;          // normally-closed button to GND; open = stop

// ---- mechanics (calibrate these) ----
// Feed: lead screw pitch 2 mm (T8x2) direct, 200 step motor, 16 microsteps.
const float FEED_STEPS_PER_MM = 200.0 * 16 / 2.0;
const float FEED_TRAVEL_MM = 120.0;
const bool FEED_INVERT = false;
// Arm: 200 step motor, 16 microsteps, 10:1 planetary gearbox.
const float ARM_STEPS_PER_DEG = 200.0 * 16 * 10 / 360.0;
const float ARM_LIMIT_DEG = 100.0;
const bool ARM_INVERT = false;
// After the arm switch trips, how far to move to reach neutral (deg).
const float ARM_HOME_TO_NEUTRAL_DEG = -45.0;

const float FEED_SPEED_MM_S = 8.0, FEED_ACCEL_MM_S2 = 40.0;
const float ARM_SPEED_DEG_S = 90.0, ARM_ACCEL_DEG_S2 = 400.0;

AccelStepper feed(AccelStepper::DRIVER, FEED_STEP, FEED_DIR);
AccelStepper arm(AccelStepper::DRIVER, ARM_STEP, ARM_DIR);
bool homed = false;
bool stopped = false;
String line;

bool estop() { return digitalRead(ESTOP_PIN) == HIGH; }

// Run one stepper to its target; false if the E-stop opened on the way.
bool runTo(AccelStepper &m) {
  while (m.distanceToGo() != 0) {
    if (estop()) { m.setCurrentPosition(m.currentPosition()); stopped = true; homed = false; return false; }
    m.run();
  }
  return true;
}

// Creep toward a switch at constant speed until it closes.
bool homeAxis(AccelStepper &m, int sw, long dir, float speed) {
  float normal = m.maxSpeed();
  m.setMaxSpeed(speed);
  m.setSpeed(dir * speed);
  while (digitalRead(sw) == HIGH) {
    if (estop()) { m.setMaxSpeed(normal); stopped = true; return false; }
    m.runSpeed();
  }
  m.setCurrentPosition(0);
  m.setMaxSpeed(normal);
  return true;
}

void reply(const String &s) { Serial.println(s); }

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT); digitalWrite(ENABLE_PIN, LOW);
  pinMode(FEED_HOME_SW, INPUT_PULLUP);
  pinMode(ARM_HOME_SW, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  feed.setPinsInverted(FEED_INVERT, false, false);
  arm.setPinsInverted(ARM_INVERT, false, false);
  feed.setMaxSpeed(FEED_SPEED_MM_S * FEED_STEPS_PER_MM);
  feed.setAcceleration(FEED_ACCEL_MM_S2 * FEED_STEPS_PER_MM);
  arm.setMaxSpeed(ARM_SPEED_DEG_S * ARM_STEPS_PER_DEG);
  arm.setAcceleration(ARM_ACCEL_DEG_S2 * ARM_STEPS_PER_DEG);
  reply("retainer-bender ready");
}

void handle(String cmd) {
  cmd.trim();
  int sp = cmd.indexOf(' ');
  String op = sp < 0 ? cmd : cmd.substring(0, sp);
  op.toUpperCase();
  float val = sp < 0 ? 0 : cmd.substring(sp + 1).toFloat();

  if (op == "STATUS") {
    Serial.print("feed_mm "); Serial.print(feed.currentPosition() / FEED_STEPS_PER_MM, 3);
    Serial.print(" arm_deg "); Serial.print(arm.currentPosition() / ARM_STEPS_PER_DEG, 2);
    Serial.print(homed ? " homed" : " not-homed");
    Serial.println(estop() ? " ESTOP" : "");
    reply("ok"); return;
  }
  if (op == "OFF") { digitalWrite(ENABLE_PIN, HIGH); homed = false; reply("ok"); return; }
  if (estop()) { reply("error: e-stop is open"); return; }

  if (op == "HOME") {
    digitalWrite(ENABLE_PIN, LOW);
    stopped = false;
    if (!homeAxis(arm, ARM_HOME_SW, 1, 20 * ARM_STEPS_PER_DEG)) { reply("error: e-stop while homing"); return; }
    arm.moveTo(lround(ARM_HOME_TO_NEUTRAL_DEG * ARM_STEPS_PER_DEG));
    if (!runTo(arm)) { reply("error: e-stop while homing"); return; }
    arm.setCurrentPosition(0);
    if (!homeAxis(feed, FEED_HOME_SW, -1, 4 * FEED_STEPS_PER_MM)) { reply("error: e-stop while homing"); return; }
    homed = true;
    reply("ok"); return;
  }
  if (!homed) { reply("error: not homed; send HOME"); return; }

  if (op == "FEED" || op == "JOG") {
    if (op == "FEED" && val < 0) { reply("error: FEED must be positive; use JOG"); return; }
    float target = feed.currentPosition() / FEED_STEPS_PER_MM + val;
    if (target < -0.01 || target > FEED_TRAVEL_MM) { reply("error: past stage travel"); return; }
    if (fabs(arm.currentPosition()) > 2) { reply("error: arm not at neutral"); return; }
    feed.moveTo(lround(target * FEED_STEPS_PER_MM));
    if (!runTo(feed)) { reply("error: e-stop"); return; }
    reply("ok"); return;
  }
  if (op == "ARM") {
    if (fabs(val) > ARM_LIMIT_DEG) { reply("error: past arm limit"); return; }
    arm.moveTo(lround(val * ARM_STEPS_PER_DEG));
    if (!runTo(arm)) { reply("error: e-stop"); return; }
    reply("ok"); return;
  }
  reply("error: unknown command " + op);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) handle(line);
      line = "";
    } else if (line.length() < 64) {
      line += c;
    }
  }
}
