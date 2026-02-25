#include <Servo.h>
#include <math.h>

// ---------------- Pins ----------------
#define PIN_RELAIS            4
#define PIN_SPEED_CONTR       5
#define PIN_STEER_SERV        6

// --------------- Servos ---------------
Servo esc;
Servo steer;

// ----------- Tunable params -----------
const double SPEED_FACTOR     = 1.0;
const double YAWRATE_FACTOR   = 1.0;
const double SPEED_MAX_MS     = 22.0 / 3.6;
const double YAWRATE_MAX      = 0.60;

const int ESC_NEUTRAL         = 93;
const int ESC_RANGE           = 60;
const int STEER_CENTER        = 90;
const int STEER_RANGE         = 34;
const int STEER_TRIM_MAX      = 4;

const unsigned long MSG_TIMEOUT_MS = 400;

// ----------- State variables ----------
double cmdSpeedMs   = 0.0;
double cmdYawRate   = 0.0;
unsigned long lastMsgTime = 0;
bool inTimeout = true;

// Static buffer for serial parsing
char serialBuf[32];
int bufIdx = 0;

void applyFailsafe() {
  cmdSpeedMs = 0.0;
  cmdYawRate = 0.0;
}

int speedToEscCmd(double speedMs) {
  speedMs = constrain(speedMs, -SPEED_MAX_MS, SPEED_MAX_MS);
  double norm = speedMs / SPEED_MAX_MS;
  return constrain((int)lround(ESC_NEUTRAL + norm * ESC_RANGE), 0, 180);
}

int yawRateToSteerCmd(double yawRate) {
  yawRate = constrain(yawRate, -YAWRATE_MAX, YAWRATE_MAX);
  double norm = yawRate / YAWRATE_MAX;
  return constrain((int)lround(STEER_CENTER + (-norm) * STEER_RANGE), 0, 180);
}

void parseAndSet(char* line) {
  String s = String(line);
  int ix = s.indexOf("x:");
  int iz = s.indexOf("z:");
  if (ix >= 0 && iz > ix) {
    cmdSpeedMs = s.substring(ix + 2, iz).toFloat() * SPEED_FACTOR;
    cmdYawRate = s.substring(iz + 2).toFloat() * YAWRATE_FACTOR;
    lastMsgTime = millis();
    if (inTimeout) {
      inTimeout = false;
      Serial.println("DEBUG: Link Restored");
    }
  }
} 

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELAIS, OUTPUT);
  digitalWrite(PIN_RELAIS, HIGH);
  esc.attach(PIN_SPEED_CONTR);
  steer.attach(PIN_STEER_SERV);
  esc.write(ESC_NEUTRAL);
  steer.write(STEER_CENTER);
  lastMsgTime = millis();
}

void loop() {
  // Non-blocking serial read
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf[bufIdx] = '\0';
      if (bufIdx > 0) parseAndSet(serialBuf);
      bufIdx = 0;
    } else if (bufIdx < 31) {
      serialBuf[bufIdx++] = c;
    }
  }

  // Failsafe logic
  if (millis() - lastMsgTime > MSG_TIMEOUT_MS) {
    if (!inTimeout) {
      inTimeout = true;
      Serial.println("DEBUG: Failsafe Active");
    }
    applyFailsafe();
  }

  esc.write(speedToEscCmd(cmdSpeedMs));
  steer.write(yawRateToSteerCmd(cmdYawRate));
}
