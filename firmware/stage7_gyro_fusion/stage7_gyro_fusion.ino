// Stage 7: gyro + accel fusion (complementary filter), debounced ARM switch
// Everything from Stage 6 kept: ARM, kill-disarm, pause, buzzer, CRSF

#include <Wire.h>

// ---------- Pins ----------
const int CLIMB_PIN    = 4;    // yellow
const int DESCEND_PIN  = 5;    // green
const int TRICK_PIN    = 6;    // blue (pause)
const int BUZZER_PIN   = 7;
const int ARM_PIN      = 10;   // ARM rocker (LOW = ON)
const int FAKE_GND_PIN = 11;   // ARM rocker's 2nd wire (driven LOW = GND). Move to GND rail on perfboard.
const int CRSF_TX_PIN  = 17;
const int CRSF_RX_PIN  = 18;
const uint8_t IMU_ADDR = 0x6A;

// ---------- Tuning ----------
const float MAX_ANGLE     = 30.0;
const float DEADZONE      = 3.0;
const int   STICK_RANGE   = 400;
const float THROTTLE_RATE = 300.0;
const float ALPHA         = 0.98;   // gyro trust (0.95 = more accel, 0.99 = more gyro)
const float ROLL_SIGN     = 1.0;    // flips the OUTPUT direction
const float PITCH_SIGN    = 1.0;
const float GX_SIGN       = 1.0;    // flips the GYRO direction if it fights the accel
const float GY_SIGN       = 1.0;
const uint32_t KILL_LOCKOUT_MS    = 1500;
const uint32_t DEBOUNCE_MS        = 50;
const uint32_t SWITCH_DEBOUNCE_MS = 30;

// ---------- CRSF ----------
HardwareSerial CRSF(1);
uint16_t ch[16];
uint8_t payload[22];
uint8_t frame[26];

// ---------- State ----------
float rollZero = 0, pitchZero = 0;      // level offsets
float gyroBiasX = 0, gyroBiasY = 0;     // gyro offsets while still
float rollFused = 0, pitchFused = 0;    // filter output (degrees)
float lastAccRoll = 0, lastAccPitch = 0;
float throttleUs = 1000;

bool armed = false;
bool armBlocked = true;
bool paused = false;
bool killLatched = false;
uint32_t killTime = 0;

// ===== Buzzer (non-blocking) =====
int beepsLeft = 0;
uint16_t beepFreq = 0, beepOnMs = 0, beepGapMs = 0;
uint32_t beepNext = 0;

void startBeeps(int count, uint16_t freq, uint16_t onMs, uint16_t gapMs) {
  beepsLeft = count; beepFreq = freq; beepOnMs = onMs; beepGapMs = gapMs;
  beepNext = millis();
}

void updateBeeps() {
  if (beepsLeft > 0 && (int32_t)(millis() - beepNext) >= 0) {
    tone(BUZZER_PIN, beepFreq, beepOnMs);
    beepsLeft--;
    beepNext = millis() + beepOnMs + beepGapMs;
  }
}

void beepArm()    { startBeeps(2, 2500,  80,  80); }
void beepDisarm() { startBeeps(1, 1000, 400,   0); }
void beepKill()   { startBeeps(3, 1000, 100,  60); }
void beepPause()  { startBeeps(1, 2000, 100,   0); }
void beepResume() { startBeeps(2, 2000, 100,  80); }
void beepError()  { startBeeps(4, 3000,  50,  50); }

// ===== IMU =====
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Reads gyro (0x22-0x27) and accel (0x28-0x2D) in one burst
bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x22);                                   // OUTX_L_G
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)12) != 12) return false;

  uint8_t b[12];
  for (int i = 0; i < 12; i++) b[i] = Wire.read();
  int16_t raw[6];
  for (int i = 0; i < 6; i++) raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));

  gx = raw[0] * 0.0175f;            // deg/s (+/-500 dps range)
  gy = raw[1] * 0.0175f;
  gz = raw[2] * 0.0175f;
  ax = raw[3] * 0.061f / 1000.0f;   // g (+/-2g range)
  ay = raw[4] * 0.061f / 1000.0f;
  az = raw[5] * 0.061f / 1000.0f;
  return true;
}

void accelAngles(float ax, float ay, float az, float &roll, float &pitch) {
  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// ===== Mapping =====
int angleToUs(float a) {
  if (fabs(a) < DEADZONE) return 1500;
  float s = (a > 0) ? (a - DEADZONE) : (a + DEADZONE);
  s = s / (MAX_ANGLE - DEADZONE);
  if (s > 1) s = 1;
  if (s < -1) s = -1;
  return 1500 + (int)(s * STICK_RANGE);
}

uint16_t usToCrsf(int us) {
  return 992 + ((us - 1500) * 8) / 5;
}

// ===== CRSF frame =====
void packChannels() {
  memset(payload, 0, 22);
  uint32_t bitPos = 0;
  for (int i = 0; i < 16; i++) {
    uint32_t v = ch[i] & 0x07FF;
    for (int b = 0; b < 11; b++) {
      if (v & (1UL << b)) payload[bitPos / 8] |= (1 << (bitPos % 8));
      bitPos++;
    }
  }
}

uint8_t crc8(const uint8_t *data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
      else crc <<= 1;
    }
  }
  return crc;
}

void buildFrame() {
  packChannels();
  frame[0] = 0xEE;
  frame[1] = 24;
  frame[2] = 0x16;
  memcpy(&frame[3], payload, 22);
  frame[25] = crc8(&frame[2], 23);
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  pinMode(CLIMB_PIN, INPUT_PULLUP);
  pinMode(DESCEND_PIN, INPUT_PULLUP);
  pinMode(TRICK_PIN, INPUT_PULLUP);
  pinMode(ARM_PIN, INPUT_PULLUP);
  pinMode(FAKE_GND_PIN, OUTPUT);
  digitalWrite(FAKE_GND_PIN, LOW);

  Wire.begin(8, 9);
  Wire.setClock(400000);
  writeReg(0x10, 0x50);   // CTRL1_XL: accel 208 Hz, +/-2g
  writeReg(0x11, 0x54);   // CTRL2_G:  gyro 208 Hz, +/-500 dps
  delay(100);

  CRSF.begin(400000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, true);

  for (int i = 0; i < 16; i++) ch[i] = usToCrsf(1500);
  ch[2] = usToCrsf(1000);
  ch[4] = usToCrsf(1000);

  // Calibration: keep IMU flat (chip side up) and STILL
  delay(1500);
  Serial.println("Calibrating - keep IMU flat, chip up, and still...");
  float rSum = 0, pSum = 0, gxSum = 0, gySum = 0;
  int good = 0;
  for (int i = 0; i < 200; i++) {
    float ax, ay, az, gx, gy, gz, r, p;
    if (readIMU(ax, ay, az, gx, gy, gz)) {
      accelAngles(ax, ay, az, r, p);
      rSum += r; pSum += p; gxSum += gx; gySum += gy;
      good++;
    }
    delay(5);
  }
  if (good > 0) {
    rollZero  = rSum / good;   pitchZero  = pSum / good;
    gyroBiasX = gxSum / good;  gyroBiasY  = gySum / good;
    rollFused = rollZero;      pitchFused = pitchZero;   // start the filter at the real angle
  }
  Serial.printf("Calibrated. good samples = %d (of 200) | gyro bias X %.2f Y %.2f dps\n",
                good, gyroBiasX, gyroBiasY);

  if (digitalRead(ARM_PIN) == LOW) {
    Serial.println("WARNING: ARM switch is ON at startup - flip it OFF then ON to arm");
    beepError();
  } else {
    beepPause();
  }
}

// ===== Loop =====
void loop() {
  static uint32_t lastSend = micros();
  static uint32_t lastPrint = 0;
  static bool trickWasDown = false;
  static uint32_t trickChangeTime = 0;
  static bool switchRawLast = false;
  static bool switchStable = false;
  static uint32_t switchChangeTime = 0;

  updateBeeps();

  if (micros() - lastSend >= 4000) {           // 250 Hz
    uint32_t now = micros();
    float dt = (now - lastSend) / 1000000.0;
    lastSend = now;

    // --- Tilt: complementary filter ---
    float ax, ay, az, gx, gy, gz;
    if (readIMU(ax, ay, az, gx, gy, gz)) {
      accelAngles(ax, ay, az, lastAccRoll, lastAccPitch);
      float rateRoll  = GX_SIGN * (gx - gyroBiasX);
      float ratePitch = GY_SIGN * (gy - gyroBiasY);
      rollFused  = ALPHA * (rollFused  + rateRoll  * dt) + (1.0 - ALPHA) * lastAccRoll;
      pitchFused = ALPHA * (pitchFused + ratePitch * dt) + (1.0 - ALPHA) * lastAccPitch;
    }
    float rollAngle  = rollFused  - rollZero;
    float pitchAngle = pitchFused - pitchZero;

    // --- Inputs ---
    bool climb   = digitalRead(CLIMB_PIN) == LOW;
    bool descend = digitalRead(DESCEND_PIN) == LOW;
    bool trick   = digitalRead(TRICK_PIN) == LOW;

    // ARM switch with debounce: must be steady for 30 ms
    bool switchRaw = digitalRead(ARM_PIN) == LOW;
    if (switchRaw != switchRawLast) {
      switchRawLast = switchRaw;
      switchChangeTime = millis();
    }
    if (millis() - switchChangeTime >= SWITCH_DEBOUNCE_MS) switchStable = switchRaw;
    bool switchOn = switchStable;

    // --- 1. Kill combo ---
    if (climb && descend) {
      if (armed) {
        armed = false;
        paused = false;
        beepKill();
      }
      killLatched = true;
      killTime = millis();
    } else if (killLatched) {
      if (!climb && !descend && millis() - killTime >= KILL_LOCKOUT_MS) {
        killLatched = false;
      }
    }

    // --- 2. ARM switch ---
    if (!switchOn) {
      if (armed) {
        armed = false;
        paused = false;
        beepDisarm();
      }
      armBlocked = false;
    } else if (!armed && !armBlocked) {
      if (!killLatched && throttleUs <= 1000) {
        armed = true;
        beepArm();
      } else {
        beepError();
      }
      armBlocked = true;
    }

    // --- 3. Pause toggle ---
    if (trick != trickWasDown && millis() - trickChangeTime >= DEBOUNCE_MS) {
      trickWasDown = trick;
      trickChangeTime = millis();
      if (trick && armed) {
        paused = !paused;
        if (paused) beepPause(); else beepResume();
      }
    }

    // --- 4. Throttle ---
    if (!armed || killLatched) {
      throttleUs = 1000;
    } else if (!paused) {
      if (climb)        throttleUs += THROTTLE_RATE * dt;
      else if (descend) throttleUs -= THROTTLE_RATE * dt;
    }
    throttleUs = constrain(throttleUs, 1000, 2000);

    // --- 5. Roll / Pitch ---
    int rollUs  = paused ? 1500 : angleToUs(ROLL_SIGN * rollAngle);
    int pitchUs = paused ? 1500 : angleToUs(PITCH_SIGN * pitchAngle);

    // --- 6. Channels (AETR) ---
    ch[0] = usToCrsf(rollUs);
    ch[1] = usToCrsf(pitchUs);
    ch[2] = usToCrsf((int)throttleUs);
    ch[3] = usToCrsf(1500);
    ch[4] = usToCrsf(armed ? 2000 : 1000);

    buildFrame();
    CRSF.write(frame, 26);
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    float rollAngle  = rollFused  - rollZero;
    float pitchAngle = pitchFused - pitchZero;
    Serial.printf("%s%s | R %4d P %4d T %4d | roll fused %6.1f acc %6.1f | pitch fused %6.1f acc %6.1f | sw %d\n",
      armed ? "ARMED   " : "disarmed",
      paused ? " PAUSED" : "       ",
      paused ? 1500 : angleToUs(ROLL_SIGN * rollAngle),
      paused ? 1500 : angleToUs(PITCH_SIGN * pitchAngle),
      (int)throttleUs,
      rollAngle, lastAccRoll - rollZero,
      pitchAngle, lastAccPitch - pitchZero,
      digitalRead(ARM_PIN) == LOW);
  }
}