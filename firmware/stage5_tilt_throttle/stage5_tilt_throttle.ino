// Stage 5: IMU tilt -> Roll/Pitch, ring buttons -> Throttle, sent over CRSF
// AUX1 (ARM) held LOW - cannot arm. No motors connected.

#include <Wire.h>

// ---------- Pins ----------
const int CLIMB_PIN   = 4;    // yellow
const int DESCEND_PIN = 5;    // green
const int TRICK_PIN   = 6;    // blue
const int CRSF_TX_PIN = 17;   // to Ranger Nano socket 1
const int CRSF_RX_PIN = 18;   // not wired
const uint8_t IMU_ADDR = 0x6A;

// ---------- Tuning ----------
const float MAX_ANGLE     = 30.0;   // degrees of tilt for full stick
const float DEADZONE      = 3.0;    // degrees ignored around level
const int   STICK_RANGE   = 400;    // +/-400 us around 1500
const float THROTTLE_RATE = 300.0;  // us per second while a button is held
const float FILTER        = 0.1;    // smoothing (0.05 smoother, 0.3 faster)
const float ROLL_SIGN     = 1.0;    // change to -1.0 if Roll moves the wrong way
const float PITCH_SIGN    = 1.0;    // change to -1.0 if Pitch moves the wrong way
const uint32_t KILL_LOCKOUT_MS = 1500;  // throttle locked this long after a kill

// ---------- CRSF ----------
HardwareSerial CRSF(1);
uint16_t ch[16];
uint8_t payload[22];
uint8_t frame[26];

// ---------- State ----------
float rollZero = 0, pitchZero = 0;   // calibration offsets
float rollF = 0, pitchF = 0;         // filtered angles
float throttleUs = 1000;

// ===== IMU =====
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readAngles(float &roll, float &pitch) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x28);                              // OUTX_L_A
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, (uint8_t)6) != 6) return false;

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  int16_t rx = (int16_t)(b[0] | (b[1] << 8));
  int16_t ry = (int16_t)(b[2] | (b[3] << 8));
  int16_t rz = (int16_t)(b[4] | (b[5] << 8));

  float ax = rx * 0.061f / 1000.0f;              // g
  float ay = ry * 0.061f / 1000.0f;
  float az = rz * 0.061f / 1000.0f;

  roll  = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  return true;
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

  Wire.begin(8, 9);
  Wire.setClock(400000);
  writeReg(0x10, 0x40);          // CTRL1_XL: 104 Hz, +/-2g
  delay(100);

  CRSF.begin(400000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, true);  // inverted = works

  for (int i = 0; i < 16; i++) ch[i] = usToCrsf(1500);
  ch[2] = usToCrsf(1000);        // throttle low
  ch[4] = usToCrsf(1000);        // AUX1 ARM low

  // Calibration: hold hand level and still
  delay(1500);
  Serial.println("Calibrating - keep hand level and still...");
  float rSum = 0, pSum = 0;
  int good = 0;
  for (int i = 0; i < 100; i++) {
    float r, p;
    if (readAngles(r, p)) { rSum += r; pSum += p; good++; }
    delay(5);
  }
  if (good > 0) { rollZero = rSum / good; pitchZero = pSum / good; }
  Serial.print("Calibrated. good samples = ");
  Serial.println(good);
}

// ===== Loop =====
void loop() {
  static uint32_t lastSend = micros();
  static uint32_t lastPrint = 0;

  if (micros() - lastSend >= 4000) {           // 250 Hz
    uint32_t now = micros();
    float dt = (now - lastSend) / 1000000.0;
    lastSend = now;

    // --- Tilt ---
    float r, p;
    if (readAngles(r, p)) {
      rollF  += FILTER * ((r - rollZero)  - rollF);
      pitchF += FILTER * ((p - pitchZero) - pitchF);
    }
    int rollUs  = angleToUs(ROLL_SIGN * rollF);
    int pitchUs = angleToUs(PITCH_SIGN * pitchF);

    // --- Throttle from buttons ---
    bool climb   = digitalRead(CLIMB_PIN) == LOW;
    bool descend = digitalRead(DESCEND_PIN) == LOW;
    static bool killLatched = false;
    static uint32_t killTime = 0;

    if (climb && descend) {                       // kill combo
      throttleUs = 1000;
      killLatched = true;
      killTime = millis();                        // timer restarts while both are held
    } else if (killLatched) {
      throttleUs = 1000;                          // stay locked
      if (!climb && !descend && millis() - killTime >= KILL_LOCKOUT_MS) {
        killLatched = false;                      // unlock only when both released AND 1.5 s passed
      }
    } else if (climb) {
      throttleUs += THROTTLE_RATE * dt;
    } else if (descend) {
      throttleUs -= THROTTLE_RATE * dt;
    }
    throttleUs = constrain(throttleUs, 1000, 2000);

    // --- Channels (AETR) ---
    ch[0] = usToCrsf(rollUs);          // Roll
    ch[1] = usToCrsf(pitchUs);         // Pitch
    ch[2] = usToCrsf((int)throttleUs); // Throttle
    ch[3] = usToCrsf(1500);            // Yaw (not used yet)
    ch[4] = usToCrsf(1000);            // AUX1 ARM - always low for now

    buildFrame();
    CRSF.write(frame, 26);
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("R %4d  P %4d  T %4d | roll %6.1f  pitch %6.1f | trick %d\n",
      angleToUs(ROLL_SIGN * rollF), angleToUs(PITCH_SIGN * pitchF), (int)throttleUs,
      rollF, pitchF, digitalRead(TRICK_PIN) == LOW);
  }
}