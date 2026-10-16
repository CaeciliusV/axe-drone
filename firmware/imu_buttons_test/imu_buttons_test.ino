#include <Wire.h>
#include <math.h>

const int BTN_CLIMB = 4;
const int BTN_DESCEND = 5;
const int BTN_TRICK = 6;
const uint8_t IMU_ADDR = 0x6A;   // change if the scan found a different address

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)IMU_ADDR, 1);
  return Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(BTN_CLIMB, INPUT_PULLUP);
  pinMode(BTN_DESCEND, INPUT_PULLUP);
  pinMode(BTN_TRICK, INPUT_PULLUP);

  Wire.begin(8, 9);
  uint8_t id = readReg(0x0F);               // WHO_AM_I register
  Serial.printf("IMU ID: 0x%02X (should be 0x6C)\n", id);

  writeReg(0x10, 0x40);                     // turn accelerometer on: 104 Hz, ±2g
}

void loop() {
  // read 6 bytes: X, Y, Z (each low byte then high byte)
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x28);
  Wire.endTransmission(false);
  Wire.requestFrom((int)IMU_ADDR, 6);

  int16_t raw[3];
  for (int i = 0; i < 3; i++) {
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }

  // convert to g (±2g range: 0.061 mg per step)
  float ax = raw[0] * 0.061f / 1000.0f;
  float ay = raw[1] * 0.061f / 1000.0f;
  float az = raw[2] * 0.061f / 1000.0f;

  float roll  = atan2(ay, az) * 180.0f / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  Serial.printf("roll:%6.1f pitch:%6.1f | climb:%d descend:%d trick:%d\n",
    roll, pitch,
    digitalRead(BTN_CLIMB) == LOW,
    digitalRead(BTN_DESCEND) == LOW,
    digitalRead(BTN_TRICK) == LOW);

  delay(100);
}