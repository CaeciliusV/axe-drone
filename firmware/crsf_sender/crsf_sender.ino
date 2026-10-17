// crsf_sender: sends CRSF channel frames to the Ranger Nano
// Roll sweeps slowly; throttle and ARM held LOW (safe)

const int CRSF_TX_PIN = 17;   // to Ranger Nano socket 1
const int CRSF_RX_PIN = 18;   // not wired, just reserved
HardwareSerial CRSF(1);

uint16_t ch[16];
uint8_t payload[22];
uint8_t frame[26];

uint16_t usToCrsf(int us) {
  return 992 + ((us - 1500) * 8) / 5;
}

void packChannels() {
  memset(payload, 0, 22);
  uint32_t bitPos = 0;
  for (int i = 0; i < 16; i++) {
    uint32_t v = ch[i] & 0x07FF;          // 11 bits
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
  frame[0] = 0xEE;              // address: TX module
  frame[1] = 24;                // length = type + payload + crc
  frame[2] = 0x16;              // type: RC channels
  memcpy(&frame[3], payload, 22);
  frame[25] = crc8(&frame[2], 23);
}

void setup() {
  Serial.begin(115200);
  CRSF.begin(400000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, true);  // true = inverted

  for (int i = 0; i < 16; i++) ch[i] = usToCrsf(1500);
  ch[2] = usToCrsf(1000);   // throttle LOW
  ch[4] = usToCrsf(1000);   // AUX1 (ARM) LOW

  Serial.println("crsf_sender running");
}

void loop() {
  static uint32_t lastSend = 0;
  if (micros() - lastSend >= 4000) {      // 250 frames per second
    lastSend += 4000;
    int rollUs = 1500 + (int)(400 * sin(millis() / 1000.0));
    ch[0] = usToCrsf(rollUs);
    buildFrame();
    CRSF.write(frame, 26);
  }
    static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("roll us: ");
    Serial.println(1500 + (int)(400 * sin(millis() / 1000.0)));
  }
}