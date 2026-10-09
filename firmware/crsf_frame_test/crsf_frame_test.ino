uint16_t ch[16];        // 16 channel values (11-bit numbers)
uint8_t  payload[22];   // the 22 packed bytes

void packChannels() {
  uint32_t bucket = 0;
  int bitsInBucket = 0;
  int byteIndex = 0;

  for (int i = 0; i < 16; i++) {
    bucket |= (uint32_t)ch[i] << bitsInBucket;     // C++

    bitsInBucket += 11;

    while (bitsInBucket >= 8) {
      payload[byteIndex] = bucket & 0xFF;
      byteIndex++;

      bucket >>= 8;
      bitsInBucket -= 8;
    }
  }
}

uint8_t crc8(uint8_t data[], int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];              // C++ (shortcut for crc = crc ^ data[i];)

    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {            // is the top bit 1?
        crc = (crc << 1) ^ 0xD5;            // C++ (no & 0xFF needed: uint8_t keeps only 8 bits)

      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

uint8_t frame[26];   // the complete CRSF frame

void buildFrame() {
  packChannels();                        // fill payload[0..21]

  frame[0] = 0xEE;                       // box 0: address (the TX module)
  frame[1] = 24;                         // box 1: length
  frame[2] = 0x16;                       // box 2: type = RC channels

  for (int i = 0; i < 22; i++) {
    frame[3 + i] = payload[i];           // boxes 3..24: payload
  }

  frame[25] = crc8(&frame[2], 23);       // box 25: CRC of the 23 bytes from box 2
}

void setup() {
  Serial.begin(115200);
  delay(2000);                              // give Serial Monitor time to open

  for (int i = 0; i < 16; i++) ch[i] = 0;   // all channels 0...
  ch[0] = 992;                              // ...except channel 1
  ch[1] = 172;                              // ...and channel 2

  buildFrame();

  for (int i = 0; i < 26; i++) {
    Serial.printf("%02X ", frame[i]);       // print the whole 26-byte frame
  }
  Serial.println();

  uint8_t test[] = {'1','2','3','4','5','6','7','8','9'};
  Serial.printf("CRC test: %02X\n", crc8(test, 9));
}

void loop() {}