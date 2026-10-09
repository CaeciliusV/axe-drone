# CRSF frame test

Builds one CRSF RC_CHANNELS_PACKED frame (type 0x16) on the ESP32 and prints it.
Tested 9 Oct 2026.

Input: ch1 = 992 (centre), ch2 = 172 (min), all others 0

Output:
EE 18 16 E0 63 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 C9
CRC test: BC   (correct check value for CRC8 DVB-S2 on "123456789")

Next: send this frame over UART to the Ranger Nano every 4 ms (stage 5).
