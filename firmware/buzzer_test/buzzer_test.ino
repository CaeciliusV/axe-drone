const int BUZZER = 7;

void setup() {
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  tone(BUZZER, 2000, 200);   // 2000 Hz for 200 ms
  delay(1000);               // wait 1 second
}