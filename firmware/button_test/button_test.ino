const int BTN_CLIMB = 4;
const int BTN_DESCEND = 5;
const int BTN_TRICK = 6;

void setup() {
  Serial.begin(115200);
  pinMode(BTN_CLIMB, INPUT_PULLUP);
  pinMode(BTN_DESCEND, INPUT_PULLUP);
  pinMode(BTN_TRICK, INPUT_PULLUP);
}

void loop() {
  Serial.printf("climb:%d descend:%d trick:%d\n",
    digitalRead(BTN_CLIMB) == LOW,
    digitalRead(BTN_DESCEND) == LOW,
    digitalRead(BTN_TRICK) == LOW);
  delay(100);
}