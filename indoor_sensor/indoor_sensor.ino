const int PIRPIN = 19;

void setup() {
  Serial.begin(9600);
  pinMode(PIRPIN, INPUT);
}

void loop() {
  Serial.println(digitalRead(PIRPIN));
}
