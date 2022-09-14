#define L_WHEEL_POS 10
#define L_WHEEL_NEG 11
#define R_WHEEL_POS 12
#define R_WHEEL_NEG 13

void setup() {
  Serial.begin(9600);
  pinMode(L_WHEEL_POS, OUTPUT);
  pinMode(L_WHEEL_NEG, OUTPUT);
  pinMode(R_WHEEL_POS, OUTPUT);
  pinMode(R_WHEEL_NEG, OUTPUT);
}

void moveForward() {
  digitalWrite(L_WHEEL_POS, HIGH);
  digitalWrite(L_WHEEL_NEG, LOW);
  digitalWrite(R_WHEEL_POS, HIGH);
  digitalWrite(R_WHEEL_NEG, LOW);
}

void moveBackwards() {
  digitalWrite(L_WHEEL_POS, LOW);
  digitalWrite(L_WHEEL_NEG, HIGH);
  digitalWrite(R_WHEEL_POS, LOW);
  digitalWrite(R_WHEEL_NEG, HIGH);
}

void test() {
  moveForward();
  delay(2000);
  moveBackwards();
  delay(2000);
}

void loop() {
  test();
}
