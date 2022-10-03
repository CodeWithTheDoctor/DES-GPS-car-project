// naming conventions as per L298 diver specs.
int enA = 5;
int enB = 6;
int in1 = 4;
int in2 = 7;

void setup() {
  // NOTE: enA, enB are connected to analog pins, can out anything from 0 to 1.
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // initialise motors, off.
}

void forwards() {
  analogWrite(enA, 0);
  digitalWrite(in1, HIGH);
  analogWrite(enB, 0);
  digitalWrite(in2, HIGH);
}

void backwards() {
  analogWrite(enA, 255);
  digitalWrite(in1, LOW);
  analogWrite(enB, 255);
  digitalWrite(in2, LOW);
}

void left() {
  analogWrite(enA, 0);
  digitalWrite(in1, HIGH);
  analogWrite(enB, 255);
  digitalWrite(in2, LOW);
}

void right() {
  analogWrite(enA, 255);
  digitalWrite(in1, LOW);
  analogWrite(enB, 0);
  digitalWrite(in2, HIGH);
}

void loop() {
    forwards();
    delay(5000);
    backwards();
    delay(5000);
    left();
    delay(5000);
    right();
    delay(5000);
}