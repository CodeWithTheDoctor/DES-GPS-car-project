/** NOTE:  I have been uploading through the arduino IDE, unsure if certain 
 *         headers need to be otherwise included 
 *         
 * use https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/
 *  for info on how to use the L298 driver.         
 */
// naming conventions as per L298 diver specs.
int enA = 5;
int enB = 6;
int in1 = 3;
int in2 = 4;
int in3 = 8;
int in4 = 7;

int power = 100;


void setup() {
  // NOTE: enA, enB are connected to analog pins, can out anything from 0 to 1.
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // initialise motors, off.
}

void forwards() {
  curve(2);
}

void backwards() {
  analogWrite(enA, power);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  
  analogWrite(enB, power);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void left() {
  analogWrite(enA, 0);
  // right wheel
  analogWrite(enB, power - 30);
  delay(100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(100);
}

void right() {
   analogWrite(enA, power - 30);
  // right wheel
  analogWrite(enB, 0);
  delay(100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(100);
}

void stop() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/*
 * @param amount, positive curve left, negative curve right.
 */
void curve(int amount) {
  // left wheel
  analogWrite(enA, power - amount);
  // right wheel
  analogWrite(enB, power + amount);
  delay(100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(100);
}

void leftLean(int amount) {
  curve(12);
  delay(amount);
  forwards();
  delay(5000);
}

void rightLean(int amount) {
  curve(-10);
  delay(amount);
  forwards();
  delay(5000);
}

void loop() {
    delay(500);
    for(int i = 0; i < 10; i++) {
      rightLean(500);
      stop();
      delay(800);
      leftLean(500);
      stop();
      delay(800); 
    }
}