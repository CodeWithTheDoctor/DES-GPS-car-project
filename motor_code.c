/** NOTE:  I have been uploading through the arduino IDE, unsure if certain 
 *           headers need to be otherwise included 
 *         used https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/
 *           for info on how to use the L298 driver.         
 * */
// 
// for tutorial on how to use L298
// naming conventions as per L298 diver specs.
int enA = 5;
int enB = 6;
int in1 = 4;
int in2 = 7;

void setup() {
  // NOTE: enA, enB are analog pins, can out anything from 0 to 255.
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
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