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

/*
* @param int power, should be [0,255] controls power of motors.
*/
void forwards(int power) {
  //right wheel
  analogWrite(enA, power);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // left wheel
  analogWrite(enB, power);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

/*
* @param int power, should be [0,255] controls power of motors.
*/
void backwards(int power) {
  analogWrite(enA, power);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  analogWrite(enB, power);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

/*
* @param int power, should be [0,255] controls power of motors.
*/
void left(int power) {
  analogWrite(enA, power);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  analogWrite(enB, power);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

/*
* @param int power, should be [0,255] controls power of motors.
*/
void right(int power) {
  analogWrite(enA, power);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  analogWrite(enB, power);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void loop() {
    int power = 100;
    forwards(power);
    delay(5000);
    backwards(power);
    delay(5000);
    left(power);
    delay(5000);
    right(power);
    delay(5000);
}