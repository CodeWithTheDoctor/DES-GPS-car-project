#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <math.h>

//Configure the destination here

#define LATITUDE -31.979984
#define LONGITUDE 115.817852

SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data


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

double home[2];
double rC[2];
double rA[2];


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


void setup() {
  // NOTE: enA, enB are connected to analog pins, can out anything from 0 to 1.
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // initialise motors, off.


  // ash code
  Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  
  pinMode(LED_BUILTIN, OUTPUT);

// Flickers the inbuilt led until the GPS module is ready to send data to the nanu
  while(!serial_connection.available()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  while(serial_connection.available()) {
    // LED will turn on and off until data from satelite starts being sent.
    if(gps.location.isUpdated()) {
      break;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
  }

  // Satellite connection established! Wait 5 seconds for satellite to stablize 
  delay(5000);

  // Initialising variables for the car routing
  home[0] = gps.location.lat();
  home[1] = gps.location.lng();
  rC[0] = LATITUDE;
  rC[1] = LONGITUDE;
  rA[0] = rC[0];
  rA[1] = rC[1];

  // Code to Start driving forward
  forwards();
}

void loop() {
    // delay(500);
    // for(int i = 0; i < 10; i++) {
    //   rightLean(500);
    //   stop();
    //   delay(800);
    //   leftLean(500);
    //   stop();
    //   delay(800); 
    // }

  while(serial_connection.available()) // While there is a connection between the nano and the gps module
  {
    gps.encode(serial_connection.read()); //This feeds the serial NMEA data into the library one char at a time

  }
  if(gps.location.isUpdated()) // If any gps data was collected
  {

    delay(2000);
    double rB[2] = {gps.location.lat(), gps.location.lng()};

    double rAB[2] = {rB[0] - rA[0], rB[1] - rB[1]};
    double rAC[2] = {rC[0] - rA[0], rC[1] - rA[1]};
    double rBC[2] = {rC[0] - rB[0], rC[1] - rB[1]};

    double a = sqrt(rAB[0]*rAB[0] + rAB[1]*rAB[1]);
    double b = sqrt(rBC[0]*rBC[0] + rBC[1]*rBC[1]);
    double c = sqrt(rAC[0]*rAC[0] + rAC[1]*rAC[1]);

    //Calculating the angle (in Radians)
    double angle = acos((a*a - b*b + c*c)/(2*a*c));

    // Update the rA value to that of current positon)
    rA[0] = rB[0];
    rA[1] = rB[1];

    // Checking which manuever to execute based on angle and the location of the destination coordinate relative to current position, rB.
    int angleFluctionThreshold = 0.0872665; // in Radians

    if ((angle > 0 + angleFluctionThreshold) && (rAB[0] > rAC[0])) leftLean(500); 
    // else if(rAB[1] > rAC[1]) rightLean(2);
    else rightLean(500);


    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    // Serial.println("Satellite Count:");
    // Serial.println(gps.satellites.value());
    // Serial.println("Latitude:");
    // Serial.println(gps.location.lat(), 6);
    // Serial.println("Longitude:");
    // Serial.println(gps.location.lng(), 6);
    // Serial.println("Speed MPH:");

  }
}