#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <math.h>

//Configure the destination here

//#define LATITUDE -31.9802627563
//#define LONGITUDE 115.8175659179
#define TOMETERS 111139
boolean there = false;
double home[2];
double rA[2], rC[2], a, b, c, angleA, angleB, angleC;
int numAway = 0;

double lattitude,longitude;

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

void forwards(int amount) {
  curve(2);
  delay(amount);
}

void stop(int amount) {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(amount);
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

void leftLean(int amount, int sec_amount) {
  curve(12);
  delay(amount);
  forwards(sec_amount);
}

void rightLean(int amount, int sec_amount) {
  curve(-10);
  delay(amount);
  forwards(sec_amount);
}

void uTurn() {
  curve(16);
  delay(3200);
}

void readCoords() {
  while(1) {
    while (serial_connection.available() > 0) { 
      gps.encode(serial_connection.read()); // idk what this does but it doesnt work without.
    }

    if (gps.location.isUpdated()) {
      lattitude=gps.location.lat();
      longitude=gps.location.lng();
      break;
    }
  }
}

void printCoords() {
  Serial.println("Satellite Count:");
  Serial.println(gps.satellites.value());
  Serial.println("Latitude:");
  Serial.println(lattitude, 10);
  Serial.println("Longitude:");
  Serial.println(longitude, 10);
}

void setup() {
  // NOTE: enA, enB are connected to analog pins, can out anything from 0 to 1.
  // switch code.
  pinMode(14 + 1, OUTPUT);
  pinMode(14 + 3, OUTPUT);
  pinMode(14 + 2, INPUT);
  digitalWrite(14+1, LOW);
  digitalWrite(14+3, HIGH);
  
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
    delay(500);
  }

  readCoords();
  printCoords();
  while(!digitalRead(14 + 2)) {delay(10);}
  readCoords();
  double LATITUDE = lattitude;
  double LONGITUDE = longitude;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  // Satellite connection established! Wait 5 seconds for satellite to stablize 
  delay(5000);

  while(digitalRead(14 + 2)) {delay(10);}
// Initialising variables for the car routing
  readCoords();
  home[0] = lattitude;
  home[1] = longitude;
  digitalWrite(LED_BUILTIN, LOW);

  rC[0] = LATITUDE;
  rC[1] = LONGITUDE;
  forwards(7000);
  stop(200);
  readCoords();
  rA[0] = lattitude;
  rA[1] = longitude;
  forwards(1000);
  stop(200);
  
}

void loop() { // If any gps data was collected
    readCoords();
    printCoords();
    double rB[2] = {lattitude, longitude};


    double rAB[2] = {rB[0] - rA[0], rB[1] - rA[1]};
    double rAC[2] = {rC[0] - rA[0], rC[1] - rA[1]};
    double rBC[2] = {rC[0] - rB[0], rC[1] - rB[1]};

    a = sqrt(rAB[0]*rAB[0] + rAB[1]*rAB[1]);
    b = sqrt(rBC[0]*rBC[0] + rBC[1]*rBC[1]);
    c = sqrt(rAC[0]*rAC[0] + rAC[1]*rAC[1]);

    //Calculating the angle (in Radians)
    angleB = acos((a*a - b*b + c*c)/(2*a*c));
    angleC = acos((a*a - c*c + b*b)/(2*a*b));
    angleA = acos((c*c - a*a + b*b)/(2*c*b));

    double theta = acos((rAB[0]*rAC[0] + rAB[1]*rAC[1]) / a / c);
    printCoords();
    // Update the rA value to that of current positon)
    
    rA[0] = rB[0];
    rA[1] = rB[1];

    if(isnan(angleA) || isnan(angleB) || isnan(angleC)) {
      return;
    }

    
    if (c * TOMETERS < 4 && !there) {
        uTurn();
        uTurn();
        uTurn();
        uTurn();
        uTurn();
        uTurn();
        stop(5000);
        there = true;
        rC[0] = home[0];
        rC[1] = home[1];
        uTurn();
        stop(800);
        return;
    } else if (c * TOMETERS < 4 && there) {
        uTurn();
        uTurn();
        uTurn();
        uTurn();
        uTurn();
        uTurn();
      stop(20000);
    }


    // very simplistic measure to try to stop it going away form place.
    // can also check if moving away & angle is small - that is backwards too.
    // should really be up a little, insde of the otehr motor logic.
    if (b * TOMETERS > 15) {
      digitalWrite(LED_BUILTIN, LOW);
      // actively moved a bit backwards
      if (angleB / M_PI * 180 > 100) {
        uTurn();
        stop(800);
        forwards(7000);
      } else if (angleC / M_PI * 180 > 100) {
        forwards(7000);
        stop(800);
      }
      else {
        // logic to turn left / right
        if (theta / M_PI * 180 < 180) {
          leftLean(1300, 7000);
          stop(800);
        } else {
          rightLean(1300, 7000);
          stop(800);
        }
      }
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      if (angleB / M_PI * 180 > 100) {
        uTurn();
        forwards(5000);
        stop(800);
      } else if (angleC / M_PI * 180 > 100) {
        forwards(5000);
        stop(800);
      }
      else {
        // logic to turn left / right
        if (theta / M_PI * 180 < 180) {
          leftLean(1000, 3000);
          stop(800);
        } else {
          rightLean(1000, 3000);
          stop(800);
        }
      }
    }
}