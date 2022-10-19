#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <math.h>

//Configure the destination here

#define LATITUDE -31.9802627563
#define LONGITUDE 115.8175659179
#define ANGLETHRES 0.524 // 30deg in radians.
#define TOMETERS 111139
// Latitude:
// -31.9802627563
// Longitude:
// 115.8175659179
// Turn around!


double home[2];
double rA[2], rC[2], a, b, c, angleA, angleB, angleC;
int numAway = 0;

double lattitude,longitude;

SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data


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

  readCoords();
  printCoords();

  // Satellite connection established! Wait 5 seconds for satellite to stablize 
  delay(5000);

  // Initialising variables for the car routing
  readCoords();
  home[0] = lattitude;
  home[1] = longitude;

  rC[0] = LATITUDE;
  rC[1] = LONGITUDE;
  Serial.println("go forwards for a little pls...");
  readCoords();
  rA[0] = lattitude;
  rA[1] = longitude;
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
    Serial.print("\n theta = "); Serial.println(theta);

    Serial.print("angle == (B?)"); Serial.println(angleB / M_PI * 180);
    Serial.print("angle == (C?)"); Serial.println(angleC / M_PI * 180);
    Serial.print("angle == (A?)"); Serial.println(angleA / M_PI * 180);
    
    
    Serial.print("length == (a?)"); Serial.println(a * TOMETERS, 10);
    Serial.print("length == (b?)"); Serial.println(b * TOMETERS, 10);
    Serial.print("length == (c?)"); Serial.println(c * TOMETERS, 10);
    printCoords();
    // Update the rA value to that of current positon)
    
    rA[0] = rB[0];
    rA[1] = rB[1];

    if(isnan(angleA) || isnan(angleB) || isnan(angleC)) {
      Serial.print("it was nan : ("); Serial.println(angleA / M_PI * 180);
      return;
    }

    
    if (c * TOMETERS < 4) {
        Serial.println("\nYou're there!");
        Serial.println("Do a spiral!");
        delay(5000);
    }


    // very simplistic measure to try to stop it going away form place.
    // can also check if moving away & angle is small - that is backwards too.
    // should really be up a little, insde of the otehr motor logic.
    if (b * TOMETERS > 15) {
      // actively moved a bit backwards
      if (angleB / M_PI * 180 > 100) {
        Serial.println("\nTurn around!");
      } else if (angleC / M_PI * 180 > 100) {
        Serial.println("\nKeep going forwards!");
      }
      else {
        // logic to turn left / right
        if (theta / M_PI * 180 < 180) {
          Serial.println("Go left!");
        } else {
          Serial.println("Go right?!");
        }
      }
    } else {
      Serial.println("Within 15 meters!");

    }
    // yuppp
    delay(5000);
}