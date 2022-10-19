#include "TinyGPS++.h"
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <math.h>

//Configure the destination here

float lattitude,longitude;
SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

void setup() {
  // NOTE: enA, enB are connected to analog pins, can out anything from 0 to 1.

  // ash code
  Serial.begin(9600); //This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600); //This opens up communications to the GPS
  Serial.println("GPS Start"); //Just show to the monitor that the sketch has started
  delay(1000); // in caseys : )

}

// sets lattitude, longitude vars.
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

void loop() {
  readCoords();
  Serial.print("LATTITUDE="); Serial.println(lattitude,6);
  Serial.print("LONGITUDE="); Serial.println(longitude,6);
  delay(1000);
}