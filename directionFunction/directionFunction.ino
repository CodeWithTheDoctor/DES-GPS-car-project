#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <math.h>

//Configure the destination here

#define LATITUDE 00.00
#define LONGITUDE 00.00

SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
void setup()
{
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
  double home[2] = [gps.satellites.lat(), gps.satellites.lng()];
  double rC[2] = [LATITUDE, LONGITUDE]; 
  double rA[2] = [C[0], C[1]];

  // Todo: Code to Start driving forward
  

}

void loop()
{
  while(serial_connection.available()) // While there is a connection between the nano and the gps module
  {
    gps.encode(serial_connection.read()); //This feeds the serial NMEA data into the library one char at a time

  }
  if(gps.location.isUpdated()) // If any gps data was collected
  {

    sleep(2000);
    double rB[2] = [gps.satellites.lat(), gps.satellites.lng()];

    double rAB[2] = [rB[0] - rA[0], rB[1] - rB[1]];
    double rAC[2] = [rC[0] - rA[0], rC[1] - rA[1]];
    double rBC[2] = [rC[0] - rB[0], rC[1] - rB[1]];

    double a = sqrt(rAB[0]**2 + rAB[1]**2);
    double b = sqrt(rBC[0]**2 + rBC[1]**2);
    double c = sqrt(rAC[0]**2 + rAC[1]**2);

    //Calculating the angle (in Radians)

    double angle = acos((a**2 - b**2 + c**2)/(2*a*c));

    // Update the rA value to that of current positon)
    rA = [rB[0], rB[1]]; 

    // Checking which manuever to execute based on angle and the location of the destination coordinate relative to current position, rB.
    int angleFluctionThreshold = 0.0872665; // in Radians

    if(angle < 0 + angleFluctionThreshold) {
      continue;
    }

    else if (rAB[0] > rAC[0]) leftLean(2); 
    // else if(rAB[1] > rAC[1]) rightLean(2);
    else rightLean(2);





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