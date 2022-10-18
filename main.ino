#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"

// for tutorial on how to use L298
// naming conventions as per L298 diver specs.
int enA = 5;
int enB = 6;
int in1 = 4;
int in2 = 7;



SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
void setup()
{
  Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
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



// Example coordinate

double destination_lat = 50; // y_coordinate
double destination_long = 150; // x_coordinate

double current_lat;
double current_long;
double cur_distance;
double scale = 10000000;
double distance = sqrt((scale * gps.location.lat() - destination_lat)*(scale * gps.location.lat() - destination_lat) + (scale * gps.location.lng() - destination_long)*(scale *gps.location.lng() - destination_long));


void check_direction(){

  
  cur_distance = sqrt((scale * gps.location.lat() - destination_lat)*(scale *gps.location.lat() - destination_lat) + (scale *gps.location.lng() - destination_long)*(scale *gps.location.lng() - destination_long));
  double difference = cur_distance - distance;
  Serial.println("Current_distance");
  Serial.println(cur_distance);
  if(cur_distance > 10){
 
  Serial.println("Difference");
  Serial.println(difference);
    if(difference < 0){ // heading to the correct direction
        forwards();
        delay(3000);
        distance = cur_distance;
   
    }
    else{ // heading to the wrong direction
        right();
        delay(3000);
        forwards();
        delay(3000);
        distance = cur_distance;
      }
  }
}




void loop()
{
  while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.println("Satellite Count:");
    Serial.println(gps.satellites.value());
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 9);
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 9);
    Serial.println("Speed MPH:");
    Serial.println(gps.speed.mph());
    Serial.println("Altitude Feet:");
    Serial.println(gps.altitude.feet());
    Serial.println("");

    delay(1000000);
    check_direction();
    }
  
}

/*
 * $GPRMC,183729,A,3907.356,N,12102.482,W,000.0,360.0,080301,015.5,E*6F
$GPRMB,A,,,,,,,,,,,,V*71
$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75
$GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D
$GPGSV,2,1,08,02,43,088,38,04,42,145,00,05,11,291,00,07,60,043,35*71
$GPGSV,2,2,08,08,02,145,00,09,46,303,47,24,16,178,32,26,18,231,43*77
$PGRME,22.0,M,52.9,M,51.0,M*14
$GPGLL,3907.360,N,12102.481,W,183730,A*33
$PGRMZ,2062,f,3*2D
$PGRMM,WGS 84*06
$GPBOD,,T,,M,,*47
$GPRTE,1,1,c,0*07
$GPRMC,183731,A,3907.482,N,12102.436,W,000.0,360.0,080301,015.5,E*67
$GPRMB,A,,,,,,,,,,,,V*71
*/
/** NOTE:  I have been uploading through the arduino IDE, unsure if certain 
 *           headers need to be otherwise included 
 *         used https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/
 *           for info on how to use the L298 driver.         
 * */
// 
