#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include "SoftwareSerial.h"
#include <Math.h>
#include <bool.h>

// for tutorial on how to use L298
// naming conventions as per L298 diver specs.
int enA = 5;
int enB = 6;
int in1 = 3;
int in2 = 4;
int in3 = 8;
int in4 = 7;


// The coordinates of the destination. 
double destination_coords_x = -31.979590;
double destination_coords_y = 115.817867;


// TODO: threshold was estimated. Should double check what to precisly do with it.
double threshold = 0.000001;


// NOTE: split up arrays halfway through, since pointers were being annoying to figure out quickly.
double home_coords_x = 0;
double home_coords_y = 0;

double curr_coords_x = 0;
double curr_coords_y = 0;


SoftwareSerial serial_connection(10, 9); //RX=pin 10, TX=pin 9 (Connect GPS modules RX to the Arduino's TX input and the modules TX to ardunio's RX input).
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data


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
void right(int power) {
  analogWrite(enA, power);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  analogWrite(enB, power);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
}

void current_coords(double *coords_x, double *coords_y) {
  *coords_x = gps.location.lat();
  *coords_y = gps.location.lng();
}

int scale = 1;

double calc_distance( double old_x, double old_y, double new_x, double new_y) {
  return sqrt(fabs(pow((scale * new_x - scale * old_x), 2) + pow((scale * new_y - scale * old_y), 2)));
}

void goto_dest(bool home) {
  double destination_x = home ? home_coords_x : destination_coords_x;
  double destination_y = home ? home_coords_y : destination_coords_y;

  double position_x;
  double position_y;
  current_coords(&position_x, &position_y);

  double distance = calc_distance(position_x, position_y, destination_x, destination_y);
  double old_distance = 10000000000000; // initialise large.

  while(distance > threshold*scale) {
    if (distance > old_distance) {
      // turn right slightly
      right(20);
      delay(100);
      forwards(100);
    } else {
      // keep going forwards.
      forwards(100);
    }

    // @ 10Hz
    delay(100);
    current_coords(&position_x, &position_y);
    old_distance = distance;
    distance = calc_distance(position_x, position_y, destination_x, destination_y);
  }
}

void setup()
{
  Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  currentCoords(&home_coords_x, &home_coords_y);
  currentCoords(&curr_coords_x, &curr_coords_y);
}

void loop() {
  goto_dest(false);
  goto_dest(true);
}

