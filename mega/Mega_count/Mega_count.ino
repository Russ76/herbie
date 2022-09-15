// Arduino Mega 
// Works with AI Python script for Oak-D weed sprayer machine "Herbie"
// Encoder messages utilized for speed sensing, 3 speed ranges.
// AI script selects from 3 camera views according to machine speed
// to allow for more precise spray control. 
// Spray wand moves in only one dimension, as machine moves forward.
// Encoder setup

#define ENCODER_OPTIMIZE_INTERRUPTS //Only for Teensy or Arduino
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

#include <ros.h>
// #include "ros/time.h"
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <Wire.h>
// Relay pin is controlled with D8. The active wire is connected to Normally Closed and common
int relay = 8;
volatile byte relayState = LOW;
long vl, vr, savedLeft, savedRight, distance;
int z=0;
int d1=0;
int d2=0;
int d3=0;
int d4=0;
long low_wrap = -2147483647;
long high_wrap = 2147483647;

int LED1 = 5;      // _slow_
int led2 = 6;  // medium
int led3 = 7;  // _fast_
// char speed[7] = "      ";


Encoder encoderRight( 3, 2 ); // Pins on Mega for MOTOR A encoder data (Had to be reversed!)
Encoder encoderLeft( 19, 18 ); // pins on Mega for MOTOR B encoder data

ros::NodeHandle nh;


// std_msgs::String str_msg;
std_msgs::Int32 encoder_left_value, encoder_right_value, distance_value;
ros::Publisher encoder_left_pub("encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("encoder_right_value", &encoder_right_value);
// ros::Publisher herbie_speed_pub("herbie_speed", &str_msg);
ros::Publisher distance_pub("distance_value", &distance_value);

 
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  
  pinMode(LED1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  nh.loginfo("Wheel Encoders:");
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);
//  nh.advertise(herbie_speed_pub);
  nh.advertise(distance_pub);
//  nh.advertise(imu_pub);
  
  
while (!nh.connected())
{
  nh.spinOnce();
}
  nh.loginfo("MEGA CONNECTED");
  delay(1);
//motorTimer = millis();
}

void loop()
{
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();
  if (newLeft > high_wrap) {newLeft  = 0;
  }
  if (newLeft < low_wrap)  {newLeft  = 0;
  }
  if (newRight > high_wrap){newRight = 0;
  }
  if (newRight < low_wrap) {newRight = 0;
  }
  
  vl = (newLeft  - savedLeft );
  vr = (newRight - savedRight);
  d1 = (vl + vr)/2;
  distance = int((d1 + d2 + d3 + d4)/40);
//  nh.loginfo("Distance = ", distance);
  
  if (distance > 5){
//    char speed[7] = " fast ";
// Normally, turn on pump for spray enable
    digitalWrite(relay, LOW);
    digitalWrite(led3, HIGH);
    digitalWrite(led2, LOW);
    digitalWrite(LED1, LOW);
  }
  else if (distance < 1){
//    char speed[7] = "stoped";
// Here, turn off pump, machine stopped, reverse or turning sharp
    digitalWrite(relay, HIGH);
    digitalWrite(LED1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }
  else if (distance < 3){
//    char speed[7] = " slow ";
    digitalWrite(relay, LOW);
    digitalWrite(LED1, HIGH);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
  }
  else{ // (distance < 5 > 3)
//    char speed[7] = "medium";
    digitalWrite(relay, LOW);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, LOW);
    digitalWrite(LED1, LOW);
  }
  
  encoder_left_value.data = newLeft;
  encoder_right_value.data = newRight;
//  publish two encoder messages
  encoder_left_pub.publish( &encoder_left_value );
  encoder_right_pub.publish( &encoder_right_value );

  z = z + 1;
  // publish distance less frequently for nn script
  if (z > 10){
    distance_value.data = distance;
    distance_pub.publish(&distance_value);
    z=0;
  }

 
  nh.spinOnce();

  delay(20);
//  nLastCompute = nNow;
  savedLeft  = newLeft ;
  savedRight = newRight;
  d4 = d3;
  d3 = d2;
  d2 = d1;
  
}
