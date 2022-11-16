// Arduino Teensy 4.0 (or 3.2) and ROS(1) Noetic
// Works with AI Python script for Oak-D weed sprayer machine "Herbie"
// Encoder messages utilized for speed sensing, 3 speed ranges.
// AI script selects from 3 camera views according to machine speed
// to allow for more precise spray control. 
// Spray wand moves in only one dimension (side to side), as machine moves forward.
// Uses Sabertooth motor controller, 2x12 or 2x25
// Serial2 pin is #8
// Motor scale for needed input: -0.5 to 0.5
// Will work directly with Joy and teleop_twist_joy messages
// Callback routine multiplies number by 70 or more for full range
// Encoder messages published
// Adafruit_BNO055 code added. 
// Red to 3.3V, Black GRND, Yellow SCL 19, Blue SDA 18
// Next code version will incorporate three ultrasonic sensors
// Run Joy and Teleop_twist_joy on laptop to control machine speed
// Encoder setup

#define ENCODER_OPTIMIZE_INTERRUPTS //Only for Teensy or Arduino
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include <ros.h>
#include "ros/time.h"
#include <std_msgs/Int32.h>
// #include <std_msgs/String.h>
#include <Wire.h>
#include "sensor_msgs/Imu.h"
#include <Adafruit_Sensor.h>
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int pwrLeft = 64;  // start with brakes on
int pwrRight = 192; // this is Sabertooth's method of simplified serial
// Each motor has 7 bits of speed resolution. One wire used for both motors
// Angular boost makes turning stronger, useful for skid steer or tracked units
float angularboost = 1.2;
float linx;
float angZ;

// Relay pin is controlled with D8. The active wire is connected to Normally Closed and common
int relay = 2;
volatile byte relayState = LOW;
long vl, vr, savedLeft, savedRight, distance;
int z=0;
int d1=0;
int d2=0;
int d3=0;
int d4=0;
long low_wrap = -2147483647;
long high_wrap = 2147483647;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int LED1 = 3;  // _slow_
int led2 = 4;  // medium
int led3 = 5;  // _fast_

Encoder encoderRight( 9, 10 ); // Pins on Teensy for MOTOR A encoder data (Had to be reversed!)
Encoder encoderLeft( 11, 12 ); // pins on Teensy for MOTOR B encoder data

ros::NodeHandle nh;


// Callback

void messageCb( const geometry_msgs::Twist& msg)
{
  linx = msg.linear.x * 90;
  angZ = msg.angular.z * -70 * angularboost;

// Twist data converted into motor commands here
	
  if(linx == 0){  // turning
    pwrRight = angZ;
    pwrLeft = (-1) * pwrRight;
  }else if(angZ == 0){ // fordward / backward
    pwrLeft = pwrRight = linx;
  }else{ // moving doing arcs
    pwrLeft = linx - angZ;
    pwrRight = linx + angZ;
  }
  pwrLeft = pwrLeft + 64;
  if (pwrLeft > 127) pwrLeft = 127;
  if (pwrLeft < 1) pwrLeft = 1;
  pwrRight = pwrRight + 192;
  if (pwrRight > 255) pwrRight = 255;
  if (pwrRight < 128) pwrRight = 128;
  }

std_msgs::Int32 encoder_left_value, encoder_right_value, distance_value;
ros::Publisher encoder_left_pub("encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("encoder_right_value", &encoder_right_value);
ros::Publisher distance_pub("distance_value", &distance_value);
sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu/data", &imu_msg);
//ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCb);
ros::Subscriber<geometry_msgs::Twist> sub("/drive/cmd_vel", &messageCb );

 
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.loginfo("Wheel Encoders:");
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);  
  nh.advertise(imu_pub);
  nh.advertise(distance_pub);
  
  Serial2.begin(9600);
  
   /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
   
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  
  pinMode(LED1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  
while (!nh.connected())
{
  nh.spinOnce();
}
  nh.loginfo("TEENSY CONNECTED");
  delay(1);
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
    z = 0;
  }

  nh.spinOnce();

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();
  
  //nh.loginfo("In main loop now.");
	
  // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
	sensor_msgs::Imu imu_msg;
    ros::Time ros_time;
    imu_msg.header.stamp = ros_time;
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = quat.w();
    imu_msg.orientation.y = quat.x();
    imu_msg.orientation.z = quat.y();
    imu_msg.orientation.w = quat.z();
    imu_msg.linear_acceleration.x = (euler.x());
    imu_msg.linear_acceleration.y = (euler.y());
    imu_msg.linear_acceleration.z = (euler.z());
    imu_msg.angular_velocity.x = (gyro.x());
    imu_msg.angular_velocity.y = (gyro.y());
    imu_msg.angular_velocity.z = (gyro.z());
  // https://github.com/Russ76/ros_mpu6050_node-1/blob/master/src/mpu6050_node.cpp
	imu_pub.publish(&imu_msg);
		
  Serial2.write(pwrLeft); // motor speed and stop
  Serial2.write(pwrRight); // Power motors both directions
  
  delay(25);
  // With this large delay she was publishing about 35 messages/second on 3 topics
  
  //  Save old data for calculations
  savedLeft  = newLeft ;
  savedRight = newRight;
  d4 = d3;
  d3 = d2;
  d2 = d1;
  
}
