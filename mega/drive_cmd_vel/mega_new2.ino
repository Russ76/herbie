// Arduino Mega and ROS(1)
// Works with AI Python script for Oak-D weed sprayer machine "Herbie"
// Encoder messages utilized for speed sensing, 3 speed ranges.
// AI script selects from 3 camera views according to machine speed
// to allow for more precise spray control. 
// Spray wand moves in only one dimension (side to side), as machine moves forward.
// Uses Sabertooth motor controller, 2x12 or 2x25
// Serial2 pin is #16
// Motor scale for needed input: -0.5 to 0.5
// Will work directly with Joy and teleop_twist_joy messages
// Callback routine multiplies number by 70 for full range
// Encoder messages published
// Adafruit_MPU6050 code added. 
// Red to 5V, Black GRND, Yellow SCL 21, Blue SDA 20
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
//#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define GRAVITY 9.81 // 0.00059855

int pwrLeft = 64;  // start with brakes on
int pwrRight = 192; // this is Sabertooth's method of simplified serial
// Each motor has 7 bits of speed resolution. One wire used for both motors
// Angular boost makes turning stronger, useful for skid steer or tracked units
float angularboost = 1.2;
float linx;
float angZ;

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

int LED1 = 5;  // _slow_
int led2 = 6;  // medium
int led3 = 7;  // _fast_

Encoder encoderRight( 3, 2 ); // Pins on Mega for MOTOR A encoder data (Had to be reversed!)
Encoder encoderLeft( 19, 18 ); // pins on Mega for MOTOR B encoder data

ros::NodeHandle nh;


// Callback

void messageCb( const geometry_msgs::Twist& msg)
{
  linx = msg.linear.x * 80;
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
// ros::Publisher herbie_speed_pub("herbie_speed", &str_msg);
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
//  nh.advertise(herbie_speed_pub);
  nh.advertise(distance_pub);
  
  Serial2.begin(9600);
  
  if (!mpu.begin()) {
    nh.loginfo("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  nh.loginfo("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
   
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  
  pinMode(LED1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  
while (!nh.connected())
{
  nh.spinOnce();
}
  nh.loginfo("MEGA CONNECTED");
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

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  //nh.loginfo("In main loop now.");
	
  // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
	sensor_msgs::Imu imu_msg;
        ros::Time ros_time;
        imu_msg.header.stamp = ros_time;
        imu_msg.header.frame_id = "base_link";
        imu_msg.linear_acceleration.x = (a.acceleration.x) * GRAVITY;
        imu_msg.linear_acceleration.y = (a.acceleration.y) * GRAVITY;
        imu_msg.linear_acceleration.z = (a.acceleration.z) * GRAVITY;
        imu_msg.angular_velocity.x = (g.gyro.x)*3.1415926/180;
        imu_msg.angular_velocity.y = (g.gyro.y)*3.1415926/180;
        imu_msg.angular_velocity.z = (g.gyro.z)*3.1415926/180;
  // https://github.com/Russ76/ros_mpu6050_node-1/blob/master/src/mpu6050_node.cpp
	imu_pub.publish(&imu_msg);
		
  Serial2.write(pwrLeft); // motor speed and stop
  Serial2.write(pwrRight); // Power motors both directions
  
  delay(15);
  
//  nLastCompute = nNow;
  savedLeft  = newLeft ;
  savedRight = newRight;
  d4 = d3;
  d3 = d2;
  d2 = d1;
  
}
