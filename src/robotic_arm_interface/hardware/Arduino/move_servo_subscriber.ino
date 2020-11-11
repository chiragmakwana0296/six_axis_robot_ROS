/*
 * Servo Joint State Controller 
 * ROS-Kinetic 
 * Author : Chiragkumar Makwana
 */

#include <Servo.h>
#include <ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle nh;

#define VELOCITY 0.1
#define pi  3.14
//double angle_rad[6];

Servo joint0;

sensor_msgs::JointState data;
std_msgs::Float64 state_msg;

//ros::Publisher pub("angle_state_publisher", &state_msg);

void servo_cb(const sensor_msgs::JointState& msg)
{
      float angle_rad[6]={}      ;
      angle_rad[1] = msg.position[1]; 
      state_msg.data=(long)angle_rad[1];
      
      angle_rad[2] = msg.position[2]; 
      state_msg.data=(long)angle_rad[2];
      int angle_degrr = angle_rad[2]*(180/pi) ;
      joint0.write(angle_degrr);
     
      angle_rad[3] = msg.position[3]; 
      state_msg.data=(long)angle_rad[3];
      
      angle_rad[4] = msg.position[4]; 
      state_msg.data=(long)angle_rad[4];
      
      angle_rad[5] = msg.position[5]; 
      state_msg.data=(long)angle_rad[5];
      delay(1);
      
}

ros::Subscriber<sensor_msgs::JointState> sub("JointStates", servo_cb);


void setup()
{
  pinMode(13, OUTPUT);
  joint0.attach(12);
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
 // nh.advertise(pub);
}

void loop()
{
 //state_msg.data=angle_rad;
 //pub.publish( &state_msg);
 nh.spinOnce();
 //delay(50)  ;
}
/*
void MoveServo(int servoNumber, double angleInDegrees)
{
  uint16_t pulselength = (uint16_t)map(angleInDegrees, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(servoNumber, 0, pulselength);
}
*/

double radiansToDegrees(float position_radians)
{
  position_radians = position_radians + 1.6;

  return position_radians * 57.2958;
}


