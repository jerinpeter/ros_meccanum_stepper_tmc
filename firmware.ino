#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"
//#include "A4988.h"

#include <AccelStepper.h>
#define MOTOR_STEPS 200

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

AccelStepper LeftBackWheel(1, 42, 43);   // (Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 40, 41);  // Stepper2
AccelStepper RightBackWheel(1, 44, 45);  // Stepper3
AccelStepper RightFrontWheel(1, 46, 47); // Stepper4

int wheelSpeed = 1500;

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist

std_msgs::Int32 value;  
 
double rpm_motor_fl;
double rpm_motor_fr;
double rpm_motor_br;
double rpm_motor_bl;

ros::Publisher motor_value("motor_value", &value);
double circumference_ = 0.314;
double tangential_vel_;
double speed_x;
double speed_y;
double speed_rot;

//double speed_front_left;
//double speed_front_right;
//double speed_back_left;
//double speed_back_right;

double linear_vel_x_mins_;
double linear_vel_y_mins_;
double angular_vel_z_mins_;

double x_rpm_;
double y_rpm_;
double tan_rpm_;




double wheel_radius = 0.05;
double wheel_seperation_width = 0.37;
double wheel_seperation_length = 0.32;
double front_left=0,front_right=0;
double back_left=0,back_right=0;


double wheel_geometry = (wheel_seperation_width + wheel_seperation_length) / 2;



void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
 {
  speed_x = max(min(msg.linear.x, 1.0f), -1.0f);   
  speed_y = max(min(msg.linear.y, 1.0f), -1.0f); 
  speed_rot = max(min(msg.angular.z, 1.0f), -1.0f); 

  linear_vel_x_mins_ = speed_x * 60;
  linear_vel_y_mins_ = speed_y * 60;
  angular_vel_z_mins_ = speed_rot * 60;
  
  tangential_vel_ = angular_vel_z_mins_ * 0.37;  // 0.37 -> base width

   x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

   rpm_motor_fl = x_rpm_ - y_rpm_ - tan_rpm_;
  //rear-left motor
  rpm_motor_bl = x_rpm_ + y_rpm_ - tan_rpm_;

  //front-right motor
  rpm_motor_fr = x_rpm_ + y_rpm_ + tan_rpm_;
  //rear-right motor
  rpm_motor_br = x_rpm_ - y_rpm_ + tan_rpm_;

//    front_left = (speed_x - speed_y - speed_rot * wheel_geometry) / wheel_radius;
//    front_right = (speed_x + speed_y + speed_rot * wheel_geometry) / wheel_radius;
//    back_left = (speed_x + speed_y - speed_rot * wheel_geometry) / wheel_radius;
//    back_right = (speed_x - speed_y + speed_rot * wheel_geometry) / wheel_radius;
//    
// if((front_left) && (front_right) && (back_left) && (back_right) == 0)
// {
//stopMoving();
// 
//  }
//
//  else {
//  speed_front_left = mapFloat(fabs(front_left), 0.0, 20.0, 100,3000);
//  speed_front_right = mapFloat(fabs(front_right), 0.0, 20.0, 100,3000);
//  speed_back_right= mapFloat(fabs(back_right), 0.0, 20.0, 100,3000);
//  speed_back_left = mapFloat(fabs(back_left), 0.0, 20.0, 100,3000);
//  }
//
//if((front_left) && (front_right) && (back_left) && (back_right) > 0) 
// 
//  moveForward();
//   
// else if((front_left) && (front_right) && (back_left) && (back_right) > 0)
//  
//  moveBackward();
//  
//  else if(((front_left) && (back_left) > 0) && ((back_left) && (front_left) < 0))
//  
//  rotateRight();
//  
//  else if(((front_left) && (back_left) < 0) && ((back_left) && (front_left) > 0))
//  
//  rotateLeft();

  }


  
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  // creation of subscriber object sub for recieving the cmd_vel



void setup() 
  {
  LeftFrontWheel.setMaxSpeed(3000);
  LeftBackWheel.setMaxSpeed(3000);
  RightFrontWheel.setMaxSpeed(3000);
  RightBackWheel.setMaxSpeed(3000);

     nh.initNode();      // initialzing the node handle object
     nh.subscribe(sub);  // subscribing to cmd vel with sub object
   nh.advertise(motor_value);

  }

void loop() {
  value.data = rpm_motor_fl;
  motor_value.publish(&value);
  nh.spinOnce();
 
}
