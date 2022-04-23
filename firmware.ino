#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"
//#include "A4988.h"

#include <AccelStepper.h>
#define MOTOR_STEPS 200

////Front Right
//#define STEP1 3
//#define front_right_DIR 4
//
////Front Left
//#define STEP2 5
//#define front_left_DIR 6
//
////Rear Right
//#define STEP3 7
//#define rear_right_DIR 8
//
//
////Rear Left
//#define STEP4 9
//#define rear_rightDIR 10
//
//A4988 front_right_motor(MOTOR_STEPS, front_right_DIR, STEP1);
//A4988 front_left_motor(MOTOR_STEPS, front_left_DIR, STEP2);
//A4988 rear_right_motor(MOTOR_STEPS, rear_right_DIR, STEP3);
//A4988 rear_left_motor(MOTOR_STEPS, rear_right_DIR, STEP4);

AccelStepper LeftBackWheel(1, 42, 43);   // (Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 40, 41);  // Stepper2
AccelStepper RightBackWheel(1, 44, 45);  // Stepper3
AccelStepper RightFrontWheel(1, 46, 47); // Stepper4

int wheelSpeed = 1500;

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist

//std_msgs::Int32 value;  
 

//ros::Publisher motor_value("motor_value", &value);


double speed_x;
double speed_y;
double speed_rot;

double speed_front_left;
double speed_front_right;
double speed_back_left;
double speed_back_right;


double wheel_radius = 0.05;
double wheel_seperation_width = 0.37;
double wheel_seperation_length = 0.32;
double front_left=0,front_right=0;
double back_left=0,back_right=0;


double wheel_geometry = (wheel_seperation_width + wheel_seperation_length) / 2;



void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
 {
  speed_x = max(min(msg.linear.x, 1.0f), -1.0f);   // limits the linear x value from -1 to 1
  speed_y = max(min(msg.linear.y, 1.0f), -1.0f);  // limits the angular z value from -1 to 1
  speed_rot = max(min(msg.angular.z, 1.0f), -1.0f);  // limits the angular z value from -1 to 1

    front_left = (speed_x - speed_y - speed_rot * wheel_geometry) / wheel_radius;
    front_right = (speed_x + speed_y + speed_rot * wheel_geometry) / wheel_radius;
    back_left = (speed_x + speed_y - speed_rot * wheel_geometry) / wheel_radius;
    back_right = (speed_x - speed_y + speed_rot * wheel_geometry) / wheel_radius;
    
 if((front_left) && (front_right) && (back_left) && (back_right) == 0)
 {
stopMoving();
 
  }

  else {
  speed_front_left = mapFloat(fabs(front_left), 0.0, 20.0, 100,3000);
  speed_front_right = mapFloat(fabs(front_right), 0.0, 20.0, 100,3000);
  speed_back_right= mapFloat(fabs(back_right), 0.0, 20.0, 100,3000);
  speed_back_left = mapFloat(fabs(back_left), 0.0, 20.0, 100,3000);
  }

if((front_left) && (front_right) && (back_left) && (back_right) > 0) 
 
  moveForward();
   
 else if((front_left) && (front_right) && (back_left) && (back_right) > 0)
  
  moveBackward();
  
  else if(((front_left) && (back_left) > 0) && ((back_left) && (front_left) < 0))
  
  rotateRight();
  
  else if(((front_left) && (back_left) < 0) && ((back_left) && (front_left) > 0))
  
  rotateLeft();


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
//    nh.advertise(motor_value);

  }

void loop() {
//  value.data = back_right;
//  motor_value.publish(&value);
  nh.spinOnce();
 
}


void moveForward() {
  LeftFrontWheel.setSpeed(speed_front_left);
  LeftBackWheel.setSpeed(speed_back_left);
  RightFrontWheel.setSpeed(speed_front_right);
  RightBackWheel.setSpeed(speed_back_right);
}
void moveBackward() {
  LeftFrontWheel.setSpeed(-speed_front_left);
  LeftBackWheel.setSpeed(-speed_back_left);
  RightFrontWheel.setSpeed(-speed_front_right);
  RightBackWheel.setSpeed(-speed_back_right);
}
void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(speed_front_left);
  LeftBackWheel.setSpeed(-speed_back_left);
  RightFrontWheel.setSpeed(-speed_front_right);
  RightBackWheel.setSpeed(speed_back_right);
}
void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(-speed_front_left);
  LeftBackWheel.setSpeed(speed_back_left);
  RightFrontWheel.setSpeed(speed_front_right);
  RightBackWheel.setSpeed(-speed_back_right);
}
void rotateLeft() {
  LeftFrontWheel.setSpeed(-speed_front_left);
  LeftBackWheel.setSpeed(-speed_back_left);
  RightFrontWheel.setSpeed(speed_front_right);
  RightBackWheel.setSpeed(speed_back_right);
}
void rotateRight() {
  LeftFrontWheel.setSpeed(speed_front_left);
  LeftBackWheel.setSpeed(speed_back_left);
  RightFrontWheel.setSpeed(-speed_front_right);
  RightBackWheel.setSpeed(-speed_back_right);
}
void moveRightForward() {
  LeftFrontWheel.setSpeed(speed_front_left);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(speed_back_right);
}
void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-speed_back_left);
  RightFrontWheel.setSpeed(-speed_front_right);
  RightBackWheel.setSpeed(0);
}
void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(speed_back_left);
  RightFrontWheel.setSpeed(speed_front_right);
  RightBackWheel.setSpeed(0);
}
void moveLeftBackward() {
  LeftFrontWheel.setSpeed(-speed_front_left);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-speed_back_right);
}
void stopMoving() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
}
