#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"
#include "A4988.h"


#define MOTOR_STEPS 200

//Front Right
#define STEP1 3
#define front_right_DIR 4

//Front Left
#define STEP2 5
#define front_left_DIR 6

//Rear Right
#define STEP3 7
#define rear_right_DIR 8


//Rear Left
#define STEP4 9
#define rear_rightDIR 10

A4988 front_right_motor(MOTOR_STEPS, front_right_DIR, STEP1);
A4988 front_left_motor(MOTOR_STEPS, front_left_DIR, STEP2);
A4988 rear_right_motor(MOTOR_STEPS, rear_right_DIR, STEP3);
A4988 rear_left_motor(MOTOR_STEPS, rear_right_DIR, STEP4);



ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist

std_msgs::Int32 value;  
 

ros::Publisher motor_value("motor_value", &value);






double speed_x;
double speed_y;
double speed_rot;

double wheel_radius;
double wheel_seperation_width;
double wheel_seperation_length;
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
    
 
  }
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  // creation of subscriber object sub for recieving the cmd_vel



void setup() 
  {
     nh.initNode();      // initialzing the node handle object
     nh.subscribe(sub);  // subscribing to cmd vel with sub object
    nh.advertise(motor_value);

  }

void loop() {
  value.data = front_left;
  motor_value.publish(&value);
  nh.spinOnce();
 
}
