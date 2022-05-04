#include<TMCStepper.h>
#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"
#include<SoftwareSerial.h>


SoftwareSerial mySerial(2, 3); 

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist


double speed_x;
double speed_y;
double speed_rot;

double linear_vel_x_mins_;
double linear_vel_y_mins_;
double angular_vel_z_mins_;
double tangential_vel_;

double x_rpm_;
double y_rpm_;
double tan_rpm_;

double circumference_ = 0.314;



double speed_front_left;
double speed_front_right;
double speed_back_left;
double speed_back_right;

double wheel_radius = 0.05;
double wheel_seperation_width = 0.37;
double wheel_seperation_length = 0.32;
double front_left;double front_right;
double back_left;double back_right;

double wheel_geometry = (wheel_seperation_width + wheel_seperation_length) / 2;

#define FR_EN_PIN           38 // Enable
#define FR_DIR_PIN          A1 // Direction
#define FR_STEP_PIN         A0 // Step
#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define FR_SERIAL_PORT Serial1 // Arduino mega Hw serial pins Tx=18 and Rx=19 


#define FL_EN_PIN           A2 // Enable
#define FL_DIR_PIN          A7 // Direction
#define FL_STEP_PIN         A6 // Step
//#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define FL_SERIAL_PORT Serial2 // Arduino mega Hw serial pins Tx=18 and Rx=19

#define BR_EN_PIN           A8 // Enable
#define BR_DIR_PIN          48 // Direction
#define BR_STEP_PIN         46 // Step
//#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define BR_SERIAL_PORT Serial3 // Arduino mega Hw serial pins Tx=18 and Rx=19 


#define BL_EN_PIN           24 // Enable
#define BL_DIR_PIN          28 // Direction
#define BL_STEP_PIN         26 // Step
#define SW_RX            D3 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            D2 // TMC2208/TMC2224 SoftwareSerial transmit pin
//#define BL_SERIAL_PORT Serial2 // Arduino mega Hw serial pins Tx=18 and Rx=19

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

TMC2209Stepper FR_driver(&FR_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper FL_driver(&FL_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper BR_driver(&BR_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper BL_driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

constexpr uint32_t steps_per_m = 1911; // steps needed for robot to move 1m

AccelStepper FR_stepper = AccelStepper(FR_stepper.DRIVER, FR_STEP_PIN, FR_DIR_PIN);
AccelStepper FL_stepper = AccelStepper(FL_stepper.DRIVER, FL_STEP_PIN, FL_DIR_PIN);
AccelStepper BR_stepper = AccelStepper(BR_stepper.DRIVER, BR_STEP_PIN, BR_DIR_PIN);
AccelStepper BL_stepper = AccelStepper(BL_stepper.DRIVER, BL_STEP_PIN, BL_DIR_PIN);


void messageCb(const geometry_msgs::Twist& msg)  // cmd_vel callback function definition
 {

  
  speed_x = max(min(msg.linear.x, 1.0f), -1.0f);   
  speed_y = max(min(msg.linear.y, 1.0f), -1.0f); 
  speed_rot = max(min(msg.angular.z, 1.0f), -1.0f); 

    front_left = (speed_x - speed_y - speed_rot * wheel_geometry) / wheel_radius;
    front_right = (speed_x + speed_y + speed_rot * wheel_geometry) / wheel_radius;
    back_left = (speed_x + speed_y - speed_rot * wheel_geometry) / wheel_radius;
    back_right = (speed_x - speed_y + speed_rot * wheel_geometry) / wheel_radius;


if((front_left) && (front_right) && (back_left) && (back_right) == 0)
 {

  
FR_stepper.setSpeed(0);
FL_stepper.setSpeed(0);
BR_stepper.setSpeed(0);
BL_stepper.setSpeed(0);
FR_stepper.runSpeed();
FL_stepper.runSpeed();
BR_stepper.runSpeed();
BL_stepper.runSpeed();


  }

  else {
  speed_front_left = mapFloat(fabs(front_left), 0.0, 20.0, 100,1000);
  speed_front_right = mapFloat(fabs(front_right), 0.0, 20.0, 100,3000);
  speed_back_right= mapFloat(fabs(back_right), 0.0, 20.0, 100,3000);
  speed_back_left = mapFloat(fabs(back_left), 0.0, 20.0, 100,3000);
  }

   if(((front_left) && (back_left) > 0) && ((back_right) && (front_right) < 0))
  {

  FR_stepper.setSpeed(-speed_front_right);
  FL_stepper.setSpeed(speed_front_left);
  BR_stepper.setSpeed(-speed_back_right);
  BL_stepper.setSpeed(speed_back_left);
  }
// perpendicular left
  else if(((front_left) && (back_right) < 0) && ((front_right) && (back_left) > 0))
  {
   
  FR_stepper.setSpeed(speed_front_right);
  FL_stepper.setSpeed(-speed_front_left);
  BR_stepper.setSpeed(speed_back_right);
  BL_stepper.setSpeed(-speed_back_left);
  }


  else if(((front_left) && (back_left) < 0) && ((back_right) && (front_right) > 0))
  {
   
  FR_stepper.setSpeed(speed_front_right);
  FL_stepper.setSpeed(-speed_front_left);
  BR_stepper.setSpeed(speed_back_right);
  BL_stepper.setSpeed(-speed_back_left);
  }

else if((front_left) && (front_right) && (back_left) && (back_right) < 0)
  {

  FR_stepper.setSpeed(-speed_front_right);
  FL_stepper.setSpeed(-speed_front_left);
  BR_stepper.setSpeed(-speed_back_right);
  BL_stepper.setSpeed(-speed_back_left);
  }
  // perpendicular left
  else if(((front_left) && (back_right) > 0) && ((front_right) && (back_left) < 0))
  {
    
  FR_stepper.setSpeed(speed_front_right);
  FL_stepper.setSpeed(-speed_front_left);
  BR_stepper.setSpeed(speed_back_right);
  BL_stepper.setSpeed(-speed_back_left);
  }
  
else if((front_left) && (front_right) && (back_left) && (back_right) > 0) 
 {
  
  FR_stepper.setSpeed(speed_front_right);
  FL_stepper.setSpeed(speed_front_left);
  BR_stepper.setSpeed(speed_back_right);
  BL_stepper.setSpeed(speed_back_left);
 }
 
  
else
{
 
  FR_stepper.setSpeed(0);
  FL_stepper.setSpeed(0);
  BR_stepper.setSpeed(0);
  BL_stepper.setSpeed(0);
  }
 }



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  

void setup() {
     
   nh.initNode();     
   nh.subscribe(sub);  
// FRONT RIGHT STEPPER SETUP
     FR_SERIAL_PORT.begin(115200);      
    //driver.begin();             
    FR_driver.toff(5);                 
    FR_driver.rms_current(1200);    
    FR_driver.en_spreadCycle(true);   
    //driver.pwm_autoscale(1);
    FR_driver.intpol(true); 
    FR_driver.microsteps(1); 
    FR_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
    FR_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
    FR_stepper.setEnablePin(FR_EN_PIN);
    FR_stepper.setPinsInverted(false, false, true);
    FR_stepper.enableOutputs();
    FR_stepper.setSpeed(0);

    
// FRONT LEFT STEPPER SETUP

  FL_SERIAL_PORT.begin(115200);      // HW UART drivers
  //FL_driver.begin();             // Initiate pins and registeries
  FL_driver.toff(5);                 // Enables driver in software
  FL_driver.rms_current(1200);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  FL_driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
  FL_driver.pwm_autoscale(1);
  FL_driver.intpol(true); //1/256 microstep interpolation
  FL_driver.microsteps(1); //Microsteps. 1 = full step
    
  FL_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
  FL_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
  FL_stepper.setEnablePin(FL_EN_PIN);
  FL_stepper.setPinsInverted(true, false, true);
  FL_stepper.enableOutputs();
  FL_stepper.setSpeed(0);

  // BACK RIGHT STEPPER SETUP
  BR_SERIAL_PORT.begin(115200);      
  //driver.begin();             
  BR_driver.toff(5);                 
  BR_driver.rms_current(1200);    
  BR_driver.en_spreadCycle(true);   
  //driver.pwm_autoscale(1);
  BR_driver.intpol(true); 
  BR_driver.microsteps(1); 
  BR_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
  BR_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
  BR_stepper.setEnablePin(BR_EN_PIN);
  BR_stepper.setPinsInverted(false, false, true);
  BR_stepper.enableOutputs();
  BR_stepper.setSpeed(0);

    
// BACK LEFT STEPPER SETUP

  //BL_SERIAL_PORT.begin(115200);      // HW UART drivers
  BL_driver.begin();             // Initiate pins and registeries
  BL_driver.toff(5);                 // Enables driver in software
  BL_driver.rms_current(1200);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  BL_driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
  BL_driver.pwm_autoscale(1);
  BL_driver.intpol(true); //1/256 microstep interpolation
  BL_driver.microsteps(1); //Microsteps. 1 = full step
    
  BL_stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
  BL_stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
  BL_stepper.setEnablePin(BL_EN_PIN);
  BL_stepper.setPinsInverted(true, false, true);
  BL_stepper.enableOutputs();
  BL_stepper.setSpeed(0);

    
}

void loop() {
      
    nh.spinOnce();
    FR_stepper.runSpeed();
    FL_stepper.runSpeed();
    BR_stepper.runSpeed();
    BL_stepper.runSpeed();
}
