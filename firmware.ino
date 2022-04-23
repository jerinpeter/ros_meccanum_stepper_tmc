#include<TMCStepper.h>
#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "MapFloat.h"

#include <SoftwareSerial.h>

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

double rpm_motor_fl;
double rpm_motor_fr;
double rpm_motor_br;
double rpm_motor_bl;


#define EN_PIN           A8 // Enable
#define DIR_PIN          48 // Direction
#define STEP_PIN         46 // Step
//#define SW_RX            A11 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            D42 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // Arduino mega Hw serial pins Tx=18 and Rx=19 
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
constexpr uint32_t steps_per_m = 1911; // steps needed for robot to move 1m

AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);


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

  rpm_motor_fl = x_rpm_ - y_rpm_ - tan_rpm_;    //front-left motor
   
  rpm_motor_bl = x_rpm_ + y_rpm_ - tan_rpm_;     //back-left motor

  rpm_motor_fr = x_rpm_ + y_rpm_ + tan_rpm_;     //front-right motor
  
  rpm_motor_br = x_rpm_ - y_rpm_ + tan_rpm_;     //back-right motor
  

if((rpm_motor_fl) && (rpm_motor_bl) && (rpm_motor_fr) && (rpm_motor_br) > 0)
{

      // insert all motors forward code here + multiply rpm with 10
}
  
  
  else if((rpm_motor_fl) && (rpm_motor_bl) && (rpm_motor_fr) && (rpm_motor_br) < 0)
{
 
      // insert all motors backward code here + multiply rpm with 10
}


 }


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",&messageCb);  

void setup() {
     
     nh.initNode();      // initialzing the node handle object
     nh.subscribe(sub);  // subscribing to cmd vel with sub object
    
    SERIAL_PORT.begin(115200);      // HW UART drivers
    //driver.begin();             // Initiate pins and registeries
    driver.toff(5);                 // Enables driver in software
    driver.rms_current(900);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driver.en_spreadCycle(true);   // Toggle spreadCycle on TMC2208/2209/2224
    //driver.pwm_autoscale(1);
    driver.intpol(true); //1/256 microstep interpolation
    driver.microsteps(1); //Microsteps. 1 = full step
    
    stepper.setMaxSpeed(0.5*steps_per_m); //0.5m/s
    stepper.setAcceleration(0.1*steps_per_m); // 0.1m/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
    
}

void loop() {
    nh.spinOnce();
//    stepper.setSpeed(0.5*steps_per_m);
//    stepper.runSpeed();
}
