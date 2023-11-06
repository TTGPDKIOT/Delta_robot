#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1_bc.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

#define LOOPTIME 100

Motor right(2,3,20,21);
Motor left(5,4,18,19);

double left_kp = 0.42, left_ki = 0, left_kd = 0.0;             // modify for optimal performance
double right_kp = 0.42, right_ki = 0, right_kd = 0.0;


double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float left_wheel_tick_diff;
float right_wheel_tick_diff;

float left_wheel_tick_count_prev;
float right_wheel_tick_count_prev;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

const float encoder_minimum = -1.7e+308;
const float encoder_maximum = 1.7e+308;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

// Keep track of the number of wheel ticks
std_msgs::Float64 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Float64 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Float64 leftpid;
ros::Publisher left_pid("left_pid", &leftpid);  
std_msgs::Float64 rightpid;
ros::Publisher right_pid("right_pid", &rightpid);

double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(left_pid);
  nh.advertise(right_pid);
  nh.advertise(speed_pub);
    
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-255, 255);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-255, 255);

   pinMode(18 , INPUT_PULLUP);
   pinMode(19 , INPUT);
   pinMode(20 , INPUT_PULLUP);
   pinMode(21 , INPUT);
  
   attachInterrupt(digitalPinToInterrupt(18), left_wheel_tick, RISING);
   attachInterrupt(digitalPinToInterrupt(20), right_wheel_tick, RISING);
}

void loop() {
  
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;

    rightPub.publish(&right_wheel_tick_count);
    leftPub.publish(&left_wheel_tick_count);
  
    demand_speed_left = demandx - (demandz*0.1425);
    demand_speed_right = demandx + (demandz*0.1425);
    left_wheel_tick_diff = left_wheel_tick_count.data - left_wheel_tick_count_prev; // Get difference between ticks to compute speed
    right_wheel_tick_diff = right_wheel_tick_count.data - right_wheel_tick_count_prev;
    
    speed_act_left = left_wheel_tick_diff/216.5;                    
    speed_act_right = right_wheel_tick_diff/216.5;
    
    speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
    speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
    speed_pub.publish(&speed_msg); 
  
    left_wheel_tick_count_prev = left_wheel_tick_count.data; // Saving values
    right_wheel_tick_count_prev = right_wheel_tick_count.data;
  
    left_setpoint = demand_speed_left*216.5;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*216.5;
  
    left_input = left_wheel_tick_diff;  //Input to PID controller is the current difference
    right_input = right_wheel_tick_diff;
    
    leftPID.Compute();
    if (left_output > 20 && left_output < 130){
      left_output = 130; 
    }
    else if (left_output > -130 && left_output < -20){
      left_output = -130;
    }
//    else if (left_output >= -30 && left_output <= 30){
//      left_output = 0;
//    }
    left.rotate(left_output);

    leftpid.data = left_output;
    left_pid.publish(&leftpid);
    
    rightPID.Compute();
    if (right_output > 20 && right_output < 140){
      right_output = 140; 
    }
    else if (right_output > -140 && right_output < -20){
      right_output = -140;
    }
//    else if (right_output >= -30 && right_output <= 30){
//      right_output = 0;
//    }
    right.rotate(right_output);

    rightpid.data = right_output;
    right_pid.publish(&rightpid);

    
  }
  nh.spinOnce();
}


// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(21);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(19);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
