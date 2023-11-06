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

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 5 , left_ki = 0 , left_kd = 0;             // modify for optimal performance
double right_kp = 5 , right_ki = 0 , right_kd = 0;


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

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

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
    
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);

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
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    speed_act_left = encoder0Diff/21.65;                    
    speed_act_right = encoder1Diff/21.65; 
  
    encoder0Error = (demand_speed_left*21.65)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*21.65)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*21.65;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*21.65;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
 
    leftpid.data = left_output;
    left_pid.publish(&leftpid);
    
    rightPID.Compute();
    right.rotate(right_output);

    rightpid.data = right_output;
    right_pid.publish(&rightpid);
    
    nh.spinOnce();
  }
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
