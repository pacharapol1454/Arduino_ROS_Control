/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

//TODO PID

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define MOTOR_L_Pin 6
#define DIR1_L_Pin 7
#define DIR2_L_Pin 8
#define MOTOR_R_Pin 9
#define DIR1_R_Pin 10
#define DIR2_R_Pin 11
#define ENCODER_L_Pin A0
#define ENCODER_R_Pin A1

//GLOBAL VARIABLE DECLARATION
float linear_vel_x = 0.0;
float angular_vel_z = 0.0;
float wheel_l = 0.0;
float wheel_r = 0.0;
int direction_l = 0.0;
int direction_r = 0.0;
float encoder_l = 0.0;
float encoder_r = 0.0;
//PID VARIABLES
float kp=1; 
float ki=0.01; 
float kd=1;
//setPoint is wheel_l and wheel_r
unsigned long currentTime, previousTime, elapsedTime;
float error, error_p;
float inVal_L, inVal_R, outVal, setPoint;
float cumError = 0;
float rateError = 0;
float PIDOut_L = 0;
float PIDOut_R = 0;
//END
//PID STRUCT
struct PIDValues{
  float input = 0;
  float setPoint = 0;
  float error = 0;
  float error_p = 0;
  float cumError = 0;
  float rateError = 0;
  float output = 0;
};
PIDValues left_wheel;
PIDValues right_wheel;
PIDValues left_steer;
PIDValues right_steer;
float computePID(struct PIDValues wheel);
//END

char buff[30];
int sprintfOut;
String test_msg = "";

ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::Float64 float_msg;

ros::Publisher wheel_pub("wheel_vel", &str_msg);


void set_speed(float Vx, float W ){
 direction_l = 1;
 direction_r = 1;
 float _Wl = ((Vx - (2.22169002*W))/1.08)*100.0;
 float _Wr = (((2.22169002*W) + Vx)/1.08)*100.0;
 if(_Wl < 0){
  direction_l = 0;
  _Wl = -_Wl;
  }
 if(_Wr <0){
  direction_r = 0;
  _Wr = -_Wr;
 }
 wheel_l = _Wl;
 wheel_r = _Wr;
 analogWrite(MOTOR_L_Pin, _Wl);
 digitalWrite(DIR1_L_Pin, direction_l);
 digitalWrite(DIR2_L_Pin, !direction_l);
 analogWrite(MOTOR_R_Pin, _Wr);
 digitalWrite(DIR1_R_Pin, direction_r);
 digitalWrite(DIR2_R_Pin, !direction_r);
}

void cmd_velCb( const geometry_msgs::Twist& toggle_msg){
  linear_vel_x = toggle_msg.linear.x;
  angular_vel_z = toggle_msg.angular.z;
  set_speed(linear_vel_x, angular_vel_z);
  test_msg = String(wheel_l)+","+String(wheel_r)+","+String(direction_l)+","+String(direction_r);
  test_msg.toCharArray(buff, 30);
  str_msg.data = buff;
  wheel_pub.publish( &str_msg );
}
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_velCb );
void getTime(){
  currentTime = millis();
  elapsedTime = (float)(currentTime-elapsedTime);
  previousTime = currentTime;
}
float computePID(struct PIDValues wheel){  //float _input, float _setPoint
  //currentTime = millis();
  //elapsedTime = (float)(currentTime-elapsedTime);
  wheel.error = wheel.setPoint - wheel.input;
  wheel.cumError += wheel.error * elapsedTime;
  wheel.rateError += (wheel.error - wheel.error_p)/elapsedTime;
  wheel.output = kp*wheel.error + ki*wheel.cumError + kd*wheel.rateError;
  wheel.error_p = wheel.error;
  //previousTime = currentTime;
  return wheel.output;
}

void setup()
{
  
  nh.initNode();
  nh.advertise(wheel_pub);
  nh.subscribe(cmd_sub);

  pinMode(MOTOR_L_Pin, OUTPUT);
  pinMode(MOTOR_R_Pin, OUTPUT);
  pinMode(DIR1_L_Pin, OUTPUT);
  pinMode(DIR2_L_Pin, OUTPUT);
  pinMode(DIR1_R_Pin, OUTPUT);
  pinMode(DIR2_R_Pin, OUTPUT);
  pinMode(ENCODER_L_Pin, INPUT);
  pinMode(ENCODER_R_Pin, INPUT);
  //PID SETUP
  
}

void loop()
{
  encoder_l = analogRead(ENCODER_L_Pin);
  encoder_r = analogRead(ENCODER_R_Pin);
  getTime();
  //LEFT WHEEL
  left_wheel.input = encoder_l;
  left_wheel.setPoint = wheel_l;
  //RIGHT WHEEL
  right_wheel.input = encoder_r;
  right_wheel.setPoint = wheel_r;
  PIDOut_L = computePID(left_wheel);
  PIDOut_R = computePID(right_wheel);
  nh.spinOnce();
}
