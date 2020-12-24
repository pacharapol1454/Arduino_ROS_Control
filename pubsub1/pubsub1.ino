/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

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

//GLOBAL VARIABLE DECLARATION
float linear_vel_x = 0.0;
float angular_vel_z = 0.0;
float wheel_l = 0.0;
float wheel_r = 0.0;
int direction_l = 0.0;
int direction_r = 0.0;

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


void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(wheel_pub);
  nh.subscribe(cmd_sub);

  pinMode(MOTOR_L_Pin, OUTPUT);
  pinMode(MOTOR_R_Pin, OUTPUT);
  pinMode(DIR1_L_Pin, OUTPUT);
  pinMode(DIR2_L_Pin, OUTPUT);
  pinMode(DIR1_R_Pin, OUTPUT);
  pinMode(DIR2_R_Pin, OUTPUT);
  
}

void loop()
{
    nh.spinOnce();
}
