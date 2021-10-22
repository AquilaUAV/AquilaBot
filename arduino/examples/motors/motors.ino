#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

#define pin_motor_dir_R 4
#define pin_motor_pwm_R 5
#define pin_motor_pwm_L 6
#define pin_motor_dir_L 7
#define pwm_limit 255

void motor_init(){
  pinMode(pin_motor_dir_R, OUTPUT);
  pinMode(pin_motor_dir_L, OUTPUT);
  pinMode(pin_motor_pwm_R, OUTPUT);
  pinMode(pin_motor_pwm_L, OUTPUT);
}

void motor_cb(const std_msgs::Int16MultiArray& cmd_array_msg){
  int left_cmd = cmd_array_msg.data[0];
  int right_cmd = cmd_array_msg.data[1];
  if (left_cmd > pwm_limit){
    left_cmd = pwm_limit;
  }
  if (left_cmd < -pwm_limit){
    left_cmd = -pwm_limit;
  }
  if (right_cmd > pwm_limit){
    right_cmd = pwm_limit;
  }
  if (right_cmd < -pwm_limit){
    right_cmd = -pwm_limit;
  }
  if (left_cmd >= 0){
    digitalWrite(pin_motor_dir_L, HIGH);
  } else {
    digitalWrite(pin_motor_dir_L, LOW);
  }
  if (right_cmd >= 0){
    digitalWrite(pin_motor_dir_R, HIGH);
  } else {
    digitalWrite(pin_motor_dir_R, LOW);
  }
  analogWrite(pin_motor_pwm_L, abs(left_cmd));
  analogWrite(pin_motor_pwm_R, abs(right_cmd));
}
ros::Subscriber<std_msgs::Int16MultiArray> motor_sub("motor_cmd", motor_cb);

void setup() {
  motor_init();
  
  nh.initNode();
  nh.subscribe(motor_sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
