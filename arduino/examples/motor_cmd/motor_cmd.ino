#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

#define pin_motor_dir_right 4
#define pin_motor_pwm_right 5
#define pin_motor_pwm_left 6
#define pin_motor_dir_left 7
#define pwm_limit 255
#define motor_securely_system_timeout 250
#define BAUD 115200
#define DELAY 10

unsigned long motor_update_time_last = 0;

void motor_init(){
  pinMode(pin_motor_dir_right, OUTPUT);
  pinMode(pin_motor_dir_left, OUTPUT);
  pinMode(pin_motor_pwm_right, OUTPUT);
  pinMode(pin_motor_pwm_left, OUTPUT);
}

void motor_cb(int motor_cmd, int motor_dir_pin, int motor_voltage_pin){
  if (motor_cmd > pwm_limit){
    motor_cmd = pwm_limit;
  }
  else if (motor_cmd < -pwm_limit){
    motor_cmd = -pwm_limit;
  }
  if (motor_cmd >= 0){
    digitalWrite(motor_dir_pin, HIGH);
  } else {
    digitalWrite(motor_dir_pin, LOW);
  }
  analogWrite(motor_voltage_pin, abs(motor_cmd));
}

void motor_left_cb(const std_msgs::Int16 &motor_left_cmd){
  motor_update_time_last = millis();
  motor_cb(motor_left_cmd.data, pin_motor_dir_left, pin_motor_pwm_left);
}

void motor_right_cb(const std_msgs::Int16 &motor_right_cmd){
  motor_update_time_last = millis();
  motor_cb(motor_right_cmd.data, pin_motor_dir_right, pin_motor_pwm_right);
}

ros::Subscriber<std_msgs::Int16> sub_motor_left("/omegabot/cmd/motor/left", motor_left_cb);
ros::Subscriber<std_msgs::Int16> sub_motor_right("/omegabot/cmd/motor/right", motor_right_cb);

void motor_securely_system_update(){
  unsigned long motor_update_time = millis();
  if (motor_update_time - motor_update_time_last > motor_securely_system_timeout){
    motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
    motor_update_time_last = motor_update_time;
  }
}

void setup() {
  motor_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub_motor_left);
  nh.subscribe(sub_motor_right);
}

void loop() {
  motor_securely_system_update();
  nh.spinOnce();
  delay(DELAY);
}
