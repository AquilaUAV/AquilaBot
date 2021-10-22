#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Servo.h>

ros::NodeHandle nh;

#define pin_encoder_1 2
#define pin_encoder_2 3
#define pin_analog_1 0
#define pin_analog_2 1
#define pin_motor_dir_R 4
#define pin_motor_pwm_R 5
#define pin_motor_pwm_L 6
#define pin_motor_dir_L 7
#define pwm_limit 255
#define pin_servo_1 8
#define pin_servo_2 9
#define pin_servo_3 10
#define pin_servo_4 11

#define DELAY 0

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;

std_msgs::Int16MultiArray encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);
std_msgs::Int16MultiArray analog_read_msg;
ros::Publisher pub_analog_read("analog_read", &analog_read_msg);

int encoder_value[2];

void encoder_1_cb(){
  encoder_value[0]++;
}

void encoder_2_cb(){
  encoder_value[1]++;
}

void encoder_init(){
  encoder_msg.data_length = 2;
  encoder_msg.data = (int*)malloc(sizeof(int) * 2);
  pinMode(pin_encoder_1, INPUT_PULLUP);
  pinMode(pin_encoder_2, INPUT_PULLUP);
  attachInterrupt(0, encoder_1_cb, CHANGE);
  attachInterrupt(1, encoder_2_cb, CHANGE);
}

void encoder_spin(){
  encoder_msg.data[0] = encoder_value[0];
  encoder_msg.data[1] = encoder_value[1];
  pub_encoder.publish(&encoder_msg);
}

void analog_init(){
  pinMode(pin_analog_1, OUTPUT);
  pinMode(pin_analog_2, OUTPUT);
  analog_read_msg.data_length = 2;
  analog_read_msg.data = (int*)malloc(sizeof(int) * 2);
}

void analog_spin(){
  int analog_value_1 = analogRead(pin_analog_1);
  int analog_value_2 = analogRead(pin_analog_2);
  analog_read_msg.data[0] = analog_value_1;
  analog_read_msg.data[1] = analog_value_2;
  pub_analog_read.publish(&analog_read_msg);
}

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

void servo_init(){
  servo_1.attach(pin_servo_1);
  servo_2.attach(pin_servo_2);
  servo_3.attach(pin_servo_3);
  servo_4.attach(pin_servo_4);
}

void servo_cb(const std_msgs::Int16MultiArray& cmd_array_msg){
  int servo_1_cmd = cmd_array_msg.data[0];
  int servo_2_cmd = cmd_array_msg.data[1];
  int servo_3_cmd = cmd_array_msg.data[2];
  int servo_4_cmd = cmd_array_msg.data[3];
  servo_1.write(servo_1_cmd);
  servo_2.write(servo_2_cmd);
  servo_3.write(servo_3_cmd);
  servo_4.write(servo_4_cmd);
}
ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo_cmd", servo_cb);


void setup() {
  encoder_init();
  analog_init();
  motor_init();
  servo_init();
  
  nh.initNode();
  nh.advertise(pub_encoder);
  nh.advertise(pub_analog_read);
  nh.subscribe(motor_sub);
  nh.subscribe(servo_sub);
}

void loop() {
  encoder_spin();
  analog_spin();
  nh.spinOnce();
  delay(DELAY);
}
