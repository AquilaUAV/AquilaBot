#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <Servo.h>

// subscribers, publishers, input buffer size, output buffer size
ros::NodeHandle_<ArduinoHardware, 5, 5, 128, 128> nh;

// ----- global parameters -----

#define BAUD 500000
#define DELAY 10

// ----- local parameters  -----

#define pin_button 8

#define pin_buzzer 9

#define pin_line_sensor_L 2
#define pin_line_sensor_R 3

#define pin_encoder_L 2
#define pin_encoder_R 3
#define pin_interrupt_encoder_L 0
#define pin_interrupt_encoder_R 1

#define pin_motor_dir_right 4
#define pin_motor_pwm_right 5
#define pin_motor_pwm_left 6
#define pin_motor_dir_left 7
#define pwm_limit 255
#define motor_securely_system_timeout 250

#define servo_pin_1 10
#define servo_pin_2 11

// ----- button_sensor -----

std_msgs::Bool button_msg;
ros::Publisher pub_button_msg("/omegabot/sensor/button", &button_msg);

void button_init(){
  pinMode(pin_button, INPUT);
}

void button_spin(){
  button_msg.data = digitalRead(pin_button);
  pub_button_msg.publish(&button_msg);
}

// ----- buzzer_cmd -----

void buzzer_init(){
  pinMode(pin_buzzer, OUTPUT);
}

void buzzer_cb(const std_msgs::Int16 &buzzer_cmd){
  analogWrite(pin_buzzer, buzzer_cmd.data);
}

ros::Subscriber<std_msgs::Int16> sub_buzzer("/omegabot/cmd/buzzer", buzzer_cb);

// ----- line_sensor -----

std_msgs::Int16 line_sensor_msg_L;
std_msgs::Int16 line_sensor_msg_R;
ros::Publisher pub_sensor_msg_L("/omegabot/sensor/line/L", &line_sensor_msg_L);
ros::Publisher pub_sensor_msg_R("/omegabot/sensor/line/R", &line_sensor_msg_R);

void line_sensor_init(){
  pinMode(pin_line_sensor_L, INPUT);
  pinMode(pin_line_sensor_R, INPUT);
}

void line_sensor_spin(){
  line_sensor_msg_L.data = analogRead(pin_line_sensor_L);
  line_sensor_msg_R.data = analogRead(pin_line_sensor_R);
  pub_sensor_msg_L.publish(&line_sensor_msg_L);
  pub_sensor_msg_R.publish(&line_sensor_msg_R);
}

// ----- encoder_sensor -----

std_msgs::UInt32 encoder_msg_L;
std_msgs::UInt32 encoder_msg_R;
ros::Publisher pub_encoder_L("/omegabot/sensor/encoder/L", &encoder_msg_L);
ros::Publisher pub_encoder_R("/omegabot/sensor/encoder/R", &encoder_msg_R);

unsigned long encoder_value[2];

void encoder_L_cb(){
  encoder_value[0]++;
}

void encoder_R_cb(){
  encoder_value[1]++;
}

void encoder_init(){
  pinMode(pin_encoder_L, INPUT_PULLUP);
  pinMode(pin_encoder_R, INPUT_PULLUP);
  attachInterrupt(pin_interrupt_encoder_L, encoder_L_cb, CHANGE);
  attachInterrupt(pin_interrupt_encoder_R, encoder_R_cb, CHANGE);
}

void encoder_spin(){
  encoder_msg_L.data = encoder_value[0];
  encoder_msg_R.data = encoder_value[1];
  pub_encoder_L.publish(&encoder_msg_L);
  pub_encoder_R.publish(&encoder_msg_R);
}

// ----- motor_cmd -----

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

// ----- servo_cmd -----

Servo servo_1;
Servo servo_2;

void servo_init(){
  servo_1.attach(servo_pin_1);
  servo_2.attach(servo_pin_2);
}

void servo_1_cb(const std_msgs::Int16& servo_1_cmd){
  servo_1.write(servo_1_cmd.data);
}

void servo_2_cb(const std_msgs::Int16& servo_2_cmd){
  servo_2.write(servo_2_cmd.data);
}

ros::Subscriber<std_msgs::Int16> sub_servo_1("/omegabot/cmd/servo/1", servo_1_cb);
ros::Subscriber<std_msgs::Int16> sub_servo_2("/omegabot/cmd/servo/2", servo_2_cb);

// ----- main -----

void setup() {
  button_init();
  buzzer_init();
  line_sensor_init();
  encoder_init();
  motor_init();
  servo_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  
  nh.advertise(pub_button_msg);
  nh.subscribe(sub_buzzer);
  nh.advertise(pub_sensor_msg_L);
  nh.advertise(pub_sensor_msg_R);
  nh.advertise(pub_encoder_L);
  nh.advertise(pub_encoder_R);
  nh.subscribe(sub_motor_left);
  nh.subscribe(sub_motor_right);
  nh.subscribe(sub_servo_1);
  nh.subscribe(sub_servo_2);
}

void loop() {
  button_spin();
  line_sensor_spin();
  encoder_spin();
  motor_securely_system_update();
  nh.spinOnce();
  delay(DELAY);
}
