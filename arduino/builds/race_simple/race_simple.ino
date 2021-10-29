#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <Servo.h>

// subscribers, publishers, input buffer size, output buffer size
ros::NodeHandle_<ArduinoHardware, 2*3, 4*3, 128*3, 128*3> nh;

// ----- global parameters -----

#define BAUD 115200 // max stable for raspi is 115200; 250000 is semi-stable
#define DELAY 20

// ----- local parameters  -----

#define pin_encoder_left 2
#define pin_encoder_right 3
#define pin_interrupt_encoder_left 0
#define pin_interrupt_encoder_right 1

#define pin_motor_dir_right 4
#define pin_motor_pwm_right 5
#define pin_motor_pwm_left 6
#define pin_motor_dir_left 7
#define pwm_limit 255
#define motor_securely_system_timeout 250

#define servo_pin_1 10
#define servo_pin_2 11

// ----- encoder_sensor -----

std_msgs::UInt32 encoder_msg_left;
std_msgs::UInt32 encoder_msg_right;
ros::Publisher pub_encoder_left("/omegabot/sensor/encoder/left", &encoder_msg_left);
ros::Publisher pub_encoder_right("/omegabot/sensor/encoder/right", &encoder_msg_right);

unsigned long encoder_value[2];

void encoder_left_cb(){
  encoder_value[0]++;
}

void encoder_right_cb(){
  encoder_value[1]++;
}

void encoder_init(){
  pinMode(pin_encoder_left, INPUT_PULLUP);
  pinMode(pin_encoder_right, INPUT_PULLUP);
  attachInterrupt(pin_interrupt_encoder_left, encoder_left_cb, CHANGE);
  attachInterrupt(pin_interrupt_encoder_right, encoder_right_cb, CHANGE);
}

void encoder_spin(){
  encoder_msg_left.data = encoder_value[0];
  encoder_msg_right.data = encoder_value[1];
  pub_encoder_left.publish(&encoder_msg_left);
  pub_encoder_right.publish(&encoder_msg_right);
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
  encoder_init();
  motor_init();
  servo_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  
  nh.advertise(pub_encoder_left);
  nh.advertise(pub_encoder_right);
  nh.subscribe(sub_motor_left);
  nh.subscribe(sub_motor_right);
  nh.subscribe(sub_servo_1);
  nh.subscribe(sub_servo_2);
}

void loop() {
  encoder_spin();
  motor_securely_system_update();
  nh.spinOnce();
  delay(DELAY);
}
