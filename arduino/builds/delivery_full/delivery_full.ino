#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <Servo.h>

// subscribers, publishers, input buffer size, output buffer size
ros::NodeHandle_<ArduinoHardware, 7, 5, 128, 128> nh;

// ----- global parameters -----

#define BAUD 500000
#define DELAY 10

// ----- local parameters  -----

#define pin_sonar_trig 10
#define pin_sonar_echo 11
#define SONAR_CLEAR_Mc 2
#define SONAR_ACTIVE_Mc 10
#define DEFAULT_SONAR_PULSE_TIMEOUT 10000
#define SPEED_OF_SOUND_DEV_2 0.00017

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

#define servo_pin_1 8
#define servo_pin_2 9
#define servo_pin_3 A0
#define servo_pin_4 A1

// ----- sonar_call -----

std_msgs::Float32 sonar_distance_msg;
ros::Publisher pub_sonar_distance("/omegabot/sensor/sonar_distance", &sonar_distance_msg);

void sonar_cb(const std_msgs::UInt32& sonar_pulse_timeout){
  digitalWrite(pin_sonar_trig, LOW);
  delayMicroseconds(SONAR_CLEAR_Mc);
  digitalWrite(pin_sonar_trig, HIGH);
  delayMicroseconds(SONAR_ACTIVE_Mc);
  digitalWrite(pin_sonar_trig, LOW);
  unsigned long tics;
  if (sonar_pulse_timeout.data == 0) {
    tics = pulseIn(pin_sonar_echo, HIGH, DEFAULT_SONAR_PULSE_TIMEOUT);
   } else {
    tics = pulseIn(pin_sonar_echo, HIGH, sonar_pulse_timeout.data);
  }
  if (tics > 0){
    sonar_distance_msg.data = SPEED_OF_SOUND_DEV_2 * tics;
  }
  else {
    sonar_distance_msg.data = INFINITY;
  }
  pub_sonar_distance.publish(&sonar_distance_msg);
}

ros::Subscriber<std_msgs::UInt32> sonar_pulse_timeout_sub("/omegabot/call/sonar_timeout_mc", sonar_cb);

void sonar_init(){
  pinMode(pin_sonar_echo, INPUT);
  pinMode(pin_sonar_trig, OUTPUT);
}

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
Servo servo_3;
Servo servo_4;

void servo_init(){
  servo_1.attach(servo_pin_1);
  servo_2.attach(servo_pin_2);
  servo_3.attach(servo_pin_3);
  servo_4.attach(servo_pin_4);
}

void servo_1_cb(const std_msgs::Int16& servo_1_cmd){
  servo_1.write(servo_1_cmd.data);
}

void servo_2_cb(const std_msgs::Int16& servo_2_cmd){
  servo_2.write(servo_2_cmd.data);
}

void servo_3_cb(const std_msgs::Int16& servo_3_cmd){
  servo_3.write(servo_3_cmd.data);
}

void servo_4_cb(const std_msgs::Int16& servo_4_cmd){
  servo_4.write(servo_4_cmd.data);
}

ros::Subscriber<std_msgs::Int16> sub_servo_1("/omegabot/cmd/servo/1", servo_1_cb);
ros::Subscriber<std_msgs::Int16> sub_servo_2("/omegabot/cmd/servo/2", servo_2_cb);
ros::Subscriber<std_msgs::Int16> sub_servo_3("/omegabot/cmd/servo/3", servo_3_cb);
ros::Subscriber<std_msgs::Int16> sub_servo_4("/omegabot/cmd/servo/4", servo_4_cb);

// ----- main -----

void setup() {
  sonar_init();
  line_sensor_init();
  encoder_init();
  motor_init();
  servo_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  
  nh.advertise(pub_sonar_distance);
  nh.subscribe(sonar_pulse_timeout_sub);
  nh.advertise(pub_sensor_msg_L);
  nh.advertise(pub_sensor_msg_R);
  nh.advertise(pub_encoder_L);
  nh.advertise(pub_encoder_R);
  nh.subscribe(sub_motor_left);
  nh.subscribe(sub_motor_right);
  nh.subscribe(sub_servo_1);
  nh.subscribe(sub_servo_2);
  nh.subscribe(sub_servo_3);
  nh.subscribe(sub_servo_4);
}

void loop() {
  line_sensor_spin();
  encoder_spin();
  motor_securely_system_update();
  nh.spinOnce();
  delay(DELAY);
}
