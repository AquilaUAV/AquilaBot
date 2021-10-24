#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
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
#define pin_sonar_trig 10
#define pin_sonar_echo 11
#define SONAR_CLEAR_Mc 2
#define SONAR_ACTIVE_Mc 10
#define DEFAULT_SONAR_PULSE_TIMEOUT 100000

#define BAUD 115200
#define DELAY 10

Servo servo_1;
Servo servo_2;

std_msgs::Int16MultiArray encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);
std_msgs::Int16MultiArray analog_read_msg;
ros::Publisher pub_analog_read("analog_read", &analog_read_msg);
std_msgs::Float32 sonar_distance_msg;
ros::Publisher pub_sonar_distance("sonar_distance", &sonar_distance_msg);

void encoder_1_cb(){
  encoder_msg.data[0]++;
}

void encoder_2_cb(){
  encoder_msg.data[1]++;
}

void encoder_init(){
  encoder_msg.data_length = 2;
  encoder_msg.data = (int*)malloc(sizeof(int) * 2);
  encoder_msg.data[0] = 0;
  encoder_msg.data[1] = 0;
  pinMode(pin_encoder_1, INPUT_PULLUP);
  pinMode(pin_encoder_2, INPUT_PULLUP);
  attachInterrupt(0, encoder_1_cb, CHANGE);
  attachInterrupt(1, encoder_2_cb, CHANGE);
}

void encoder_spin(){
  pub_encoder.publish(&encoder_msg);
}

void analog_init(){
  pinMode(pin_analog_1, OUTPUT);
  pinMode(pin_analog_2, OUTPUT);
  analog_read_msg.data_length = 2;
  analog_read_msg.data = (int*)malloc(sizeof(int) * 2);
}

void analog_spin(){
  analog_read_msg.data[0] = analogRead(pin_analog_1);
  analog_read_msg.data[1] = analogRead(pin_analog_2);
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
}

void servo_cb(const std_msgs::Int16MultiArray& cmd_array_msg){
  servo_1.write(cmd_array_msg.data[0]);
  servo_2.write(cmd_array_msg.data[1]);
}
ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo_cmd", servo_cb);

void sonar_cb(const std_msgs::Int16& sonar_pulse_timeout){
  digitalWrite(pin_sonar_trig, LOW);
  delayMicroseconds(SONAR_CLEAR_Mc);
  digitalWrite(pin_sonar_trig, HIGH);
  delayMicroseconds(SONAR_ACTIVE_Mc);
  digitalWrite(pin_sonar_trig, LOW);
  long tics;
  if (sonar_pulse_timeout.data == 0) {
    tics = pulseIn(pin_sonar_echo, HIGH, DEFAULT_SONAR_PULSE_TIMEOUT);
   } else {
    tics = pulseIn(pin_sonar_echo, HIGH, 1000l * sonar_pulse_timeout.data);
  }
  sonar_distance_msg.data = 0.00034 * tics / 2;
  pub_sonar_distance.publish(&sonar_distance_msg);
}
ros::Subscriber<std_msgs::Int16> sonar_pulse_timeout_sub("sonar_pulse_timeout", sonar_cb);

void sonar_init(){
  pinMode(pin_sonar_echo, INPUT);
  pinMode(pin_sonar_trig, OUTPUT);
}

void setup() {
  analog_init();
  motor_init();
  servo_init();
  sonar_init();
  encoder_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_analog_read);
  nh.subscribe(motor_sub);
  nh.subscribe(servo_sub);
  nh.advertise(pub_sonar_distance);
  nh.subscribe(sonar_pulse_timeout_sub);
  nh.advertise(pub_encoder);
}

void loop() {
  //analog_spin();
  encoder_spin();
  nh.spinOnce();
  delay(DELAY);
}
