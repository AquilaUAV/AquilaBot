#include <ros.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;

#define pin_encoder_left 2
#define pin_encoder_right 3
#define pin_interrupt_encoder_left 0
#define pin_interrupt_encoder_right 1
#define BAUD 115200
#define DELAY 10

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

void setup() {
  encoder_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_encoder_left);
  nh.advertise(pub_encoder_right);
}

void loop() {
  encoder_spin();
  nh.spinOnce();
  delay(DELAY);
}
