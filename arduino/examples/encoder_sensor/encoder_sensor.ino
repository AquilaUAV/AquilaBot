#include <ros.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;

#define pin_encoder_L 2
#define pin_encoder_R 3
#define pin_interrupt_encoder_L 0
#define pin_interrupt_encoder_R 1
#define BAUD 115200
#define DELAY 10

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

void setup() {
  encoder_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_encoder_L);
  nh.advertise(pub_encoder_R);
}

void loop() {
  encoder_spin();
  nh.spinOnce();
  delay(DELAY);
}
