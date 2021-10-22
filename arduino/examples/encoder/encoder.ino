#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::NodeHandle nh;

#define pin_encoder_1 2
#define pin_encoder_2 3
#define DELAY 10

std_msgs::UInt16MultiArray encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);

unsigned int encoder_value[2];

void encoder_1_cb(){
  encoder_value[0]++;
}

void encoder_2_cb(){
  encoder_value[1]++;
}

void encoder_init(){
  encoder_msg.data_length = 2;
  encoder_msg.data = (unsigned int*)malloc(sizeof(unsigned int) * 2);
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

void setup() {
  encoder_init();
  
  nh.initNode();
  nh.advertise(pub_encoder);
}

void loop() {
  encoder_spin();
  nh.spinOnce();
  delay(DELAY);
}
