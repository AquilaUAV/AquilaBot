#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

ros::NodeHandle nh;

#define pin_analog_1 0
#define pin_analog_2 1
#define DELAY 10

std_msgs::Int16MultiArray analog_read_msg;
ros::Publisher pub_analog_read("analog_read", &analog_read_msg);

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

void setup() {
  analog_init();
  
  nh.initNode();
  nh.advertise(pub_analog_read);
}

void loop() {
  analog_spin();
  nh.spinOnce();
  delay(DELAY);
}
