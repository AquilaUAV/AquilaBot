#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

#define pin_button 8
#define BAUD 115200
#define DELAY 10

std_msgs::Bool button_msg;
ros::Publisher pub_button_msg("/omegabot/sensor/button", &button_msg);

void button_init(){
  pinMode(pin_button, INPUT);
}

void button_spin(){
  button_msg.data = digitalRead(pin_button);
  pub_button_msg.publish(&button_msg);
}


void setup() {
  button_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_button_msg);
}

void loop() {
  button_spin();
  nh.spinOnce();
  delay(DELAY);
}
