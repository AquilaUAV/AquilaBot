#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

#define pin_line_sensor_left 0
#define pin_line_sensor_right 1
#define BAUD 115200
#define DELAY 10

std_msgs::Int16 line_sensor_msg_left;
std_msgs::Int16 line_sensor_msg_right;
ros::Publisher pub_sensor_msg_left("/omegabot/sensor/line/left", &line_sensor_msg_left);
ros::Publisher pub_sensor_msg_right("/omegabot/sensor/line/right", &line_sensor_msg_right);

void line_sensor_init(){
  pinMode(pin_line_sensor_left, INPUT);
  pinMode(pin_line_sensor_right, INPUT);
}

void line_sensor_spin(){
  line_sensor_msg_left.data = analogRead(pin_line_sensor_left);
  line_sensor_msg_right.data = analogRead(pin_line_sensor_right);
  pub_sensor_msg_left.publish(&line_sensor_msg_left);
  pub_sensor_msg_right.publish(&line_sensor_msg_right);
}

void setup() {
  line_sensor_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_sensor_msg_left);
  nh.advertise(pub_sensor_msg_right);
}

void loop() {
  line_sensor_spin();
  nh.spinOnce();
  delay(DELAY);
}
