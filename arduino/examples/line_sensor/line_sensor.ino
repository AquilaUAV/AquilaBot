#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

#define pin_line_sensor_L 0
#define pin_line_sensor_R 1
#define BAUD 115200
#define DELAY 10

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

void setup() {
  line_sensor_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_sensor_msg_L);
  nh.advertise(pub_sensor_msg_R);
}

void loop() {
  line_sensor_spin();
  nh.spinOnce();
  delay(DELAY);
}
