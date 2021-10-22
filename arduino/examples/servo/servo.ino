#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

#define pin_servo_1 8
#define pin_servo_2 9
Servo servo_1;
Servo servo_2;

void servo_init(){
  servo_1.attach(pin_servo_1);
  servo_2.attach(pin_servo_2);
}

void servo_cb(const std_msgs::Int16MultiArray& cmd_array_msg){
  int servo_1_cmd = cmd_array_msg.data[0];
  int servo_2_cmd = cmd_array_msg.data[1];
  servo_1.write(servo_1_cmd);
  servo_2.write(servo_2_cmd);
}
ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo_cmd", servo_cb);

void setup() {
  servo_init();
  
  nh.initNode();
  nh.subscribe(servo_sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
