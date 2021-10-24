#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle nh;

#define servo_pin_1 8
#define servo_pin_2 9
#define BAUD 115200
#define DELAY 10

Servo servo_1;
Servo servo_2;

void servo_init(){
  servo_1.attach(servo_pin_1);
  servo_2.attach(servo_pin_2);
}

void servo_1_cb(const std_msgs::Int16& servo_1_cmd){
  servo_1.write(servo_1_cmd.data);
}

void servo_2_cb(const std_msgs::Int16& servo_2_cmd){
  servo_2.write(servo_2_cmd.data);
}

ros::Subscriber<std_msgs::Int16> sub_servo_1("/omegabot/cmd/servo/1", servo_1_cb);
ros::Subscriber<std_msgs::Int16> sub_servo_2("/omegabot/cmd/servo/2", servo_2_cb);

void setup() {
  servo_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub_servo_1);
  nh.subscribe(sub_servo_2);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
