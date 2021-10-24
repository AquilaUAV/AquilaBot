#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

#define pin_buzzer 9
#define BAUD 115200
#define DELAY 10

void buzzer_init(){
  pinMode(pin_buzzer, OUTPUT);
}

void buzzer_cb(const std_msgs::Int16 &buzzer_cmd){
  analogWrite(pin_buzzer, buzzer_cmd.data);
}

ros::Subscriber<std_msgs::Int16> sub_buzzer("/omegabot/cmd/buzzer", buzzer_cb);


void setup() {
  buzzer_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub_buzzer);
}

void loop() {
  nh.spinOnce();
  delay(DELAY);
}
