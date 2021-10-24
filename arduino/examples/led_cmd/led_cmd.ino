#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

#define pin_led 13
#define BAUD 115200
#define DELAY 10

void led_init(){
  pinMode(pin_led, OUTPUT);
}

void led_cb(const std_msgs::Bool &led_cmd){
  digitalWrite(pin_led, led_cmd.data);
}

ros::Subscriber<std_msgs::Bool> sub_led("/omegabot/cmd/led", led_cb);


void setup() {
  led_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(sub_led);
}

void loop() {
  nh.spinOnce();
  delay(DELAY);
}
