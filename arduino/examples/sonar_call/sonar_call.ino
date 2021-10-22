#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/MultiArrayDimension.h>

ros::NodeHandle nh;

#define pin_sonar_trig 10
#define pin_sonar_echo 11
#define SONAR_CLEAR_Mc 2
#define SONAR_ACTIVE_Mc 10
#define DELAY 10
#define DEFAULT_SONAR_PULSE_TIMEOUT 10000

std_msgs::Float32 sonar_distance_msg;
ros::Publisher pub_sonar_distance("sonar_distance", &sonar_distance_msg);


void sonar_cb(const std_msgs::UInt32& sonar_pulse_timeout){
  digitalWrite(pin_sonar_trig, LOW);
  delayMicroseconds(SONAR_CLEAR_Mc);
  digitalWrite(pin_sonar_trig, HIGH);
  delayMicroseconds(SONAR_ACTIVE_Mc);
  digitalWrite(pin_sonar_trig, LOW);
  long tics;
  if (sonar_pulse_timeout.data == 0) {
    tics = pulseIn(pin_sonar_echo, HIGH, DEFAULT_SONAR_PULSE_TIMEOUT);
  } else {
    tics = pulseIn(pin_sonar_echo, HIGH, sonar_pulse_timeout.data);
  }
  sonar_distance_msg.data = 0.034 * tics / 2;
  pub_sonar_distance.publish(&sonar_distance_msg);
}
ros::Subscriber<std_msgs::UInt32> sonar_pulse_timeout_sub("sonar_pulse_timeout", sonar_cb);

void sonar_init(){
  pinMode(pin_sonar_echo, OUTPUT);
  pinMode(pin_sonar_trig, OUTPUT);
}

void setup() {
  sonar_init();
  
  nh.initNode();
  nh.advertise(pub_sonar_distance);
  nh.subscribe(sonar_pulse_timeout_sub);
}

void loop() {
  nh.spinOnce();
  delay(DELAY);
}
