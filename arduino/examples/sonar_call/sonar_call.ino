#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;

#define pin_sonar_trig 10
#define pin_sonar_echo 11
#define SONAR_CLEAR_Mc 2
#define SONAR_ACTIVE_Mc 10
#define BAUD 115200
#define DELAY 10
#define DEFAULT_SONAR_PULSE_TIMEOUT 10000
#define SPEED_OF_SOUND_DEV_2 0.00017

std_msgs::Float32 sonar_distance_msg;
ros::Publisher pub_sonar_distance("/omegabot/sensor/sonar_distance", &sonar_distance_msg);

void sonar_cb(const std_msgs::UInt32& sonar_pulse_timeout){
  digitalWrite(pin_sonar_trig, LOW);
  delayMicroseconds(SONAR_CLEAR_Mc);
  digitalWrite(pin_sonar_trig, HIGH);
  delayMicroseconds(SONAR_ACTIVE_Mc);
  digitalWrite(pin_sonar_trig, LOW);
  unsigned long tics;
  if (sonar_pulse_timeout.data == 0) {
    tics = pulseIn(pin_sonar_echo, HIGH, DEFAULT_SONAR_PULSE_TIMEOUT);
   } else {
    tics = pulseIn(pin_sonar_echo, HIGH, sonar_pulse_timeout.data);
  }
  if (tics > 0){
    sonar_distance_msg.data = SPEED_OF_SOUND_DEV_2 * tics;
  }
  else {
    sonar_distance_msg.data = INFINITY;
  }
  pub_sonar_distance.publish(&sonar_distance_msg);
}

ros::Subscriber<std_msgs::UInt32> sonar_pulse_timeout_sub("/omegabot/call/sonar_timeout_mc", sonar_cb);

void sonar_init(){
  pinMode(pin_sonar_echo, INPUT);
  pinMode(pin_sonar_trig, OUTPUT);
}

void setup() {
  sonar_init();
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub_sonar_distance);
  nh.subscribe(sonar_pulse_timeout_sub);
}

void loop() {
  nh.spinOnce();
  delay(DELAY);
}
