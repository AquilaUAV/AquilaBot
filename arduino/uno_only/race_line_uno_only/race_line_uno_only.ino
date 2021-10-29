// ----- global parameters -----

#define BAUD 115200 // max stable for raspi is 115200; 250000 is semi-stable
#define DELAY 20

// ----- local parameters  -----
#define pwm_target_max 255
#define pwm_target_min 0

#define pin_line_sensor_left 0
#define pin_line_sensor_right 1

#define pin_encoder_left 2
#define pin_encoder_right 3
#define pin_interrupt_encoder_left 0
#define pin_interrupt_encoder_right 1

#define pin_motor_dir_right 4
#define pin_motor_pwm_right 5
#define pin_motor_pwm_left 6
#define pin_motor_dir_left 7
#define pwm_limit 255
#define motor_securely_system_timeout 250

// ----- line_sensor_logics -----

double line_sensor_sum_crossroads_threshold = 1.5;

// ----- line_sensor_calibration -----

int line_sensor_delta = 100;

int line_sensor_left_white = 324 + line_sensor_delta;
int line_sensor_left_black = 922 - line_sensor_delta;
int line_sensor_right_white = 160 + line_sensor_delta;
int line_sensor_right_black = 880 - line_sensor_delta;

// ----- line_sensor -----

double clip(double value, double value_min, double value_max){
  if (value > value_max){
    value = value_max;
  }
  if (value < value_min){
    value = value_min;
  }
  return value;
}

void line_sensor_init(){
  pinMode(pin_line_sensor_left, INPUT);
  pinMode(pin_line_sensor_right, INPUT);
}

void line_sensor_spin(){
  double line_sensor_left = 1.0d * (analogRead(pin_line_sensor_left) - line_sensor_left_white) / (line_sensor_left_black - line_sensor_left_white);
  double line_sensor_right = 1.0d * (analogRead(pin_line_sensor_right) - line_sensor_right_white) / (line_sensor_right_black - line_sensor_right_white);
  line_sensor_left = clip(line_sensor_left, 0.0, 1.0);
  line_sensor_right = clip(line_sensor_right, 0.0, 1.0);

  if (line_sensor_left + line_sensor_right > line_sensor_sum_crossroads_threshold){
    motor_cb(pwm_target_max, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(pwm_target_max, pin_motor_dir_right, pin_motor_pwm_right);
  }
  else {
    motor_cb(pwm_target_max - (int)(line_sensor_left * (pwm_target_max - pwm_target_min)), pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(pwm_target_max - (int)(line_sensor_right * (pwm_target_max - pwm_target_min)), pin_motor_dir_right, pin_motor_pwm_right);
  }
  
  
  Serial.print(line_sensor_left);
  Serial.print(" - ");
  Serial.print(line_sensor_right);
  Serial.println("");
}


// ----- encoder_sensor -----

unsigned long encoder_value[2];

void encoder_left_cb(){
  encoder_value[0]++;
}

void encoder_right_cb(){
  encoder_value[1]++;
}

void encoder_init(){
  pinMode(pin_encoder_left, INPUT_PULLUP);
  pinMode(pin_encoder_right, INPUT_PULLUP);
  attachInterrupt(pin_interrupt_encoder_left, encoder_left_cb, CHANGE);
  attachInterrupt(pin_interrupt_encoder_right, encoder_right_cb, CHANGE);
}

// ----- motor_cmd -----

unsigned long motor_update_time_last = 0;

void motor_init(){
  pinMode(pin_motor_dir_right, OUTPUT);
  pinMode(pin_motor_dir_left, OUTPUT);
  pinMode(pin_motor_pwm_right, OUTPUT);
  pinMode(pin_motor_pwm_left, OUTPUT);
}

void motor_cb(int motor_cmd, int motor_dir_pin, int motor_voltage_pin){
  if (motor_cmd > pwm_limit){
    motor_cmd = pwm_limit;
  }
  else if (motor_cmd < -pwm_limit){
    motor_cmd = -pwm_limit;
  }
  if (motor_cmd >= 0){
    digitalWrite(motor_dir_pin, HIGH);
  } else {
    digitalWrite(motor_dir_pin, LOW);
  }
  analogWrite(motor_voltage_pin, abs(motor_cmd));
}

void motor_securely_system_update(){
  unsigned long motor_update_time = millis();
  if (motor_update_time - motor_update_time_last > motor_securely_system_timeout){
    motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
    motor_update_time_last = motor_update_time;
  }
}

// ----- main -----

void setup() {
  line_sensor_init();
  encoder_init();
  motor_init();
  Serial.begin(115200);
}

void loop() {
  line_sensor_spin();
  motor_securely_system_update();
  
  //Serial.print(encoder_value[0]);

  //Serial.println("");
  
  
  delay(DELAY);
}
