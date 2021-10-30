// ----- global parameters -----

#define BAUD 115200 // max stable for raspi is 115200; 250000 is semi-stable
#define DELAY 0

// ----- local parameters  -----
#define error_k_pow 1.0 
#define error_k_P 1.0
#define error_k_I 0.1
#define error_k_D 0.0
#define error_I_windup 0.2

#define pwm_target_max 255
#define pwm_target_min -127
#define pwm_target_lost_max 255
#define pwm_target_lost_min -127

#define pin_button_line_sensor_calibration 8
#define pin_led_calibration 13
#define calibration_delay 10
#define calibration_samples 1000

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

double line_sensor_sum_black_threshold = 1.5;
double line_sensor_sum_lost_threshold = 0.05;

// ----- line_sensor_calibration -----

int line_sensor_delta = 40;

int line_sensor_left_white = 303;
int line_sensor_left_black = 943;
int line_sensor_right_white = 147;
int line_sensor_right_black = 919;

// ----- line_sensor -----

double line_sensor_left_last = 0.0;
double line_sensor_right_last = 0.0;

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
  
  /*
  Serial.print(line_sensor_left_white);
  Serial.print(" - ");
  Serial.print(line_sensor_left_black);
  Serial.print(" - ");
  Serial.print(line_sensor_right_white);
  Serial.print(" - ");
  Serial.print(line_sensor_right_black);
  Serial.println("");
  */

  control_cmd(line_sensor_left, line_sensor_right);
  if ((line_sensor_left + line_sensor_right < line_sensor_sum_black_threshold) && 
      (line_sensor_left + line_sensor_right > line_sensor_sum_lost_threshold)){
        line_sensor_left_last = line_sensor_left;
        line_sensor_right_last = line_sensor_right;
    }
}

// ----- button_line_sensor_calibration -----

void button_line_sensor_calibration_init(){
  pinMode(pin_button_line_sensor_calibration, INPUT);
  pinMode(pin_led_calibration, OUTPUT);
}

void line_sensor_calibration(){
  for (int color = 0; color < 2; color++){
    digitalWrite(pin_led_calibration, LOW);
    while (digitalRead(pin_button_line_sensor_calibration) == LOW){
      delay(calibration_delay);
    }
    digitalWrite(pin_led_calibration, HIGH);
    long calibration_line_sensor_left = 0;
    long calibration_line_sensor_right = 0;
    for (int i = 0; i < calibration_samples; i++){
      calibration_line_sensor_left += analogRead(pin_line_sensor_left);
      calibration_line_sensor_right += analogRead(pin_line_sensor_right);
      delay(1);
    }
    calibration_line_sensor_left /= calibration_samples;
    calibration_line_sensor_right /= calibration_samples;
    if (color == 0){
      line_sensor_left_white = (int)calibration_line_sensor_left;
      line_sensor_right_white = (int)calibration_line_sensor_right;
    } else if (color == 1){
      line_sensor_left_black = (int)calibration_line_sensor_left;
      line_sensor_right_black = (int)calibration_line_sensor_right;
    }
  }

  line_sensor_left_white += line_sensor_delta;
  line_sensor_left_black -= line_sensor_delta;
  line_sensor_right_white += line_sensor_delta;
  line_sensor_right_black -= line_sensor_delta;

  
  int waiting_counter = 0;
  int led_state = HIGH;
  digitalWrite(pin_led_calibration, !led_state);
  while (digitalRead(pin_button_line_sensor_calibration) == LOW){
    waiting_counter += 1;
    if (waiting_counter > 100){
      waiting_counter = 0;
      digitalWrite(pin_led_calibration, led_state);
      led_state = !led_state;
    }
    delay(1);
  }
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

// ----- control_cmd -----

int clip(int value, int value_min, int value_max){
  if (value > value_max){
    value = value_max;
  }
  if (value < value_min){
    value = value_min;
  }
  return value;
}

double sign(double value){
  if (value > 0.0){
    return 1.0;
  }
  else {
    return -1.0;
  }
}


double error_I = 0.0;
double error_last = 0.0;
double error_time_last = 0;

void control_cmd(double line_sensor_left, double line_sensor_right){
  unsigned long error_time_new = millis();
  if (line_sensor_left + line_sensor_right < line_sensor_sum_lost_threshold){
    double line_sensor_error = line_sensor_right_last - line_sensor_left_last;
    int cmd_direction = 0;
    if (line_sensor_error > 0) {
      motor_cb(pwm_target_lost_max, pin_motor_dir_left, pin_motor_pwm_left);
      motor_cb(pwm_target_lost_min, pin_motor_dir_right, pin_motor_pwm_right);
    } else {
      motor_cb(pwm_target_lost_min, pin_motor_dir_left, pin_motor_pwm_left);
      motor_cb(pwm_target_lost_max, pin_motor_dir_right, pin_motor_pwm_right);
    }
    error_I = 0.0;
    error_last = 0.0;
    /*
    Serial.print(cmd_direction);
    Serial.print(" = ");
    Serial.print("LOST");
    Serial.println("");
    */
  }
  else if (line_sensor_left + line_sensor_right > line_sensor_sum_black_threshold) {
    /*
    Serial.print("BLACK");
    Serial.println("");
    */
    motor_cb(pwm_target_max, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(pwm_target_max, pin_motor_dir_right, pin_motor_pwm_right);
    error_I = 0.0;
    error_last = 0.0;
  }
  else {
    /*
    Serial.print(line_sensor_left);
    Serial.print(" - ");
    Serial.print(line_sensor_right);
    Serial.println("");
    */
    double error = line_sensor_left - line_sensor_right;
    error_I += error;
    error_I = clip(error_I, -error_I_windup, error_I_windup);
    double error_D = (error - error_last) / (1.0 * error_time_new - 1.0 * error_time_last);
    
    double error_cmd = error_k_P * error + error_k_I * error_I + error_k_D * error_D;
    error_cmd = sign(error_cmd) * pow(abs(error), error_k_pow);
    
    int cmd_left = pwm_target_max - (int)(error * (pwm_target_max - pwm_target_min));
    int cmd_right = pwm_target_max + (int)(error * (pwm_target_max - pwm_target_min));
    cmd_left = clip(cmd_left, pwm_target_min, pwm_target_max);
    cmd_right = clip(cmd_right, pwm_target_min, pwm_target_max);
    
    motor_cb(cmd_left, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(cmd_right, pin_motor_dir_right, pin_motor_pwm_right);
  }
  
  error_time_last = millis();
}


// ----- main -----

void setup() {
  Serial.begin(115200);
  line_sensor_init();
  encoder_init();
  motor_init();
  button_line_sensor_calibration_init();

  line_sensor_calibration();
}

void loop() {
  line_sensor_spin();
  motor_securely_system_update();
  
  
  delay(DELAY);
}
