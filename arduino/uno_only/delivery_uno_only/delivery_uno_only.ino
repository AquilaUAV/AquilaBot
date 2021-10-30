// ----- global parameters -----

#define BAUD 115200 // max stable for raspi is 115200; 250000 is semi-stable
#define DELAY 0

// ----- local parameters  -----
#define pwm_target_crossroads 120
#define pwm_target_max 120
#define pwm_target_min 0
#define pwm_target_lost_max 120
#define pwm_target_lost_min 0

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

// ----- functions -----

double clip(int value, int value_min, int value_max){
  if (value > value_max){
    value = value_max;
  }
  if (value < value_min){
    value = value_min;
  }
  return value;
}

double clip(double value, double value_min, double value_max){
  if (value > value_max){
    value = value_max;
  }
  if (value < value_min){
    value = value_min;
  }
  return value;
}


int sign(int x){
  if (x >= 0){
    return 1;
  }
  else {
    return -1;
  }
}

int sign(double x){
  if (x >= 0.0){
    return 1;
  }
  else {
    return -1;
  }
}

// ----- line_sensor_logics -----

double line_sensor_sum_black_threshold = 1.5;
double line_sensor_sum_black_threshold_lower = 1.3;
double line_sensor_sum_lost_threshold = 0.05;

int lost_cmd = 0;

int iteration = 0;

int start_cmd = 0;
int rotation_cmd = 0; // 0 is stop *= 2
int forward_cmd = -1; // -1 is stop *= 2
int sonar_move_cmd = -1;
int stop_cmd = 0;

// ----- line_sensor_calibration -----

int line_sensor_delta = 30;

// 553 - 914 - 660 - 937
int line_sensor_left_white = 553; 
int line_sensor_left_black = 914;
int line_sensor_right_white = 660;
int line_sensor_right_black = 937;

// ----- line_sensor -----

double line_sensor_left_last = 0.0;
double line_sensor_right_last = 0.0;

void line_sensor_init(){
  pinMode(pin_line_sensor_left, INPUT);
  pinMode(pin_line_sensor_right, INPUT);
}

void line_sensor_spin(){
  double line_sensor_left = 1.0d * (analogRead(pin_line_sensor_left) - line_sensor_left_white) / (line_sensor_left_black - line_sensor_left_white);
  double line_sensor_right = 1.0d * (analogRead(pin_line_sensor_right) - line_sensor_right_white) / (line_sensor_right_black - line_sensor_right_white);
  line_sensor_left = clip(line_sensor_left, 0.0, 1.0);
  line_sensor_right = clip(line_sensor_right, 0.0, 1.0);
  if (line_sensor_left + line_sensor_right > line_sensor_sum_lost_threshold){
    line_sensor_left_last = line_sensor_left;
    line_sensor_right_last = line_sensor_right;
    lost_cmd = 0;
  }
  else {
    lost_cmd = sign(line_sensor_right_last - line_sensor_left_last);
  }
  control_cmd(line_sensor_left, line_sensor_right);
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
    
    Serial.print(line_sensor_left_white);
    Serial.print(" - ");
    Serial.print(line_sensor_left_black);
    Serial.print(" - ");
    Serial.print(line_sensor_right_white);
    Serial.print(" - ");
    Serial.print(line_sensor_right_black);
    Serial.println("");
    
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

// ----- control_loops -----

unsigned long encoder_last_left = 0;
unsigned long encoder_last_right = 0;

unsigned long encoder_check_timeout = 100;

double black_encoder_k_P = 20.0;
double black_encoder_k_I = 0.5;
double black_encoder_I_windup = 1.0 * pwm_target_crossroads;

double rotate_k_P = 10.0;
double rotate_k_I = 2.5;
double rotate_I_windup = 1.0 * pwm_target_crossroads;

int encoder_ticks_robot_length = -10; // 31
int encoder_ticks_half_rotation = 75;
int encoder_ticks_start = 30;
int encoder_ticks_stop = 30;

void control_loop_stop(){
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
  encoder_last_left = encoder_value[0];
  encoder_last_right = encoder_value[1];
  delay(encoder_check_timeout);
  while (encoder_value[0] != encoder_last_left && encoder_value[1] != encoder_last_right) {
    motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
    encoder_last_left = encoder_value[0];
    encoder_last_right = encoder_value[1];
    delay(encoder_check_timeout);
  }
}

void control_loop_move_steps(int steps){
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
  encoder_last_left = encoder_value[0];
  encoder_last_right = encoder_value[1];
  double error = 1.0 * abs(steps) - (1.0 * (encoder_value[0] + encoder_value[1] - encoder_last_left - encoder_last_right) / 2);
  double error_I = 0.0;
  while (error > 0.0){
    int control = (int)(round(sign(steps) * abs(black_encoder_k_P * error + black_encoder_k_I * error_I)));
    control = clip(control, -pwm_target_crossroads, pwm_target_crossroads);
    motor_cb(control, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(control, pin_motor_dir_right, pin_motor_pwm_right);
    delay(1);
    error = 1.0 * abs(steps) - (1.0 * (encoder_value[0] + encoder_value[1] - encoder_last_left - encoder_last_right) / 2);
    error_I += error * (1.0 / 1000.0);
    error_I = clip(error_I, -black_encoder_I_windup, black_encoder_I_windup);
    Serial.println(error);
    if (error <= 0.001){
      error = 0.0;
      error_I = 0.0;
      break;
    }
  }
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
}

void control_loop_rotate_steps(int steps, bool catch_lost = false){
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
  encoder_last_left = encoder_value[0];
  encoder_last_right = encoder_value[1];
  double error_rotate = abs(steps) - 1.0d * (encoder_value[0] + encoder_value[1] - encoder_last_right - encoder_last_left);
  double error_rotate_I = 0.0;
  while ((catch_lost && lost_cmd == 0) || (error_rotate > 0.0)){
    int control_rotate = (int)(round(sign(steps) * abs(rotate_k_P * error_rotate + rotate_k_I * error_rotate_I)));
    control_rotate = clip(control_rotate, -pwm_target_crossroads, pwm_target_crossroads);
    motor_cb(control_rotate, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(-control_rotate, pin_motor_dir_right, pin_motor_pwm_right);
    delay(1);
    error_rotate = abs(steps) - 1.0d * (encoder_value[0] + encoder_value[1] - encoder_last_right - encoder_last_left);
    error_rotate_I += error_rotate * (1.0 / 1000.0);
    error_rotate_I = clip(error_rotate_I, -rotate_I_windup, rotate_I_windup);
    Serial.println(control_rotate);
    if (error_rotate <= 0.001){
      error_rotate = 0.0;
      error_rotate_I = 0.0;
      break;
    }
  }
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
}

void control_loop_rotate_stabilize(){
  /*
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
  encoder_last_left = encoder_value[0];
  encoder_last_right = encoder_value[1];
  double error_left = abs(steps) - 1.0d * (encoder_value[0] - encoder_last_left);
  double error_left_I = 0.0;
  double error_right = abs(steps) - 1.0d * (encoder_value[1] - encoder_last_right);
  double error_right_I = 0.0;
  while ((catch_lost && lost_cmd == 0) || (error_left > 0.0 && error_right > 0.0)){
    int control_left = -(int)(round(sign(steps) * abs(rotate_k_P * error_left + rotate_k_I * error_left_I)));
    int control_right = (int)(round(sign(steps) * abs(rotate_k_P * error_right + rotate_k_I * error_right_I)));
    control_left = clip(control_left, -pwm_target_crossroads, pwm_target_crossroads);
    control_right = clip(control_right, -pwm_target_crossroads, pwm_target_crossroads);
    motor_cb(control_left, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(control_right, pin_motor_dir_right, pin_motor_pwm_right);
    delay(1);
    error_left = abs(steps) - 1.0d * (encoder_value[0] - encoder_last_left);
    error_right = abs(steps) - 1.0d * (encoder_value[1] - encoder_last_right);
    error_left_I += error_left * (1.0 / 1000.0);
    error_left_I = clip(error_left_I, -rotate_I_windup, rotate_I_windup);
    error_right_I += error_right * (1.0 / 1000.0);
    error_right_I = clip(error_right_I, -rotate_I_windup, rotate_I_windup);
    Serial.print(error_left);
    Serial.print(" - ");
    Serial.println(error_right);
    if (error_left <= 0.001){
      error_left = 0.0;
      error_left_I = 0.0;
    }
    if (error_right <= 0.001){
      error_right = 0.0;
      error_right_I = 0.0;
    }
    if (error_left <= 0.001 && error_right <= 0.001){
      break;
    }
  }
  */
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
}

// ----- control_cmd -----

unsigned long black_encoder_left = 0;
unsigned long black_encoder_right = 0;

void control_cmd(double line_sensor_left, double line_sensor_right){
  if (line_sensor_left + line_sensor_right > line_sensor_sum_black_threshold) {
    if (forward_cmd % 2 == 0 && forward_cmd > 0){
      forward_cmd -= 1;
    }
  }
  else if (line_sensor_left + line_sensor_right < line_sensor_sum_black_threshold_lower) {
    if (forward_cmd % 2 == 1 && forward_cmd > 0){
      forward_cmd -= 1;
    }
  }
  if (forward_cmd == 1){
    black_encoder_left = encoder_value[0];
    black_encoder_right = encoder_value[1];
    encoder_last_left = encoder_value[0] - 1;
    encoder_last_right = encoder_value[1] - 1;
  }
  //Serial.print(lost_cmd);
  //Serial.println("");
  
  //Serial.print(forward_cmd);
  //Serial.println("");
  if (start_cmd > 0) {
    control_loop_move_steps(encoder_ticks_start);
    start_cmd -= 1;
  }
  else if (rotation_cmd != 0){
    for (int i = 0; i < abs(rotation_cmd) - 1; i++){
      control_loop_rotate_steps(sign(rotation_cmd) * encoder_ticks_half_rotation, false);
      control_loop_rotate_stabilize();
      control_loop_stop();
    }
    control_loop_rotate_steps(sign(rotation_cmd) * encoder_ticks_half_rotation / 2, false);
    control_loop_rotate_steps(sign(rotation_cmd) * encoder_ticks_half_rotation, true);
    control_loop_rotate_stabilize();
    control_loop_stop();
    rotation_cmd = 0;
  }
  else if (forward_cmd % 2 == 0 && forward_cmd > 1){
    motor_cb(pwm_target_max - (int)(line_sensor_left * (pwm_target_max - pwm_target_min)), pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(pwm_target_max - (int)(line_sensor_right * (pwm_target_max - pwm_target_min)), pin_motor_dir_right, pin_motor_pwm_right);
  }
  else if (forward_cmd % 2 == 1){
    motor_cb(pwm_target_crossroads, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(pwm_target_crossroads, pin_motor_dir_right, pin_motor_pwm_right);
  }
  else if (forward_cmd == 0){
    control_loop_stop();
    int encoder_error_left = encoder_last_left - black_encoder_left;
    int encoder_error_right = encoder_last_right - black_encoder_right;
    int encoder_error = (encoder_error_left + encoder_error_right) / 2;
    control_loop_move_steps(encoder_ticks_robot_length - encoder_error);
    control_loop_stop();
    forward_cmd -= 1;
  }
  else if (stop_cmd > 0){
    control_loop_move_steps(encoder_ticks_stop);
    stop_cmd -= 1;
  }
  else {
    iteration += 1;
    if (iteration == 1){
      start_cmd = 1;
      rotation_cmd = 0; // 0 is stop *= 2
      forward_cmd = -1; // -1 is stop *= 2
      sonar_move_cmd = -1;
      stop_cmd = 0;

    } else if (iteration == 2){

      start_cmd = 0;
      rotation_cmd = 0; // 0 is stop *= 2
      forward_cmd = 4; // -1 is stop *= 2
      sonar_move_cmd = -1;
      stop_cmd = 0;

    } else if (iteration == 3){

      start_cmd = 0;
      rotation_cmd = 2; // 0 is stop *= 2
      forward_cmd = -1; // -1 is stop *= 2
      sonar_move_cmd = -1;
      stop_cmd = 0;

    } else if (iteration == 4){

      start_cmd = 0;
      rotation_cmd = 0; // 0 is stop *= 2
      forward_cmd = 2; // -1 is stop *= 2
      sonar_move_cmd = -1;
      stop_cmd = 0;

    }
    else {
      control_loop_stop();
      Serial.println("IDK");
      delay(1000);
    }
  }
  /*
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
    
    Serial.print(forward_cmd);
    Serial.print(" = ");
    Serial.print(cmd_direction);
    Serial.print(" = ");
    Serial.print("LOST");
    Serial.println("");
  }
  else {
    
    Serial.print(forward_cmd);
    Serial.print(" = ");
    Serial.print(line_sensor_left);
    Serial.print(" - ");
    Serial.print(line_sensor_right);
    Serial.println("");
    
    //motor_cb(pwm_target_max - (int)(line_sensor_left * (pwm_target_max - pwm_target_min)), pin_motor_dir_left, pin_motor_pwm_left);
    //motor_cb(pwm_target_max - (int)(line_sensor_right * (pwm_target_max - pwm_target_min)), pin_motor_dir_right, pin_motor_pwm_right);
  }
  */
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
