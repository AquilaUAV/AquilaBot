// ----- global parameters -----

#define BAUD 115200 // max stable for raspi is 115200; 250000 is semi-stable
#define DELAY 0

// ----- local parameters  -----
#define pwm_target_init 140
#define pwm_target_max 255
#define pwm_target_min -255

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


#define rotate_k_P 7.0
#define move_k_P 7.0
#define move_dr_P 30.0
#define rotate_d_control_max 2

#define ticks_pi_2 75
#define ticks_line 500
#define ticks_dx 80
#define ticks_out 100

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


void stop_ticks(){
  motor_cb(0, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(0, pin_motor_dir_right, pin_motor_pwm_right);
  delay(250);
}

void rotate_ticks(int ticks){
  long ticks_target = (long)(abs(ticks)) + encoder_value[0] + encoder_value[1];
  motor_cb(-sign(ticks) * pwm_target_init, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(sign(ticks) * pwm_target_init, pin_motor_dir_right, pin_motor_pwm_right);
  delay(20);
  
  int error = (int)(round(rotate_k_P * (ticks_target - (encoder_value[0] + encoder_value[1]))));
  while (ticks_target > encoder_value[0] + encoder_value[1]){
    int error_last = error;
    error = (int)(round(rotate_k_P * (ticks_target - (encoder_value[0] + encoder_value[1]))));
    error = clip(error, -pwm_limit, error_last + rotate_d_control_max);
    Serial.println(ticks_target);
    if (error < 0){
      break;
    }
    motor_cb(-sign(ticks) * error, pin_motor_dir_left, pin_motor_pwm_left);
    motor_cb(sign(ticks) * error, pin_motor_dir_right, pin_motor_pwm_right);
    delay(20);
  }
  
  motor_cb(sign(ticks) * pwm_target_init, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(-sign(ticks) * pwm_target_init, pin_motor_dir_right, pin_motor_pwm_right);
  delay(20);
}

void move_ticks(int ticks){
  unsigned long encoder_value_start[2];
  encoder_value_start[0] = encoder_value[0];
  encoder_value_start[1] = encoder_value[1];
  long ticks_target = (long)(2 * abs(ticks)) + encoder_value[0] + encoder_value[1];
  motor_cb(sign(ticks) * pwm_target_init, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(sign(ticks) * pwm_target_init, pin_motor_dir_right, pin_motor_pwm_right);
  delay(20);
  
  int error = (int)(round(move_k_P * (ticks_target - (encoder_value[0] + encoder_value[1]))));
  while (ticks_target > encoder_value[0] + encoder_value[1]){
    int error_last = error;
    error = (int)(round(move_k_P * (ticks_target - (encoder_value[0] + encoder_value[1]))));
    error = clip(error, -pwm_limit, pwm_limit);
    int move_dr = (int) round(move_dr_P * (encoder_value[0] + encoder_value_start[0] - encoder_value_start[1] - encoder_value[1]));
    if (error < 0){
      break;
    }
    if (move_dr > 0){
      motor_cb(sign(ticks) * error - move_dr, pin_motor_dir_left, pin_motor_pwm_left);
      motor_cb(sign(ticks) * error, pin_motor_dir_right, pin_motor_pwm_right);
      Serial.print(sign(ticks) * error - move_dr);
      Serial.print(" = ");
      Serial.println(sign(ticks) * error);
    }
    else {
      motor_cb(sign(ticks) * error, pin_motor_dir_left, pin_motor_pwm_left);
      motor_cb(sign(ticks) * error + move_dr, pin_motor_dir_right, pin_motor_pwm_right);
      Serial.print(sign(ticks) * error);
      Serial.print(" = ");
      Serial.println(sign(ticks) * error + move_dr);
    }
    delay(20);
  }
  
#define move_k_P 7.0
#define move_dr_P 7.0
  motor_cb(-sign(ticks) * pwm_target_init, pin_motor_dir_left, pin_motor_pwm_left);
  motor_cb(-sign(ticks) * pwm_target_init, pin_motor_dir_right, pin_motor_pwm_right);
  delay(20);
}

// ----- main -----

void setup() {
  Serial.begin(115200);
  encoder_init();
  motor_init();

  move_ticks(ticks_line);
  stop_ticks();
  
  rotate_ticks(ticks_pi_2);
  stop_ticks();

  move_ticks(ticks_dx);
  stop_ticks();
  
  rotate_ticks(ticks_pi_2);
  stop_ticks();

  move_ticks(ticks_line + ticks_out);
  stop_ticks();
  
}

void loop() {
  motor_securely_system_update();
  
  delay(DELAY);
}
