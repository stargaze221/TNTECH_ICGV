#include <Servo.h>

volatile uint32_t pwm_input_1 = 1500;
volatile uint32_t pwm_input_2 = 1500;
volatile uint32_t start_time_1 = 0;
volatile uint32_t start_time_2 = 0;

const int pwm_in_pin_1 = 2;   // RC Steering
const int pwm_in_pin_2 = 3;   // RC Throttle
const int pwm_out_pin_1 = 10; // Servo Output
const int pwm_out_pin_2 = 9;  // ESC Output

const int PWM_CENTER = 1500;
const int PWM_MAX = 1900;
const int PWM_MIN = 1100;
const int SAFETY_MARGIN = 10;

Servo servo1, servo2;

void setup() {
  pinMode(pwm_in_pin_1, INPUT);
  pinMode(pwm_in_pin_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(pwm_in_pin_1), pwm1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwm_in_pin_2), pwm2_isr, CHANGE);

  servo1.attach(pwm_out_pin_1);
  servo2.attach(pwm_out_pin_2);

  Serial.begin(115200);
}

void pwm1_isr() {
  if (digitalRead(pwm_in_pin_1)) {
    start_time_1 = micros();
  } else {
    pwm_input_1 = micros() - start_time_1;
  }
}

void pwm2_isr() {
  if (digitalRead(pwm_in_pin_2)) {
    start_time_2 = micros();
  } else {
    pwm_input_2 = micros() - start_time_2;
  }
}

bool is_rc_safe() {
  return (pwm_input_1 >= PWM_MIN + SAFETY_MARGIN && pwm_input_1 <= PWM_MAX - SAFETY_MARGIN) &&
         (pwm_input_2 >= PWM_MIN + SAFETY_MARGIN && pwm_input_2 <= PWM_MAX - SAFETY_MARGIN);
}

void loop() {
  int pwm1 = PWM_CENTER;
  int pwm2 = PWM_CENTER;

  // Default to safe neutral values
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (sscanf(command.c_str(), "%d,%d", &pwm1, &pwm2) == 2) {
      // Apply only if RC is safe
      if (is_rc_safe()) {
        servo1.writeMicroseconds(pwm1);
        servo2.writeMicroseconds(pwm2);
      } else {
        servo1.writeMicroseconds(PWM_CENTER);
        servo2.writeMicroseconds(PWM_CENTER);
      }
    }
  }

  // Serial feedback for debugging
  Serial.print("SENSOR:");
  Serial.print(pwm_input_1);
  Serial.print(",");
  Serial.println(pwm_input_2);

  delay(50); // 20 Hz update
}
