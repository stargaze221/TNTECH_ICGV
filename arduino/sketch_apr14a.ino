#include <Servo.h>

volatile uint32_t pwm_input_1 = 1500;
volatile uint32_t pwm_input_2 = 1500;
volatile uint32_t start_time_1 = 0;
volatile uint32_t start_time_2 = 0;

const int pwm_in_pin_1 = 2;
const int pwm_in_pin_2 = 3;
const int pwm_out_pin_1 = 10;
const int pwm_out_pin_2 = 9;

Servo servo1, servo2;

void setup() {
  pinMode(pwm_in_pin_1, INPUT);
  pinMode(pwm_in_pin_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(pwm_in_pin_1), pwm1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pwm_in_pin_2), pwm2_isr, CHANGE);

  servo1.attach(pwm_out_pin_1); // Sends proper 50Hz signals
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

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    int pwm1, pwm2;
    if (sscanf(command.c_str(), "%d,%d", &pwm1, &pwm2) == 2) {
      // PWM values in microseconds
      servo1.writeMicroseconds(pwm1);
      servo2.writeMicroseconds(pwm2);
    }
  }

  Serial.print("SENSOR:");
  Serial.print(pwm_input_1);
  Serial.print(",");
  Serial.println(pwm_input_2);
  delay(50);
}
