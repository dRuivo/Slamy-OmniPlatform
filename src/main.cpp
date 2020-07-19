#include <Arduino.h>

int FL_MOTOR_PINS[] = {2, 3, 23};  // in1, in2, en
int RL_MOTOR_PINS[] = {4, 5, 25};
int FR_MOTOR_PINS[] = {6, 7, 22};
int RR_MOTOR_PINS[] = {8, 9, 24};

int LED_PIN = 13;

bool start = false;
int motor_speeds[] = {0, 0, 0, 0};

unsigned long last_time = 0;
unsigned long delta_time = 50;  // ms

void setMotorSpeed(int motorPins[], float speed, bool def_dir);

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(FL_MOTOR_PINS[i], OUTPUT);  
    pinMode(RL_MOTOR_PINS[i], OUTPUT);  
    pinMode(FR_MOTOR_PINS[i], OUTPUT);  
    pinMode(RR_MOTOR_PINS[i], OUTPUT);  
  }

  digitalWrite(FL_MOTOR_PINS[2], HIGH);
  digitalWrite(RL_MOTOR_PINS[2], HIGH);  
  digitalWrite(FR_MOTOR_PINS[2], HIGH);  
  digitalWrite(RR_MOTOR_PINS[2], HIGH);  

  last_time = millis();

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  if ((millis()- last_time) >= delta_time) {
    last_time = millis();
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') {
        start = true;
        Serial.write('s');
      } else if (c == 'p') {
        start = false;
        Serial.write('p');
      }
    } else {
      for (int i = 0; i < 3; i++) {
        motor_speeds[i] = 0;
      }
    }
  }
  

  if (start) {
    setMotorSpeed(FL_MOTOR_PINS, 128., true);
    setMotorSpeed(RL_MOTOR_PINS, 128., true);
    setMotorSpeed(FR_MOTOR_PINS, 128., false);
    setMotorSpeed(RR_MOTOR_PINS, 128., true);
  } else {
    setMotorSpeed(FL_MOTOR_PINS, 0., true);
    setMotorSpeed(RL_MOTOR_PINS, 0., true);
    setMotorSpeed(FR_MOTOR_PINS, 0., true);
    setMotorSpeed(RR_MOTOR_PINS, 0., true);
  }
}

void setMotorSpeed(int motorPins[], float speed, bool def_dir) {
  if ((speed > 0) ^ def_dir) {
    analogWrite(motorPins[0], int(speed));
    analogWrite(motorPins[1], 0);
  } else {
    analogWrite(motorPins[0], 0);
    analogWrite(motorPins[1], int(speed));
  }
}