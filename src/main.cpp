#include <Arduino.h>

int FL_MOTOR_PINS[] = {2, 3, 23};  // in1, in2, en
int RL_MOTOR_PINS[] = {4, 5, 25};
int FR_MOTOR_PINS[] = {10, 11, 24};
int RR_MOTOR_PINS[] = {8, 9, 26};

int LED_PIN = 13;

bool start = false;
int motor_speeds[] = {0, 0, 0, 0};

unsigned long last_time = 0;
unsigned long delta_time = 50;  // ms
unsigned long motor_speed_timeout = 1000;

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

unsigned char buffer[20];
size_t n_chars;

unsigned long time_stamp = 0;

void loop() {
  if (Serial.available()) {
    n_chars = Serial.readBytesUntil('\n', buffer, 20);
    if (n_chars > 0) {
      if (buffer[0] == 's') {
        for (int i = 0; i < 4; i++ ) {
          motor_speeds[i] = (buffer[i*2 + 2] << 8);
          motor_speeds[i] |= buffer[i * 2 + 1];
          Serial.print(motor_speeds[i]);
          Serial.print(' ');
        }
        time_stamp = millis();
        Serial.println(" ok");
        setMotorSpeed(FL_MOTOR_PINS, motor_speeds[0], true);
        setMotorSpeed(RL_MOTOR_PINS, motor_speeds[1], true);
        setMotorSpeed(FR_MOTOR_PINS, motor_speeds[2], true);
        setMotorSpeed(RR_MOTOR_PINS, motor_speeds[3], true);
      }
    }
  }
/*
  if ((time_stamp - millis()) > motor_speed_timeout) {
    for (int i = 0; i < 4; i++ ) {
        motor_speeds[i] = 0;
      }
  }*/

  
}

void setMotorSpeed(int motorPins[], float speed, bool def_dir) {
  if ((speed > 0) ^ def_dir) {
    analogWrite(motorPins[0], int(fabs(speed)));
    analogWrite(motorPins[1], 0);
  } else {
    analogWrite(motorPins[0], 0);
    analogWrite(motorPins[1], int(fabs(speed)));
  }
}