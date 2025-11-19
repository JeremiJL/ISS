#include "TRSensors.h"
#include <stdio.h>

// Pins
#define RIGHT_MOTOR_FWD A0
#define RIGHT_MOTOR_REV A1
#define RIGHT_MOTOR_ENABLE 5

#define LEFT_MOTOR_FWD A3
#define LEFT_MOTOR_REV A2
#define LEFT_MOTOR_ENABLE 6

// Speed
#define DEFAULT_SPEED 100
#define ROTATION_SPEED 50
#define MOTOR_SWITCH_DELAY 100

// Timer
unsigned long timer_dt = millis();

// Sensors
const unsigned int CENTER_OF_THE_LINE = 2000;
constexpr int NUM_SENSORS = 5;
unsigned int sensorValues[NUM_SENSORS];
TRSensors trs = TRSensors();

// PID
int past_error = 0;
int dt = 50;
double integral = 0;

const double proportion_weight = 1.0;
const double integral_weight = 1.0;
const double derivative_weight = 1.0;


// Sensor calibration
void calibrateSensors(TRSensors trs){
  Serial.println("Calibration started");
  for (int i = 0; i < 500; i++) {
    trs.calibrate();
  }
  Serial.println("Calibration ended");

  Serial.println("Min values:");
  for (int i = 0; i < 5; i++ ) {
    Serial.print(trs.calibratedMin[i]);
    Serial.print(", ");
  }
  Serial.println();

  Serial.println("Max values:");
  for (int i = 0; i < 5; i++ ) {
    Serial.print(trs.calibratedMax[i]);
    Serial.print(", ");
  }
  Serial.println();
}

// Motor configuration
class Motor {
    int pinFwd;
    int pinRev;
    int pinPow;

    public:
        init() {
          pinMode(pinFwd, OUTPUT);
          pinMode(pinRev, OUTPUT);
          pinMode(pinPow, OUTPUT);
        }
        set_speed(int speed) {
          speed = constrain(speed, 0, 255);
          analogWrite(pinPow, speed);
        }
        set_direction(bool forward) {
          if (forward) {
              digitalWrite(this->pinRev, LOW);
              delay(MOTOR_SWITCH_DELAY);
              digitalWrite(this->pinFwd, HIGH);
          } else {
              digitalWrite(this->pinFwd, LOW);
              delay(MOTOR_SWITCH_DELAY);
              digitalWrite(this->pinRev, HIGH);
          }
        }
        stop(){
          set_speed(0);
        }
        Motor(int pinFwd, int pinRev, int pinPow) {
          this->pinFwd = pinFwd;
          this->pinRev = pinRev;
          this->pinPow = pinPow;
        }
};

// Motor's global declarations
Motor right_motor(RIGHT_MOTOR_FWD,RIGHT_MOTOR_REV,RIGHT_MOTOR_ENABLE);
Motor left_motor(LEFT_MOTOR_FWD,LEFT_MOTOR_REV,LEFT_MOTOR_ENABLE);

// Movements
void stop() {
  right_motor.stop();
  left_motor.stop();
}

void move_forward() {
    right_motor.set_direction(true);
    left_motor.set_direction(true);

    right_motor.set_speed(DEFAULT_SPEED);
    left_motor.set_speed(DEFAULT_SPEED);
}

void calibrate() {
    unsigned int current_position = trs.readLine(sensorValues);
    int current_error = CENTER_OF_THE_LINE - current_position;

    // PID attributes
    double proportion = current_error;
    double derivative = (past_error - current_error) / dt;
    integral = integral + current_error * dt;

    double PID = proportion * proportion_weight + derivative * derivative_weight + integral * integral_weight;

    right_motor.set_speed(ROTATION_SPEED + PID);
    left_motor.set_speed(ROTATION_SPEED - PID);

    past_error = current_error;
}

void setup() {
  // Monitor output
  Serial.begin(9600);

  // Initialzie motors
  left_motor.init();
  right_motor.init();

  // Calibrate sensors
  calibrateSensors(trs);

  // Start moving
  move_forward();
}

void loop() {
    const unsigned long current_time = millis();

    if (current_time - timer_dt >= dt) { // timer for 100 ms
      timer_dt = current_time;
      calibrate();
    }
}
