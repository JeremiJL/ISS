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
#define DEFAULT_SPEED 150
#define MOTOR_SWITCH_DELAY 5

// Timer
unsigned long timer_dt = millis();

// Sensors
const unsigned int CENTER_OF_THE_LINE = 2000;
constexpr int NUM_SENSORS = 5;
unsigned int sensorValues[NUM_SENSORS];
TRSensors trs = TRSensors();

// PID
const unsigned int dt = 50;

int past_error = 0;
double integral = 0;

const double proportion_weight = 0.5;
const double integral_weight = 0;
const double derivative_weight = 0.2;

// Logging - Debug
unsigned long calibration_iteration = 0;


// Debug
void log_state(int iteration_number, int current_error, int past_error,
                double PID, double proportion, double integral, double derivative) {

    // Iteration number
    Serial.print("Iteration : ");
    Serial.print(iteration_number);
    Serial.print("\t");

    // Print Current error
    Serial.print("Current error : ");
    Serial.print(current_error);
    Serial.print("\t");

    // Print Past Error
    Serial.print("Past error : ");
    Serial.print(past_error);
    Serial.print("\t");

    // Print PID
    Serial.print("PID : ");
    Serial.print(PID);
    Serial.print("\t");

    // Print proportion
    Serial.print("Weighted proportion : ");
    Serial.print(proportion);
    Serial.print("\t");

    // Print Integral
    Serial.print("Weighted integral : ");
    Serial.print(integral);
    Serial.print("\t");

    // Print Integral
    Serial.print("Weighted derivative : ");
    Serial.print(derivative);
    Serial.print("\n");
}

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
   // Auxuliary values
    unsigned int current_position = trs.readLine(sensorValues);
    int current_error = CENTER_OF_THE_LINE - current_position;

    // PID attributes
    double proportion = current_error;
    integral = integral + current_error * dt;
    double derivative = (past_error - current_error) / dt;

    // Caculation of PID value
    double PID = proportion * proportion_weight + integral * integral_weight + derivative * derivative_weight;

    // Reaction on the system proportional to PID value
    right_motor.set_speed(DEFAULT_SPEED + PID);
    left_motor.set_speed(DEFAULT_SPEED - PID);

    // Auxuliary values for second iteration
    past_error = current_error;

    // Incrementing number of calibrations for debug purposes
    calibration_iteration++;

    // Logging
    log_state(calibration_iteration, current_error, past_error, PID, proportion, integral, derivative);
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
