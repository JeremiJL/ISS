#define MOTOR_PIN 9
#define SENSOR_PIN A0

#include <Servo.h>

// Timer
unsigned long timer_dt = millis();
unsigned long serial_read_timer = millis();
unsigned int serial_read_interval = 1000;

// PID
int unsigned set_point = 0;
unsigned int dt = 50;

int past_error = 0;
long long int integral = 0;

const double proportion_weight = 0.5;
const double integral_weight = 0.00001;
const double derivative_weight = 0.1;

// Sensors
const unsigned int sample_size = 100;
// Converts values from the exponential domain to the linear domain
const double linearity_conversion_scalar = 1.2;
// Converts values from infrared lights readings domain to the domain of SI unit of length - centimeter
const double si_length_conversion_scalar = 10;

// Debug
void log_state(int current_position, int current_error, double PID, double proportion, double integral, double derivative) {

    // Print Current position
    Serial.print("Current position : ");
    Serial.print(current_position);
    Serial.print("\t");

    // Print Current error
    Serial.print("Current error : ");
    Serial.print(current_error);
    Serial.print("\t");

    // Print PID
    Serial.print("PID : ");
    Serial.print(pid);
    Serial.print("\t");

    // Print Weighted proportion
    Serial.print("Proportion : ");
    Serial.print(proportion);
    Serial.print("\t");

    // Print Weighted Integral
    Serial.print("Integral : ");
    Serial.print(integral);
    Serial.print("\t");

    // Print Weighted Derivative
    Serial.print("Derivative : ");
    Serial.print(derivative);
    Serial.print("\t");
}

void process_serial() {
    String order = Serial.readStringUntil('\n');
    int set_point = order.toInt() + 14;
}

int measure_distance_in_cm() {
    int raw_distance = 0;
    for (int i = 0; i < sample_size; i++) {
        raw_distance += analogRead(SENSOR_PIN);
    }
    raw_distance /= sample_size;
    int distance_in_cm = (pow(raw_distance, -linearity_conversion_scalar) * si_length_conversion_scalar).toInt();
}

void pull_plane(double pid) {

}

void calibrate() {
    // Auxuliary values
    unsigned int current_position = measure_distance_in_cm();
    int current_error = current_position - set_point;

    // PID attributes
    double weighted_proportion = (current_error) * proportion_weight;
    integral += current_error * static_cast<int>(dt);
    double weighted_integral = integral * integral_weight;
    double weighted_derivative = ((static_cast<double>(past_error) - static_cast<double>(current_error)) / dt) * derivative_weight;

    // Caculation of PID value
    double pid = weighted_proportion + weighted_integral + weighted_derivative;

    // Reaction on the system proportional to PID value
    pull_plane(pid);

    // Logging
    log_state(current_position, current_error, pid, weighted_proportion, weighted_integral, weighted_derivative);

    // Auxuliary values for second iteration
    past_error = current_error;
}

setup() {

}

loop() {
    // 1. Process serial input, alter balancing position if neccesary

    const unsigned long current_time = millis();

    if (current_time - timer_dt >= dt) {
        timer_dt = current_time;
        calibrate();
    }

    if (current_time - timer_dt >= serial_read_interval) {
        serial_read_timer = current_time;
        process_serial();
    }
}