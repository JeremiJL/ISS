#define MOTOR_PIN 9
#define SENSOR_PIN A0

#include <Servo.h>

// Timer
unsigned long timer_dt = millis();
unsigned long serial_read_timer = millis();
unsigned int serial_read_interval = 1000;

// PID
float set_point = 14;
const float dt = 0.1;

float past_error = 0.0;
float integral = 0.0;

const float proportion_weight = 0.8;
const float integral_weight = 0.08;
const float derivative_weight = 0.2;

// Sensors
const unsigned int sample_size = 200;
// Converts values from the exponential domain to the linear domain
const float linearity_conversion_scalar = 1.2134;
// Converts values from voltage domain to the domain of SI unit of length - centimeter
const float si_length_conversion_scalar = 22000;

const int reference_angle = 90;

// Servo
Servo myservo;

// Debug
void log_state(float current_position, float current_error, float pid, float proportion, float integral, float derivative, int angle) {

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

    // Print Angle
    Serial.print("Angle : ");
    Serial.print(angle);
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

    Serial.print("\n");
}

void process_serial() {
    String order = Serial.readStringUntil('\n');
    set_point = order.toInt() + 14;
}

float measure_distance_in_cm() {
    float raw_distance = 0;
    for (int i = 0; i < sample_size; i++) {
        raw_distance += analogRead(SENSOR_PIN);
    }

    raw_distance /= sample_size;

    // Print Weighted Derivative
    Serial.print("\nRaw distance : ");
    Serial.print(raw_distance);
    Serial.print("\n");

    float distance_in_cm = (pow(raw_distance, -linearity_conversion_scalar) * si_length_conversion_scalar);
    return distance_in_cm - 14;
}

void apply_to_servo(float pid) {
    int angle = constrain(reference_angle + pid, 70, 110);
    myservo.write(angle);
}

void neutralize_angle() {
    myservo.write(reference_angle);
}

void calibrate() {
    // Auxuliary values
    float current_position = measure_distance_in_cm();
    float current_error = current_position - set_point;

    // PID attributes
    float weighted_proportion = (current_error) * proportion_weight;
    integral += current_error * dt;
    float weighted_integral = integral * integral_weight;
    float weighted_derivative = ((past_error - current_error) / dt) * derivative_weight;

    // Caculation of PID value
    float pid = weighted_proportion + weighted_integral + weighted_derivative;

    // Reaction on the system proportional to PID value
    apply_to_servo(pid);

    // Logging
    int angle = myservo.read();
    log_state(current_position, current_error, pid, weighted_proportion, weighted_integral, weighted_derivative, angle);

    // Auxuliary values for second iteration
    past_error = current_error;
}

void setup() {
    Serial.begin(9600);
    myservo.attach(MOTOR_PIN);
    neutralize_angle();
}

void loop() {
    // 1. Process serial input, alter balancing position if neccesary

    const unsigned long current_time = millis();

    if (current_time - timer_dt >= static_cast<int>(dt * 100)) {
        timer_dt = current_time;
        calibrate();
    }

    if (current_time - timer_dt >= serial_read_interval) {
        serial_read_timer = current_time;
        process_serial();
    }
}