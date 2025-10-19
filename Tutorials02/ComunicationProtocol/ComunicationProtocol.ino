#include <string.h>

// Protocol ;
// Move right by x cm : move-right x
String MOVE_RIGHT = "move-right";
// Move left by x cm : move-left x
String MOVE_LEFT = "move-left";
// Move forward by x cm : move-forward x
String MOVE_FORWARD = "move-forward";
// Move backwards by x cm : move-backwards x
String MOVE_BACKWARDS = "move-backwards";
// Rotate in left direction by y angle : rotate-left y
String ROTATE_LEFT = "rotate-left";
// Rotate in right direction by y angle : rotate-right y
String ROTATE_RIGHT = "rotate-right";


#define RIGHT_MOTOR_IN1 A0
#define RIGHT_MOTOR_IN2 A1
#define RIGHT_MOTOR_ENABLE 5

#define LEFT_MOTOR_IN1 A2
#define LEFT_MOTOR_IN2 A3
#define LEFT_MOTOR_ENABLE 6

#define MOTOR_SWITCH_DELAY 20

#define ENCODER_PIN_R 2
#define ENCODER_PIN_L 3

volatile int counter_r = 0;
volatile int counter_l = 0;

float const SIGNALS_BY_CM_RATIO = 0.857;
float const ANGLE_TO_DISTANCE_RATIO = 0.5;

const int DEFAULT_SPEED = 255;

String const path = "move-forward 10\nrotate-right 90\nmove-backwards 4\nrotate-left 45\nmove-right 6\n";

// Debug :
void print_state(String order) {
    Serial.print("Right counter: ");
    Serial.print(counter_r);
    Serial.print("\n");

    Serial.print("Left counter: ");
    Serial.print(counter_l);
    Serial.print("\n");

    Serial.print("Order : ");
    Serial.print(order);
    Serial.print("\n");
}

// Counters of motor sensors :
void increment_right_counter() {
    counter_r++;
}

void increment_left_counter() {
    counter_l++;
}

// Direction of motors :
void set_right_motor_direction(bool backwards) {
    if (backwards) {
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    } else {
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    }
}

void set_left_motor_direction(bool backwards) {
    if (backwards) {
        digitalWrite(LEFT_MOTOR_IN1, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
    } else {
        digitalWrite(LEFT_MOTOR_IN2, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
    }
}

// Speed of motors:
void set_motor_speed(int speed, int motor) {
    speed = constrain(speed, 0, 255);
    analogWrite(motor, speed);
}
void set_right_motor_speed(int speed) {
    set_motor_speed(speed, RIGHT_MOTOR_ENABLE);
}
void set_left_motor_speed(int speed) {
    set_motor_speed(speed, LEFT_MOTOR_ENABLE);
}

// Units conversion
int get_goal_ticks_from_cm(int distance_in_cm, int current_counter_value) {
    int ticks = (int)(SIGNALS_BY_CM_RATIO * distance_in_cm);
    int goal_ticks = current_counter_value + ticks;
    return goal_ticks;
}

// Movements :
void move_right(int distance_in_cm) {
    set_right_motor_direction(false);
    int goal = get_goal_ticks_from_cm(distance_in_cm, counter_r);

    set_right_motor_speed(DEFAULT_SPEED);
    while(counter_r < goal) {}
    set_right_motor_speed(0);
}

void move_left(int distance_in_cm) {
    set_left_motor_direction(false);
    int goal = get_goal_ticks_from_cm(distance_in_cm, counter_l);

    set_left_motor_speed(DEFAULT_SPEED);
    while(counter_l < goal) {}
    set_left_motor_speed(0);
}

void move_in_line(int distance_in_cm, bool backwards) {
    set_right_motor_direction(backwards);
    set_left_motor_direction(backwards);

    // assuming that motors are equally calibrated
    int right_goal = get_goal_ticks_from_cm(distance_in_cm, counter_r);

    set_right_motor_speed(DEFAULT_SPEED);
    set_left_motor_speed(DEFAULT_SPEED);
    while(counter_r < right_goal) {}
    set_right_motor_speed(0);
    set_left_motor_speed(0);
}

void move_forward(int distance_in_cm) {
    move_in_line(distance_in_cm, false);
}

void move_backwards(int distance_in_cm) {
    move_in_line(distance_in_cm, true);
}

// Rotations :
void rotate(int angle, bool right) {
    int distance_in_cm = angle * ANGLE_TO_DISTANCE_RATIO;
    int right_goal = get_goal_ticks_from_cm(distance_in_cm, counter_r);

    set_right_motor_direction(right);
    set_left_motor_direction(!right);

    set_right_motor_speed(DEFAULT_SPEED);
    set_left_motor_speed(DEFAULT_SPEED);
    while(counter_r < right_goal) {}
    set_right_motor_speed(0);
    set_left_motor_speed(0);
}
void rotate_right(int angle) {
    rotate(angle, true);
}

void rotate_left(int angle) {
    rotate(angle, false);
}

// Process user's orders
int extract_distance_from_order(String raw_order, String movement) {
    int distance = raw_order.substring(movement.length(), raw_order.length()).toInt();
    return distance;
}

void process_orders(String order) {
    if (order.startsWith(MOVE_RIGHT)){
        int distance_in_cm = extract_distance_from_order(order, MOVE_RIGHT);
        move_right(distance_in_cm);

    } if (order.startsWith(MOVE_LEFT)){
        int distance_in_cm = extract_distance_from_order(order, MOVE_LEFT);
        move_left(distance_in_cm);

    } if (order.startsWith(ROTATE_LEFT)){
        int angle = extract_distance_from_order(order, ROTATE_LEFT);
        rotate_left(angle);

    } if (order.startsWith(ROTATE_RIGHT)){
        int angle = extract_distance_from_order(order, ROTATE_RIGHT);
        rotate_right(angle);

    } if (order.startsWith(MOVE_FORWARD)){
        int distance_in_cm = extract_distance_from_order(order, MOVE_FORWARD);
        move_forward(distance_in_cm);

    } if (order.startsWith(MOVE_BACKWARDS)){
        int distance_in_cm = extract_distance_from_order(order, MOVE_BACKWARDS);
        move_backwards(distance_in_cm);

    } else {
        Serial.print("No order matched : [");
        Serial.print(order);
        Serial.print("]\n");
    }
}

// Initialize
void setup() {
    // Start serial
    Serial.begin(9600);
    // Coonfigure wheels state monitoring
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), increment_right_counter, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), increment_left_counter, RISING);
    // Configure wheels' motors
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
}

// Loop
void loop() {
    // Read from
    String order = Serial.readStringUntil('\n');
    // Process orders
    process_orders(order);
    // Display state
    print_state(order);
}

// Notes :

// X = 18 [signal rises]
// Y = 21 [cm]
// X/Y = 0.857 [signal/cm]
