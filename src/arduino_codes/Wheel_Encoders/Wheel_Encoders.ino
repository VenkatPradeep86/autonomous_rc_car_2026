#define ENC_LEFT_A 0  // Left Encoder Channel A
#define ENC_LEFT_B 5  // Left Encoder Channel B
#define ENC_RIGHT_A 1 // Right Encoder Channel A
#define ENC_RIGHT_B 4 // Right Encoder Channel B

volatile int left_count = 0;
volatile int right_count = 0;
unsigned long last_time = 0;
float left_distance = 0.0;
float right_distance = 0.0;
float total_distance = 0.0;

// Constants
const float WHEEL_RADIUS = 0.050;  // 50 mm -> meters
const float WHEEL_CIRCUMFERENCE = 2 * 3.14159265359 * WHEEL_RADIUS;  // meters
const int TICKS_PER_REVOLUTION = 60;  // 30 slots -> 60 edges
const float DISTANCE_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION;  // 0.0022 meters per tick

// Interrupt service routines (ISR) for encoder pulses
void leftEncoderISR() {
    // Triggered on RISING edge of Channel A
    // Direction is determined by state of Channel B
    if (digitalRead(ENC_LEFT_B) == HIGH) {
        left_count--;    // One direction
    } else {
        left_count++;    // Opposite direction
    }
}

void rightEncoderISR() {
    if (digitalRead(ENC_RIGHT_B) == HIGH) {
        right_count++;   // One direction
    } else {
        right_count--;   // Opposite direction
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(ENC_LEFT_A, INPUT_PULLUP);
    pinMode(ENC_LEFT_B, INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, RISING);

    last_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    unsigned long time_diff = current_time - last_time;
     
    if (time_diff >=100) { // Calculate severy 100ms

        // Calculate distance traveled in the current run
        left_distance += left_count * DISTANCE_PER_TICK;
        right_distance += right_count * DISTANCE_PER_TICK;
        total_distance = (left_distance + right_distance) / 2.0;


        // Send data to ROS 2
        Serial.print("left_distance:");
        Serial.print(left_distance, 4);
        Serial.print(", right_distance:");
        Serial.print(right_distance, 4);
        Serial.print(", total_distance:");
        Serial.println(total_distance, 4);

        // Reset counters for next measurement
        left_count = 0;
        right_count = 0;
        last_time = current_time;
     }
    
}
