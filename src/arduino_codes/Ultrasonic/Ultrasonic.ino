// ---------------------------------------------------------
//  Arduino Micro Ultrasonic Firmware
//  Pins 0/1 are VALID for sensors on this board.
// ---------------------------------------------------------

const int TRIG_UL = 1;   // us_left (Pin 1 is TX on Micro, usable as Digital)
const int ECHO_UL = 4;

const int TRIG_URL = 5;  // us_rear_left
const int ECHO_URL = 0;  // us_rear_left (Pin 0 is RX on Micro, usable as Digital)

const int TRIG_URC = 6;  // us_rear_center
const int ECHO_URC = 2;

const int TRIG_URR = 3;  // us_rear_right
const int ECHO_URR = 8;

const int TRIG_UR = 9;   // us_right
const int ECHO_UR = 7;

void setup() {
    // On Micro, "Serial" is the USB connection to the PC
    Serial.begin(115200);

    pinMode(TRIG_UL, OUTPUT);
    pinMode(ECHO_UL, INPUT);
    pinMode(TRIG_URC, OUTPUT);
    pinMode(ECHO_URC, INPUT);
    pinMode(TRIG_URL, OUTPUT);
    pinMode(ECHO_URL, INPUT);
    pinMode(TRIG_URR, OUTPUT);
    pinMode(ECHO_URR, INPUT);
    pinMode(TRIG_UR, OUTPUT);
    pinMode(ECHO_UR, INPUT);
}

float readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Timeout: 12ms is roughly 2 meters. 
    // We don't need to wait 25ms (4 meters) for a small car.
    // This makes the code run 2x faster.
    long duration = pulseIn(echoPin, HIGH, 12000); 

    if (duration == 0) return 2.5; // Return 2.5m if no echo (open space)

    // Convert to meters
    return (duration * 0.0343 / 2.0) / 100.0;
}

void loop() {
    // Read sensors
    float d_UL  = readUltrasonic(TRIG_UL, ECHO_UL);
    delay(5); // Small delay to prevent cross-talk
    
    float d_URC = readUltrasonic(TRIG_URC, ECHO_URC);
    delay(5);
    
    float d_URL = readUltrasonic(TRIG_URL, ECHO_URL);
    delay(5);
    
    float d_URR = readUltrasonic(TRIG_URR, ECHO_URR);
    delay(5);
    
    float d_UR  = readUltrasonic(TRIG_UR, ECHO_UR);
    delay(5);

    // Send formatted string to ROS
    // Format: "key:value, key:value"
    Serial.print("us_left:"); Serial.print(d_UL);
    Serial.print(", us_rear_center:"); Serial.print(d_URC);
    Serial.print(", us_rear_left:"); Serial.print(d_URL);
    Serial.print(", us_rear_right:"); Serial.print(d_URR);
    Serial.print(", us_right:"); Serial.println(d_UR);

    // IMPORTANT: No massive delay here. 
    // We want data as fast as possible.
    delay(20); 
}
