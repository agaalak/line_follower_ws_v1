#include <Servo.h>

// Initialize servo objects
Servo myservo1;
Servo myservo2;

// Define feedback and servo pins
int pinFeedback1 = 9;
int pinFeedback2 = 11;
int pinServo1 = 5;
int pinServo2 = 3;

// Map function declaration (will be defined later)
long map(long x, long in_min, long in_max, long out_min, long out_max);

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 bps
  // Set feedback pins as input
  pinMode(pinFeedback1, INPUT);
  pinMode(pinFeedback2, INPUT);
  // Attach servos to their respective pins
  myservo1.attach(pinServo1);
  myservo2.attach(pinServo2);
}

void loop() {
  if (Serial.available() > 0) {
    // Expecting two comma-separated values for left and right motor speeds
    int leftMotorSpeed = Serial.parseInt();
    if (Serial.read() == ',') { // Check for comma delimiter
      int rightMotorSpeed = Serial.parseInt();
      
      // Map the motor speed values (-100 to 100) to servo pulse widths
      int leftServoPulse = map(leftMotorSpeed, -100, 100, 1000, 2000); // Example mapping
      int rightServoPulse = map(rightMotorSpeed, -100, 100, 1000, 2000); // Adjust these mappings as needed

      // Write the pulse widths to the servos
      myservo1.writeMicroseconds(leftServoPulse);
      myservo2.writeMicroseconds(rightServoPulse);
    }
  }
}

// Utility function to map one range of numbers to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}