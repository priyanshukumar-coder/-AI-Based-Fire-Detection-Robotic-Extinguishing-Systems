#include <Servo.h>

// Define servo objects for each axis
Servo servoX;
Servo servoY;
Servo servoZ;

// Define pins for each servo
const int servoXPin = 9;
const int servoYPin = 10;
const int servoZPin = 11;

// Define buzzer pin
const int buzzerPin = 13;

// Variables to store current positions
int currentX = 90;
int currentY = 90;
int currentZ = 90;

void setup() {
  Serial.begin(9600);
  
  // Attach servos to pins
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoZ.attach(servoZPin);
  
  // Set up buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  
  // Move to initial position
  servoX.write(currentX);
  servoY.write(currentY);
  servoZ.write(currentZ);
  
  delay(2000); // Allow servos to reach initial position
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("X") && command.indexOf("Y") != -1 && 
        command.indexOf("Z") != -1 && command.indexOf("B") != -1) {
      // Parse coordinates and buzzer command
      int xIndex = command.indexOf('X');
      int yIndex = command.indexOf('Y');
      int zIndex = command.indexOf('Z');
      int bIndex = command.indexOf('B');
      
      int x = command.substring(xIndex + 1, yIndex).toInt();
      int y = command.substring(yIndex + 1, zIndex).toInt();
      int z = command.substring(zIndex + 1, bIndex).toInt();
      int buzzer = command.substring(bIndex + 1).toInt();
      
      // Constrain values to safe ranges
      x = constrain(x, 0, 180);
      y = constrain(y, 0, 180);
      z = constrain(z, 0, 180);
      
      // Control buzzer
      digitalWrite(buzzerPin, buzzer == 1 ? HIGH : LOW);
      
      // Move servos smoothly
      moveServo(servoX, currentX, x);
      moveServo(servoY, currentY, y);
      moveServo(servoZ, currentZ, z);
      
      // Update current positions
      currentX = x;
      currentY = y;
      currentZ = z;
      
      // Send acknowledgment
      Serial.println("ACK");
    }
  }
}

void moveServo(Servo &servo, int from, int to) {
  int step = from < to ? 1 : -1;
  for (int pos = from; pos != to; pos += step) {
    servo.write(pos);
    delay(15); // Adjust for smoother/faster movement
  }
  servo.write(to);
}