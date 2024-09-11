// //arduino not doneeeeeeeee


// #include<Wire.h>

// #define speedL 10
// #define IN1 9
// #define IN2 8
// #define IN3 7
// #define IN4 6
// #define speedR 5
// #define trig 11
// #define echo 4

// long duration, distance;

// void setup() {
//   // Initialize pins
//   for (int i = 5; i <= 11; i++) {
//     pinMode(i, OUTPUT);
//   }
//   pinMode(echo, INPUT);

//   // Initialize serial communication for debugging
//   Serial.begin(9600); // Serial to communicate with ESP32
// }

// void Ultrasonic() {
//   // Clear the TRIG pin
//   digitalWrite(trig, LOW);
//   delayMicroseconds(2);

//   // Set TRIG pin HIGH for 10 microseconds
//   digitalWrite(trig, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trig, LOW);

//   // Read the ECHO pin
//   duration = pulseIn(echo, HIGH);

//   // Calculate distance
//   distance = (duration / 2) * 0.0343;
//   delay(5);
// }

// void forward() {
//   digitalWrite(IN1, HIGH);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
//   analogWrite(speedL, 75);
//   analogWrite(speedR, 75);
// }

// void backward() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, HIGH);
//   analogWrite(speedL, 75);
//   analogWrite(speedR, 75);
// }

// void left() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
//   analogWrite(speedL, 0);
//   analogWrite(speedR, 150);
// }

// void right() {
//   digitalWrite(IN1, HIGH);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, LOW);
//   analogWrite(speedL, 150);
//   analogWrite(speedR, 0);
// }

// void stopp() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, LOW);
//   analogWrite(speedL, 0);
//   analogWrite(speedR, 0);
// }

// void loop() {
//   Ultrasonic(); // Measure distance
//   // Serial.println("Hello from Arduino");
//   //delay(1000); // Send message every second
//   // Send distance to the serial port
//   Serial.print("Distance: ");
//   Serial.println(distance);
//   delay(100);
//   if (distance < 20 && distance > 0) { // Obstacle detected
//     stopp(); // Stop the car
//     delay(250); // Wait for half a second to ensure it's stopped

//     // Check the right side
//     right();
//     delay(500); // Turn right for a short duration
//     stopp(); // Stop again to measure distance

//     Ultrasonic(); // Measure distance again after turning right

//     if (distance <= 20) { // Path to the right is clear
//       right(); // Turn right
//       delay(500); // Continue turning right for 1 second
//       forward(); // Continue moving forward
//     } else {
//       // If the right is not clear, check the left side
//       left();
//       delay(500); // Turn left for a short duration
//       stopp(); // Stop again to measure distance

//       Ultrasonic(); // Measure distance again after turning left

//       if (distance <= 20) { // Path to the left is clear
//         left(); // Turn left
//         delay(500); // Continue turning left for 1 second
//         forward(); // Continue moving forward
//       } else {
//         // If both sides are not clear, stop or add additional logic
//         stopp();
//       }
//     }
//   } else {
//     forward(); // No obstacle detected, continue moving forward
//   }
// }
 
















 //____________________________________________________________________________


#include <Wire.h>

#define SPEED_L 10
#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
#define SPEED_R 5
#define TRIG 11
#define ECHO 4

long duration, distance;

void setup() {
  // Initialize pins
  for (int i = SPEED_L; i <= TRIG; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(ECHO, INPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600); // Serial to communicate with ESP32
}

void ultrasonic() {
  // Clear the TRIG pin
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // Set TRIG pin HIGH for 10 microseconds
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read the ECHO pin
  duration = pulseIn(ECHO, HIGH);

  // Calculate distance
  distance = (duration / 2) * 0.0343;
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(SPEED_L, 100);
  analogWrite(SPEED_R, 100);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(SPEED_L, 100);
  analogWrite(SPEED_R, 100);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(SPEED_L, 0);
  analogWrite(SPEED_R, 150);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(SPEED_L, 150);
  analogWrite(SPEED_R, 0);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(SPEED_L, 0);
  analogWrite(SPEED_R, 0);
}

void loop() {
  ultrasonic(); // Measure distance
  Serial.print("Distance: ");
  Serial.println(distance);
  
  if (distance < 20 && distance > 0) { // Obstacle detected
    stop(); // Stop the car
    delay(250); // Wait for half a second to ensure it's stopped

    // Check the right side
    right();
    delay(500); // Turn right for a short duration
    stop(); // Stop again to measure distance

    ultrasonic(); // Measure distance again after turning right

    if (distance > 20) { // Path to the right is clear
      forward(); // Move forward
    } else {
      // If the right is not clear, check the left side
      left();
      delay(500); // Turn left for a short duration
      stop(); // Stop again to measure distance

      ultrasonic(); // Measure distance again after turning left

      if (distance > 20) { // Path to the left is clear
        forward(); // Move forward
      } else {
        // If both sides are not clear, move backward or stop
        backward();
        delay(500); // Move backward for a short duration
        stop();
      }
    }
  } else {
    forward(); // No obstacle detected, continue moving forward
  }
}
