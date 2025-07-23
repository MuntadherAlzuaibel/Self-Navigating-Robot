#include <Arduino.h>

// Uncomment the following lines if you want to use LiquidCrystal_I2C
/*
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
*/

#define PWM_FREQ 1000
#define PWM_RES 8

// Define pins for ultrasonic sensor
#define US_echo 32
#define US_trig 33

// Define pins for servo motor
#define SERVO_PIN 2    
#define SERVO_CHANNEL 4  // Not 0-3 because they are used by motors

// Define pins for IR sensors
#define R_IR 35
#define L_IR 34

float distance, Rdistance, Ldistance, duration;

struct Motor 
{
  int IN1;
  int IN2;
  int PWM_CHANNEL;
  int PIN;
};

Motor motors[4];

int IN1Pins[] = {12, 27, 0 ,16}; // Define IN1 pins for motors
int IN2Pins[] = {14, 26, 4, 17}; // Define IN2 pins for motors
int PWM_Pins[] = {13, 25, 15, 5}; // Define PWM pins for motors

int rightMotors[] = {0, 1}; // Indices for right motors
int leftMotors[] = {2, 3}; // Indices for left motors

void setServoAngle(int angle)  // Function to set servo angle 
{
  // Ensure angle is within valid range to eliminate jitter
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  int minUs = 500, maxUs = 2500; // Typical servo pulse width range
  int pulseWidth = map(angle, 0, 180, minUs, maxUs); // Map angle to pulse width
  int duty = (int)((pulseWidth / 20000.0) * 65535); // Convert to duty cycle (0-65535 for 16 bits)

  ledcWrite(SERVO_CHANNEL, duty); // Write duty cycle to servo channel
}

void setup() 
{
  Serial.begin(115100); // Initialize serial communication

  for (int i = 0; i < 4; i++) // Initialize motors
  {
    motors[i].IN1 = IN1Pins[i]; // Assign IN1 pins
    motors[i].IN2 = IN2Pins[i]; // Assign IN2 pins
    motors[i].PWM_CHANNEL = i; // Assign PWM channels
    motors[i].PIN = PWM_Pins[i]; // Assign PWM pins
  }
  
  for (int i = 0; i < 4; i++) // Set pin modes for motors
  {
    pinMode(motors[i].IN1, OUTPUT); // Set IN1 pin as output
    pinMode(motors[i].IN2, OUTPUT); // Set IN2 pin as output
    ledcSetup(motors[i].PWM_CHANNEL, PWM_FREQ, PWM_RES); // Setup PWM for each motor
    ledcAttachPin(motors[i].PIN, motors[i].PWM_CHANNEL); // Attach PWM pin to channel
  }

  /////////////////// Ultrasonic sensor setup ///////////////////

  pinMode(US_trig, OUTPUT);
  pinMode(US_echo, INPUT);

  /////////////////// Servo setup /////////////////////

  ledcSetup(SERVO_CHANNEL, 50, 16); // 50 Hz, 16-bit resolution
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL); // Attach servo pin to channel
  setServoAngle(85); // Set initial angle to 85 degrees

  ///////////////// IR Sensors setup ///////////////////

  pinMode(R_IR, INPUT); // Right IR sensor
  pinMode(L_IR, INPUT); // Left IR sensor

/*
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("AMR Startup");
  delay(1000);
  lcd.clear();
*/
}

void setMotors(int i, int speed) // Function to set motor speed
{
  if (speed > 0) // If speed is positive, Spin motors forward
  {
    digitalWrite(motors[i].IN1, LOW);
    digitalWrite(motors[i].IN2, HIGH);
    ledcWrite(motors[i].PWM_CHANNEL, abs(speed));
  }

  else if (speed < 0) // If speed is negative, spin motors backward
  {
    digitalWrite(motors[i].IN1, HIGH);
    digitalWrite(motors[i].IN2, LOW);
    ledcWrite(motors[i].PWM_CHANNEL, abs(speed));
  }

  else // If speed is zero, stop motors
  {
    digitalWrite(motors[i].IN1, HIGH);
    digitalWrite(motors[i].IN2, HIGH);
    ledcWrite(motors[i].PWM_CHANNEL, 0);
  }
}

void Forward(int speed) // Function to move forward
{
  for (int i = 0; i < 2; i++)
  {
    setMotors(leftMotors[i], speed);
    setMotors(rightMotors[i], speed);
  }

  Serial.print("Moving Forward\n");
} 

void Right(int speed) // Function to Spin right
{
  for (int i = 0; i < 2; i++)
  {
    setMotors(leftMotors[i], speed);
    setMotors(rightMotors[i], - speed);
  }

  Serial.print("Moving right\n");
/*
  lcd.setCursor(0,0);
  lcd.print("Moving right     ");
*/
}

void Left(int speed) // Function to Spin left
{
  for (int i = 0; i < 2; i++)
  {
    setMotors(leftMotors[i], - speed);
    setMotors(rightMotors[i], speed);
  }

  Serial.print("Moving left\n");
/*  
  lcd.setCursor(0,0);
  lcd.print("Moving left      ");
*/
}

void Backward(int speed) // Function to move backward         
{
  for (int i = 0; i < 2; i++)
  {
    setMotors(leftMotors[i], - speed);
    setMotors(rightMotors[i], - speed);
  }

  Serial.print("Moving Backward\n");
/*
  lcd.setCursor(0,0);
  lcd.print("Moving Backward ");
*/
}

void Stop() // Function to stop motors
{
  for (int i = 0; i < 2; i++)
  {
    setMotors(leftMotors[i], 0);
    setMotors(rightMotors[i], 0);
  }
/*
  Serial.print("Stopping\n");
  lcd.setCursor(0,1);
  lcd.print("Stopping            ");
*/
}

void turnRight() // Function to turn right
{
  Backward(140); // Move backward
  delay(500);

  Stop(); // Stop 
  delay(500);

  Right(200); // Turn right
  delay(700);
    
  Stop(); // Stop after turning
  delay(500);
}

void turnLeft() // Function to turn left
{
  Backward(140); // Move backward
  delay(500);

  Stop(); // Stop 
  delay(500);

  Left(200); // Turn left
  delay(700);
  
  Stop(); // Stop after turning
  delay(500);
}

void turn180degrees() // Function to turn 180 degrees
{
  Backward(140); // Move backward
  delay(750);

  Stop(); // Stop 
  delay(500);

  Left(200); // Turn left
  delay(1500);
  
  Stop(); // Stop after turning
  delay(500);
}

float checkDistance() // Function to check distance using ultrasonic sensor
{
  digitalWrite(US_trig, LOW); // Reset trigger pin
  delayMicroseconds(2);

  digitalWrite(US_trig, HIGH); // Send a 10 microsecond pulse to trigger the ultrasonic sensor
  delayMicroseconds(10);

  digitalWrite(US_trig, LOW);  // Reset trigger pin again

  duration = pulseIn(US_echo, HIGH); // Read the echo time

  distance = duration / 58.2; // Calculate distance in cm

  return distance; // Return the calculated distance
}

void scanning()
{
  setServoAngle(0); // Look right
  delay(1000);

  Rdistance = checkDistance(); // Check right distance and store it in Rdistance variable
  Serial.print("Right Distance: ");
  Serial.println(Rdistance);
/*
    lcd.setCursor(0,0);
    lcd.print("R:");
    lcd.print(Rdistance, 0);
    lcd.print("cm          ");
*/
  delay(500);

  setServoAngle(160); // Look left
  delay(1000);

  Ldistance = checkDistance(); // Checks left distance and stores it in Ldistance variable
  Serial.print("Left Distance: ");
  Serial.println(Ldistance);
/*
    lcd.setCursor(0,1);
    lcd.print("L:");
    lcd.print(Ldistance, 0);
    lcd.print("cm          ");
*/
  delay(500);

  setServoAngle(85); // Center the servo
  delay(1000);
}

void checkBlindSpots() // Function to check blind spots using IR sensors
{
  if (digitalRead(R_IR) == LOW) // Right IR sensor detects an obstacle
    {
      while (digitalRead(R_IR) == LOW) // Wait until the right IR sensor is clear
      {
        Left(170); // Spin left until the right IR sensor is clear
      }
      Serial.print("Right IR sensor triggered.\n");
    }

    else if (digitalRead(L_IR) == LOW) // Left IR sensor detects an obstacle
    {
      while (digitalRead(L_IR) == LOW) // Wait until the left IR sensor is clear
      {
        Right(170); // Spin right until the left IR sensor is clear
      }
      Serial.print("Left IR sensor triggered.\n");
    }
}



void loop()
{
  Forward(140); // Move forward by default

  checkDistance(); // Measure distance
  Serial.print("Distance: ");
  Serial.println(distance);
/*
  lcd.setCursor(0,0);
  lcd.print("F:");
  lcd.print(distance, 0);
  lcd.print("cm          ");
*/

  if (distance > 40 && distance != 0) // Check if distance is greater than 40cm and not zero
  {
    Forward(140); // Move forward if distance is clear
    checkBlindSpots(); // Check for obstacles in UltraSonics blind spots (the corners of the robot)
  }

  else // If distance is less than or equal to 40cm, stop and check surroundings
  {
    Stop(); // Stop the robot
    delay(500);

    scanning(); // Scan surroundings

    if (Rdistance > Ldistance && abs(Rdistance - Ldistance) > 10) // If right distance is greater than left distance, right side is cleaer, move right
    {
      Serial.print("Turning right\n");
/* 
      lcd.setCursor(0,0);
      lcd.print("Turning right    ");
*/
      turnRight(); // Turn right
    }

    else if (Ldistance > Rdistance && abs(Rdistance - Ldistance) > 10) // If left distance is greater than right distance, left side is clearer, move left
    {
      Serial.print("Turning left\n");
/*      
      lcd.setCursor(0,0);
      lcd.print("Turning left    ");
*/
      turnLeft(); // Turn left
    }

    else if ( abs(Rdistance - Ldistance) < 10 || Rdistance == Ldistance ) // If both distances are similar or equal, no clear path
    {
      Serial.print("No clear path, moving backward and turning around\n");
/*
      lcd.setCursor(0,0);
      lcd.print("No clear path,   ");
      lcd.setCursor(0,1);
      lcd.print("turning around   ");
*/
      turn180degrees();
    }
  }

    delay(100);
/*    
    lcd.setCursor(0,1);
    lcd.print("Loop reset...     ");
*/    
}
