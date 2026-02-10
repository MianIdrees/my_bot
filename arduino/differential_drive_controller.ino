// Arduino Nano Motor Controller for ROS 2 Differential Drive Robot
// Receives Twist commands via Serial and controls L298N motor driver
// Compatible with Orange Pi 5 + ROS 2 Humble/Jazzy

// ============================================================================
// MOTOR DRIVER PINS (L298N)
// ============================================================================
#define ENA 3   // PWM pin for left motor speed (must be PWM-capable)
#define IN1 4   // Left motor direction 1
#define IN2 5   // Left motor direction 2

#define ENB 6   // PWM pin for right motor speed (must be PWM-capable)
#define IN3 7   // Right motor direction 1
#define IN4 8   // Right motor direction 2

// ============================================================================
// ENCODER PINS (Optional - for odometry feedback)
// NOTE: Arduino Nano only has 2 interrupt pins (D2, D3)
// Current pin assignment uses D3 for motor PWM, which conflicts with encoders
// Options:
// 1. Use encoder library with pin change interrupts (any pin)
// 2. Move motor to different pin and use D2/D3 for encoders
// 3. Read encoders via polling (less accurate)
// ============================================================================
#define LEFT_ENC_A 2   // Interrupt pin (INT0) - OK
#define LEFT_ENC_B 9
#define RIGHT_ENC_A 12  // Changed from D3 to D12 (polling or pin change interrupt)
#define RIGHT_ENC_B 10

// ============================================================================
// ROBOT PHYSICAL PARAMETERS (Must match your URDF)
// ============================================================================
const float WHEEL_SEPARATION = 0.35;  // meters (distance between wheels)
const float WHEEL_RADIUS = 0.05;      // meters
const float MAX_LINEAR_VEL = 0.5;     // m/s (from your robot specs)
const float MAX_ANGULAR_VEL = 1.0;    // rad/s

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================
const float VELOCITY_DEADZONE = 0.01;  // m/s - ignore velocities below this threshold

// ============================================================================
// SAFETY PARAMETERS
// ============================================================================
const unsigned long CMD_TIMEOUT = 1000;  // milliseconds (1 second)
unsigned long lastCmdTime = 0;

// ============================================================================
// MOTOR CONTROL VARIABLES
// ============================================================================
float linearVel = 0.0;   // Current linear velocity (m/s)
float angularVel = 0.0;  // Current angular velocity (rad/s)

// ============================================================================
// ENCODER VARIABLES (Optional)
// ============================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
const int ENCODER_PPR = 360;  // Pulses per revolution (adjust for your encoders)

// ============================================================================
// SETUP - Runs once at startup
// ============================================================================
void setup() {
  // Initialize serial communication (115200 baud for Orange Pi)
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Configure motor driver pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize motors to stopped state
  stopMotors();
  
  // Optional: Setup encoder pins and interrupts
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  // Attach interrupt handlers for encoders
  // NOTE: Only using LEFT encoder on INT0 due to D3 pin conflict with motor PWM
  // For RIGHT encoder, use polling or pin change interrupt library
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);  // Commented - D3 conflict
  
  // Startup message
  Serial.println("Arduino Differential Drive Controller Ready");
  Serial.println("Waiting for Twist commands: L:linear,A:angular");
  
  lastCmdTime = millis();
}

// ============================================================================
// MAIN LOOP - Runs continuously
// ============================================================================
void loop() {
  // Check for incoming serial data from ROS 2
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove whitespace
    
    if (command.length() > 0) {
      parseCommand(command);
      lastCmdTime = millis();
    }
  }
  
  // Safety feature: Stop motors if no command received within timeout
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    if (linearVel != 0.0 || angularVel != 0.0) {
      stopMotors();
      Serial.println("TIMEOUT: Motors stopped");
    }
  }
  
  // Execute motor control based on current velocities
  driveMotors(linearVel, angularVel);
  
  // Optional: Send encoder feedback (uncomment to enable)
  // sendEncoderFeedback();
  
  delay(10);  // 100Hz control loop
}

// ============================================================================
// PARSE COMMAND - Extract velocities from serial message
// ============================================================================
void parseCommand(String cmd) {
  // Expected format: "L:linear_velocity,A:angular_velocity"
  // Example: "L:0.5,A:0.2" means 0.5 m/s forward, 0.2 rad/s counter-clockwise
  
  int linIdx = cmd.indexOf("L:");
  int angIdx = cmd.indexOf("A:");
  
  if (linIdx >= 0 && angIdx >= 0) {
    // Extract linear velocity substring
    String linStr = cmd.substring(linIdx + 2, angIdx);
    linStr.trim();
    linStr.replace(",", "");  // Remove comma if present
    
    // Extract angular velocity substring
    String angStr = cmd.substring(angIdx + 2);
    angStr.trim();
    
    // Convert to float
    float newLinear = linStr.toFloat();
    float newAngular = angStr.toFloat();
    
    // Apply velocity limits (safety)
    newLinear = constrain(newLinear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
    newAngular = constrain(newAngular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    
    linearVel = newLinear;
    angularVel = newAngular;
    
    // Debug output (comment out in production for performance)
    // Serial.print("CMD: L=");
    // Serial.print(linearVel, 3);
    // Serial.print(" A=");
    // Serial.println(angularVel, 3);
  }
}

// ============================================================================
// DRIVE MOTORS - Convert velocities to motor commands
// ============================================================================
void driveMotors(float linear, float angular) {
  // Differential drive kinematics:
  // Left wheel velocity = linear - (angular * wheelbase / 2)
  // Right wheel velocity = linear + (angular * wheelbase / 2)
  
  float leftVel = linear - (angular * WHEEL_SEPARATION / 2.0);
  float rightVel = linear + (angular * WHEEL_SEPARATION / 2.0);
  
  // Convert velocities to PWM values (0-255)
  // Map from [-MAX_LINEAR_VEL, MAX_LINEAR_VEL] to [0, 255]
  int leftPWM = constrain(abs(leftVel) * 255.0 / MAX_LINEAR_VEL, 0, 255);
  int rightPWM = constrain(abs(rightVel) * 255.0 / MAX_LINEAR_VEL, 0, 255);
  
  // Control LEFT motor (Motor A on L298N)
  if (leftVel > VELOCITY_DEADZONE) {  // Forward (small deadzone to avoid jitter)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  } else if (leftVel < -VELOCITY_DEADZONE) {  // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, leftPWM);
  } else {  // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // Control RIGHT motor (Motor B on L298N)
  if (rightVel > VELOCITY_DEADZONE) {  // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightPWM);
  } else if (rightVel < -VELOCITY_DEADZONE) {  // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, rightPWM);
  } else {  // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

// ============================================================================
// STOP MOTORS - Emergency stop
// ============================================================================
void stopMotors() {
  // Set all enables to 0 (no power)
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Set all direction pins low
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // Reset velocity variables
  linearVel = 0.0;
  angularVel = 0.0;
}

// ============================================================================
// ENCODER INTERRUPT HANDLERS (Optional - for odometry)
// ============================================================================
void leftEncoderISR() {
  // Increment/decrement based on direction
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  // Increment/decrement based on direction
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// ============================================================================
// SEND ENCODER FEEDBACK (Optional - for closed-loop control)
// ============================================================================
void sendEncoderFeedback() {
  static unsigned long lastFeedbackTime = 0;
  const unsigned long FEEDBACK_INTERVAL = 50;  // Send every 50ms (20Hz)
  
  if (millis() - lastFeedbackTime > FEEDBACK_INTERVAL) {
    // Format: "E:left_count,right_count"
    Serial.print("E:");
    Serial.print(leftEncoderCount);
    Serial.print(",");
    Serial.println(rightEncoderCount);
    
    lastFeedbackTime = millis();
  }
}

// ============================================================================
// END OF CODE
// ============================================================================
