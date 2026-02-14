/*
 * ============================================================================
 * Arduino Uno — Dual Motor + Encoder Controller for ROS2 Differential Drive
 * ============================================================================
 *
 * Motors: JGB37-520 DC12V 110RPM with Hall Encoder (11 PPR)
 *   - Gear ratio: ~90:1
 *   - Encoder resolution: 11 pulses/motor-rev × 90 = 990 ticks/output-rev
 *   - Wheels: 65mm diameter (0.0325m radius)
 *   - Max speed: ~0.37 m/s at 12V no-load
 *
 * Motor 6-Pin Connector (as labelled on connector):
 *   M1  = Motor terminal + (Red wire)    → connect to L298N OUT
 *   GND = Encoder ground  (Black wire)   → connect to Arduino GND
 *   C2  = Encoder signal  (Yellow wire)  → connect to Arduino digital pin
 *   C1  = Encoder signal  (Green wire)   → connect to Arduino interrupt pin
 *   Vcc = Encoder power   (Blue wire)    → connect to Arduino 5V
 *   M2  = Motor terminal - (White wire)  → connect to L298N OUT
 *
 * Hardware:
 *   - 2x JGB37-520 DC12V 110RPM geared motors with Hall encoders
 *   - 1x L298N dual H-bridge motor driver
 *   - Arduino Uno connected to Orange Pi 5 via USB serial
 *
 * Serial Protocol (115200 baud):
 *   Commands FROM ROS2 (Orange Pi → Arduino):
 *     m <left_pwm> <right_pwm>\n
 *       left_pwm, right_pwm: integer -255 to 255
 *       Positive = forward, Negative = backward
 *
 *   Feedback TO ROS2 (Arduino → Orange Pi):
 *     e <left_ticks> <right_ticks>\n
 *       Sent every PUBLISH_INTERVAL_MS milliseconds
 *       Ticks are cumulative signed encoder counts
 *
 *   Reset command:
 *     r\n
 *       Resets encoder counts to zero
 *
 * Wiring:
 *   L298N Motor Driver → Left Motor (JGB37-520):
 *     ENA → Arduino Pin 5  (PWM speed)
 *     IN1 → Arduino Pin 7  (direction)
 *     IN2 → Arduino Pin 8  (direction)
 *     OUT1 → Left motor M1
 *     OUT2 → Left motor M2
 *
 *   L298N Motor Driver → Right Motor (JGB37-520):
 *     ENB → Arduino Pin 6  (PWM speed)
 *     IN3 → Arduino Pin 9  (direction)
 *     IN4 → Arduino Pin 10 (direction)
 *     OUT3 → Right motor M1
 *     OUT4 → Right motor M2
 *
 *   Left Motor Encoder (from 6-pin connector):
 *     C1  → Arduino Pin 2   (interrupt INT0)
 *     C2  → Arduino Pin 4   (direction sensing)
 *     Vcc → Arduino 5V
 *     GND → Arduino GND
 *
 *   Right Motor Encoder (from 6-pin connector):
 *     C1  → Arduino Pin 3   (interrupt INT1)
 *     C2  → Arduino Pin 12  (direction sensing)
 *     Vcc → Arduino 5V
 *     GND → Arduino GND
 *
 *   NOTE: If a wheel counts in the wrong direction, swap C1 and C2
 *         for that motor. If a motor spins the wrong way, swap M1 and M2.
 *
 * ============================================================================
 */

// ========================== PIN DEFINITIONS ==========================

// Left Motor (L298N → JGB37-520)
#define LEFT_ENA   5    // PWM speed control (L298N ENA)
#define LEFT_IN1   7    // Direction (L298N IN1)
#define LEFT_IN2   8    // Direction (L298N IN2)

// Right Motor (L298N → JGB37-520)
#define RIGHT_ENB  6    // PWM speed control (L298N ENB)
#define RIGHT_IN3  9    // Direction (L298N IN3)
#define RIGHT_IN4  10   // Direction (L298N IN4)

// Left Encoder (JGB37-520 Hall encoder, 11 PPR × 90:1 gear = 990 ticks/rev)
#define LEFT_ENC_A   2  // C1 pin on motor connector → Interrupt INT0
#define LEFT_ENC_B   4  // C2 pin on motor connector → Direction sensing

// Right Encoder (JGB37-520 Hall encoder, 11 PPR × 90:1 gear = 990 ticks/rev)
#define RIGHT_ENC_A  3  // C1 pin on motor connector → Interrupt INT1
#define RIGHT_ENC_B  12 // C2 pin on motor connector → Direction sensing

// ========================== CONFIGURATION ==========================

#define SERIAL_BAUD       115200
#define PUBLISH_INTERVAL_MS  50   // Send encoder data every 50ms (20 Hz)
#define PID_INTERVAL_MS      50   // PID update rate (same as publish)
#define CMD_TIMEOUT_MS     500    // Stop motors if no command received for 500ms
#define SERIAL_BUF_SIZE     64

// ========================== PID SPEED CONTROL ==========================
// PID controller uses encoder feedback to equalize wheel speeds.
// Without PID, one motor often runs faster than the other due to
// manufacturing differences — this is normal for DC motors.
//
// The incoming 'm' command (-255 to 255) is treated as a target speed.
// PID measures actual speed from encoder ticks and adjusts PWM to match.
//
// At 110 RPM: 990 ticks/rev × (110/60) rev/s × 0.05s = ~90 ticks per interval
#define MAX_TICKS_PER_INTERVAL  91

// PID gains — conservative defaults, tune if needed
// Kp: proportional (main correction force)
// Ki: integral (eliminates steady-state error / motor difference)
// Kd: derivative (dampens oscillation)
#define KP   2.0
#define KI   1.0
#define KD   0.1
#define INTEGRAL_LIMIT  80.0    // Anti-windup clamp

// Per-motor PID state
struct MotorPID {
    float target;          // target ticks per interval (-91 to +91)
    float integral;        // accumulated error
    float prev_error;      // previous error for derivative
    int   output_pwm;      // actual PWM being applied to motor
};

MotorPID left_pid  = {0, 0, 0, 0};
MotorPID right_pid = {0, 0, 0, 0};

// Previous tick values for computing speed (ticks per interval)
long prev_left_ticks  = 0;
long prev_right_ticks = 0;

// ========================== GLOBAL STATE ==========================

// Encoder tick counts (volatile — modified in ISR)
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// Timing
unsigned long last_publish_time = 0;
unsigned long last_pid_time     = 0;
unsigned long last_cmd_time     = 0;

// Serial input buffer
char serial_buf[SERIAL_BUF_SIZE];
int  serial_idx = 0;

// ========================== ENCODER ISRs ==========================

void leftEncoderISR() {
    if (digitalRead(LEFT_ENC_B) == HIGH) {
        left_ticks++;
    } else {
        left_ticks--;
    }
}

void rightEncoderISR() {
    if (digitalRead(RIGHT_ENC_B) == HIGH) {
        right_ticks++;
    } else {
        right_ticks--;
    }
}

// ========================== MOTOR CONTROL ==========================

void setMotor(int pwm, int ena_pin, int in1_pin, int in2_pin) {
    if (pwm > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(ena_pin, constrain(pwm, 0, 255));
    } else if (pwm < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(ena_pin, constrain(-pwm, 0, 255));
    } else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        analogWrite(ena_pin, 0);
    }
}

void setMotors(int left_pwm, int right_pwm) {
    setMotor(left_pwm,  LEFT_ENA,  LEFT_IN1,  LEFT_IN2);
    setMotor(right_pwm, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);
}

void stopMotors() {
    left_pid.target = 0;
    left_pid.integral = 0;
    left_pid.prev_error = 0;
    left_pid.output_pwm = 0;
    right_pid.target = 0;
    right_pid.integral = 0;
    right_pid.prev_error = 0;
    right_pid.output_pwm = 0;
    setMotors(0, 0);
}

// ========================== PID UPDATE ==========================

void updateMotorPID(MotorPID &pid, long delta_ticks, int ena_pin, int in1_pin, int in2_pin) {
    // If target is zero, stop immediately and reset PID state
    if (pid.target > -0.5f && pid.target < 0.5f) {
        pid.output_pwm = 0;
        pid.integral = 0;
        pid.prev_error = 0;
        setMotor(0, ena_pin, in1_pin, in2_pin);
        return;
    }

    // Compute error: difference between target and actual speed
    float actual = (float)delta_ticks;
    float error = pid.target - actual;

    // Integral with anti-windup
    pid.integral += error;
    if (pid.integral > INTEGRAL_LIMIT)  pid.integral = INTEGRAL_LIMIT;
    if (pid.integral < -INTEGRAL_LIMIT) pid.integral = -INTEGRAL_LIMIT;

    // Derivative
    float derivative = error - pid.prev_error;
    pid.prev_error = error;

    // Feedforward: estimate PWM from target speed (gets us close)
    int feedforward = (int)((pid.target / (float)MAX_TICKS_PER_INTERVAL) * 255.0f);

    // PID correction on top of feedforward
    float correction = KP * error + KI * pid.integral + KD * derivative;

    pid.output_pwm = feedforward + (int)correction;
    if (pid.output_pwm > 255)  pid.output_pwm = 255;
    if (pid.output_pwm < -255) pid.output_pwm = -255;

    setMotor(pid.output_pwm, ena_pin, in1_pin, in2_pin);
}

void updatePID() {
    // Read current encoder values
    noInterrupts();
    long l = left_ticks;
    long r = right_ticks;
    interrupts();

    // Compute ticks in this interval (= speed)
    long delta_left  = l - prev_left_ticks;
    long delta_right = r - prev_right_ticks;
    prev_left_ticks  = l;
    prev_right_ticks = r;

    // Update PID for each motor
    updateMotorPID(left_pid,  delta_left,  LEFT_ENA,  LEFT_IN1,  LEFT_IN2);
    updateMotorPID(right_pid, delta_right, RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);
}

// ========================== SERIAL COMMAND PARSING ==========================

void processCommand(const char* cmd) {
    if (cmd[0] == 'm') {
        // Motor command: m <left_pwm> <right_pwm>
        // Values -255 to 255 are treated as target speed (not raw PWM)
        // PID controller will adjust actual PWM to match target speed
        int left_pwm = 0, right_pwm = 0;
        if (sscanf(cmd + 1, "%d %d", &left_pwm, &right_pwm) == 2) {
            left_pwm  = constrain(left_pwm,  -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            // Convert command to target ticks per interval
            left_pid.target  = (left_pwm  / 255.0f) * MAX_TICKS_PER_INTERVAL;
            right_pid.target = (right_pwm / 255.0f) * MAX_TICKS_PER_INTERVAL;
            last_cmd_time = millis();
        }
    } else if (cmd[0] == 'r') {
        // Reset encoders
        noInterrupts();
        left_ticks  = 0;
        right_ticks = 0;
        interrupts();
        prev_left_ticks  = 0;
        prev_right_ticks = 0;
        left_pid.integral = 0;
        left_pid.prev_error = 0;
        right_pid.integral = 0;
        right_pid.prev_error = 0;
        Serial.println("r ok");
    }
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_idx > 0) {
                serial_buf[serial_idx] = '\0';
                processCommand(serial_buf);
                serial_idx = 0;
            }
        } else if (serial_idx < SERIAL_BUF_SIZE - 1) {
            serial_buf[serial_idx++] = c;
        }
    }
}

// ========================== PUBLISH ENCODER DATA ==========================

void publishEncoders() {
    long l, r;
    noInterrupts();
    l = left_ticks;
    r = right_ticks;
    interrupts();

    Serial.print("e ");
    Serial.print(l);
    Serial.print(" ");
    Serial.println(r);
}

// ========================== SETUP & LOOP ==========================

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Motor pins
    pinMode(LEFT_ENA,  OUTPUT);
    pinMode(LEFT_IN1,  OUTPUT);
    pinMode(LEFT_IN2,  OUTPUT);
    pinMode(RIGHT_ENB, OUTPUT);
    pinMode(RIGHT_IN3, OUTPUT);
    pinMode(RIGHT_IN4, OUTPUT);

    // Encoder pins
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncoderISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

    stopMotors();

    Serial.println("Arduino motor controller ready (PID enabled)");
}

void loop() {
    // Process incoming serial commands (non-blocking)
    readSerial();

    // Safety: stop motors if no command received recently
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
        stopMotors();
    }

    // PID speed control update at fixed interval
    if (millis() - last_pid_time >= PID_INTERVAL_MS) {
        updatePID();
        last_pid_time = millis();
    }

    // Publish encoder ticks at fixed interval
    if (millis() - last_publish_time >= PUBLISH_INTERVAL_MS) {
        publishEncoders();
        last_publish_time = millis();
    }
}
