/*
 * ============================================================================
 * Arduino Nano — Dual Motor + Encoder Controller for ROS2 Differential Drive
 * ============================================================================
 *
 * Hardware:
 *   - 2x DC motors via L298N motor driver
 *   - 2x Quadrature encoders (channels A & B each)
 *   - Arduino Nano connected to Orange Pi 5 via USB serial
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
 * Wiring (adjust pin numbers to match your setup):
 *   L298N Motor Driver 1 (Left Motor):
 *     ENA → Pin 5  (PWM)
 *     IN1 → Pin 7
 *     IN2 → Pin 8
 *
 *   L298N Motor Driver 2 (Right Motor):
 *     ENB → Pin 6  (PWM)
 *     IN3 → Pin 9
 *     IN4 → Pin 10
 *
 *   Left Encoder:
 *     Channel A → Pin 2  (interrupt)
 *     Channel B → Pin 4
 *
 *   Right Encoder:
 *     Channel A → Pin 3  (interrupt)
 *     Channel B → Pin 12
 *
 * ============================================================================
 */

// ========================== PIN DEFINITIONS ==========================

// Left Motor (L298N)
#define LEFT_ENA   5    // PWM speed control
#define LEFT_IN1   7    // Direction
#define LEFT_IN2   8    // Direction

// Right Motor (L298N)
#define RIGHT_ENB  6    // PWM speed control
#define RIGHT_IN3  9    // Direction
#define RIGHT_IN4  10   // Direction

// Left Encoder
#define LEFT_ENC_A   2  // Interrupt pin
#define LEFT_ENC_B   4

// Right Encoder
#define RIGHT_ENC_A  3  // Interrupt pin
#define RIGHT_ENC_B  12

// ========================== CONFIGURATION ==========================

#define SERIAL_BAUD       115200
#define PUBLISH_INTERVAL_MS  50   // Send encoder data every 50ms (20 Hz)
#define CMD_TIMEOUT_MS     500    // Stop motors if no command received for 500ms
#define SERIAL_BUF_SIZE     64

// ========================== GLOBAL STATE ==========================

// Encoder tick counts (volatile — modified in ISR)
volatile long left_ticks  = 0;
volatile long right_ticks = 0;

// Timing
unsigned long last_publish_time = 0;
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
    setMotors(0, 0);
}

// ========================== SERIAL COMMAND PARSING ==========================

void processCommand(const char* cmd) {
    if (cmd[0] == 'm') {
        // Motor command: m <left_pwm> <right_pwm>
        int left_pwm = 0, right_pwm = 0;
        if (sscanf(cmd + 1, "%d %d", &left_pwm, &right_pwm) == 2) {
            left_pwm  = constrain(left_pwm,  -255, 255);
            right_pwm = constrain(right_pwm, -255, 255);
            setMotors(left_pwm, right_pwm);
            last_cmd_time = millis();
        }
    } else if (cmd[0] == 'r') {
        // Reset encoders
        noInterrupts();
        left_ticks  = 0;
        right_ticks = 0;
        interrupts();
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

    Serial.println("Arduino motor controller ready");
}

void loop() {
    // Process incoming serial commands (non-blocking)
    readSerial();

    // Safety: stop motors if no command received recently
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
        stopMotors();
    }

    // Publish encoder ticks at fixed interval
    if (millis() - last_publish_time >= PUBLISH_INTERVAL_MS) {
        publishEncoders();
        last_publish_time = millis();
    }
}
