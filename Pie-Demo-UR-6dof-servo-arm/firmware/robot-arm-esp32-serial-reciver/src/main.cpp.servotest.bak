// Filename: main.cpp.servotest.bak
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define NUM_SERVOS 6  // Number of servos to test

// Initial pulse length values (you can adjust these)
#define DEFAULT_MIN_US 500
#define DEFAULT_MAX_US 2500

// I2C Pin
#define I2C_SDA_PIN   (5)
#define I2C_SCL_PIN   (6)

// Current settings for each servo
struct ServoSettings {
    uint16_t minUs;
    uint16_t maxUs;
    uint16_t currentUs;
    uint8_t channel;
} servoSettings[NUM_SERVOS];

// Convert microseconds to pulse length count
uint16_t usecToPulse(uint16_t usec) {
    return map(usec, 0, 20000, 0, 4096);
}

// Convert angle (0-270 degrees) to microseconds based on servo calibration
uint16_t angleToMicroseconds(int servoIndex, double angle) {
    if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
        return 0;
    }
    
    // Clamp angle to valid range
    angle = constrain(angle, 0.0, 270.0);
    
    // Linear interpolation: map angle (0-180) to pulse width (minUs-maxUs)
    uint16_t minUs = servoSettings[servoIndex].minUs;
    uint16_t maxUs = servoSettings[servoIndex].maxUs;
    // Use floating-point arithmetic for precision
    double micro = minUs + (angle / 270.0) * (double)(maxUs - minUs);
    uint16_t microseconds = (uint16_t)round(micro);
    
    return constrain(microseconds, minUs, maxUs);
}

void printMenu() {
    Serial.println("\n=== Servo Calibration Tool ===");
    Serial.println("Commands:");
    Serial.println("s,<channel>,<microseconds> - Set position in microseconds (e.g., s,0,1500)");
    Serial.println("m,<channel>,<microseconds> - Set min value (e.g., m,0,500)");
    Serial.println("x,<channel>,<microseconds> - Set max value (e.g., x,0,2500)");
    Serial.println("a,<channel>,<angle>        - Set position by angle in degrees (e.g., a,0,90)");
    Serial.println("t,<channel>                - Test full range");
    Serial.println("p                          - Print current settings");
    Serial.println("h                          - Show this help menu");
}

void printSettings() {
    Serial.println("\nCurrent Servo Settings:");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(": Channel=");
        Serial.print(servoSettings[i].channel);
        Serial.print(", Min=");
        Serial.print(servoSettings[i].minUs);
        Serial.print("us, Max=");
        Serial.print(servoSettings[i].maxUs);
        Serial.print("us, Current=");
        Serial.print(servoSettings[i].currentUs);
        Serial.println("us");
    }
}

void testServoRange(int servoIndex) {
    Serial.print("Testing servo ");
    Serial.print(servoIndex);
    Serial.println(" full range...");
    
    // Go to min position
    pwm.writeMicroseconds(servoSettings[servoIndex].channel, servoSettings[servoIndex].minUs);
    Serial.println("At minimum position");
    delay(1000);
    
    // Go to center
    uint16_t centerUs = (servoSettings[servoIndex].maxUs + servoSettings[servoIndex].minUs) / 2;
    pwm.writeMicroseconds(servoSettings[servoIndex].channel, centerUs);
    Serial.println("At center position");
    delay(1000);
    
    // Go to max position
    pwm.writeMicroseconds(servoSettings[servoIndex].channel, servoSettings[servoIndex].maxUs);
    Serial.println("At maximum position");
    delay(1000);
    
    // Return to center
    pwm.writeMicroseconds(servoSettings[servoIndex].channel, centerUs);
    Serial.println("Returned to center");
}

void processCommand(String command) {
    command.trim();
    char cmd = command.charAt(0);
    command = command.substring(2); // Skip command and comma
    
    switch(cmd) {
        case 's': { // Set position
            int channel = command.substring(0, command.indexOf(',')).toInt();
            int us = command.substring(command.indexOf(',') + 1).toInt();
            if (channel >= 0 && channel < NUM_SERVOS) {
                servoSettings[channel].currentUs = constrain(us, servoSettings[channel].minUs, servoSettings[channel].maxUs);
                pwm.writeMicroseconds(servoSettings[channel].channel, servoSettings[channel].currentUs);
                Serial.printf("Set servo %d to %d us\n", channel, servoSettings[channel].currentUs);
            }
            break;
        }
        case 'm': { // Set min
            int channel = command.substring(0, command.indexOf(',')).toInt();
            int us = command.substring(command.indexOf(',') + 1).toInt();
            if (channel >= 0 && channel < NUM_SERVOS) {
                servoSettings[channel].minUs = us;
                Serial.printf("Set servo %d minimum to %d us\n", channel, us);
            }
            break;
        }
        case 'x': { // Set max
            int channel = command.substring(0, command.indexOf(',')).toInt();
            int us = command.substring(command.indexOf(',') + 1).toInt();
            if (channel >= 0 && channel < NUM_SERVOS) {
                servoSettings[channel].maxUs = us;
                Serial.printf("Set servo %d maximum to %d us\n", channel, us);
            }
            break;
        }
        case 'a': { // Set by angle
            int channel = command.substring(0, command.indexOf(',')).toInt();
            double angle = command.substring(command.indexOf(',') + 1).toDouble();
            if (channel >= 0 && channel < NUM_SERVOS) {
                uint16_t us = angleToMicroseconds(channel, angle);
                servoSettings[channel].currentUs = us;
                pwm.writeMicroseconds(servoSettings[channel].channel, us);
                Serial.printf("Set servo %d to %.1f degrees (%d us)\n", channel, angle, us);
            } else {
                Serial.println("Invalid servo channel");
            }
            break;
        }
        case 't': { // Test range
            int channel = command.toInt();
            if (channel >= 0 && channel < NUM_SERVOS) {
                testServoRange(channel);
            }
            break;
        }
        case 'p': // Print settings
            printSettings();
            break;
        case 'h': // Help
            printMenu();
            break;
        default:
            Serial.println("Unknown command. Type 'h' for help.");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C and PCA9685
    // Wire.begin();
    Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN );
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    
    // Initialize servo settings
    for (int i = 0; i < NUM_SERVOS; i++) {
        servoSettings[i].minUs = DEFAULT_MIN_US;
        servoSettings[i].maxUs = DEFAULT_MAX_US;
        servoSettings[i].currentUs = (DEFAULT_MIN_US + DEFAULT_MAX_US) / 2;
        servoSettings[i].channel = i; // Using channels 0-5
        
        // Center all servos
        pwm.writeMicroseconds(servoSettings[i].channel, servoSettings[i].currentUs);
    }
    
    delay(10);
    Serial.println("\nServo Calibration Tool Ready");
    printMenu();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }
}