// ****************************************************************************
// Title        : Lab05_Motor_Controller
// Author       : Logan Frosch
// File Name    : 'main.cpp'
// Target MCU   : Espressif ESP32 (Doit DevKit Version 1)
//
// EGRT 390     : Example of basic DC motor control using L298N driver
//
// Revision History:
// When        Who        Description of change
// ****************************************************************************
// 2025-4-7    L.Frosch     Initial version

// Include Files
// ****************************************************************************
#include <Arduino.h>

// Add ProjectileMotion class
class ProjectileMotion {
private:
    const double g = -9.81;  // gravity in m/s^2
    
    // Wheel characteristics (commented out until motor is loaded)
    /*
    const double WHEEL_RADIUS = 0.1016;  // wheel radius in meters
    const double WHEEL_MASS = 0.603;  // 1.33 pounds converted to kg
    const double WHEEL_DIAMETER = 2 * WHEEL_RADIUS;  // 0.2032 meters
    const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
    const double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS * WHEEL_RADIUS * WHEEL_RADIUS;  // for solid disk
    */
    const double WHEEL_CIRCUMFERENCE = 0.6305;  //Unloaded test circumference
    
    double x0, y0, v0, angle = -1;  // Initialize angle to invalid value
    double v0x, v0y, tFinal;
    double targetX, targetY;
    const double TOLERANCE = 0.1;

public:
    double velocityToRPM(double velocity) const {
        return (velocity * 60.0) / WHEEL_CIRCUMFERENCE;
    }

    double getRange() const {
        return targetX - x0;
    }

    double getMaxHeight() const {
        return y0 + (v0y * v0y) / (2 * -g);
    }

    double getAngle() const {
        return angle;
    }

    double getVelocity() const {
        return v0;
    }

    double getTargetX() const {
        return targetX;
    }

    double getTargetY() const {
        return targetY;
    }
    
    void setAngle(double newAngle) {
        // Add minimum angle check based on target position
        double minAngle = atan2(targetY - y0, targetX - x0) * 180.0 / M_PI;
        if (newAngle < minAngle) {
            Serial.println(F("Error: Angle too shallow to reach target"));
            return;
        }
        
        angle = newAngle;
        findRequiredVelocity();
    }
    
public:
    ProjectileMotion(double initial_x, double initial_y, double target_x, double target_y) 
        : x0(initial_x), y0(initial_y), targetX(target_x), targetY(target_y) {
        // Don't set initial angle or calculate velocity
        // Wait for setAngle() call
    }
    
    void findRequiredVelocity() {
        if (angle < 0) {
            Serial.println(F("Error: Angle not set"));
            return;
        }
        
        double dx = targetX - x0;
        double dy = targetY - y0;
        double theta = angle * M_PI / 180.0;  // Convert angle to radians
        
        // Using quadratic projectile motion equation
        double num = g * dx * dx;
        double den = 2 * cos(theta) * cos(theta) * (dx * tan(theta) - dy);
        
        if (den == 0) {
            Serial.println(F("Error: Invalid trajectory parameters"));
            return;
        }
        
        v0 = sqrt(abs(num/den));
        
        // Calculate components
        v0x = v0 * cos(theta);
        v0y = v0 * sin(theta);
        
        // Verify the trajectory will reach the target
        double timeOfFlight = dx / v0x;
        double finalHeight = y0 + v0y * timeOfFlight + 0.5 * g * timeOfFlight * timeOfFlight;
        
        if (abs(finalHeight - targetY) > TOLERANCE) {
            Serial.println(F("Warning: Trajectory may not reach target height"));
        }
        
        tFinal = timeOfFlight;
    }

    bool calculateAngleFromRPM(double desiredRPM) {
        // Convert RPM to velocity in m/s
        double velocity = (desiredRPM * WHEEL_CIRCUMFERENCE) / 60.0;
        
        // Calculate possible angles using quadratic formula
        double dx = targetX - x0;
        double dy = targetY - y0;
        double v2 = velocity * velocity;
        double g = -9.81;
        
        // From projectile motion equations:
        // tan(θ) = (v²±√(v⁴-g(gx²+2yv²)))/gx
        double underSqrt = v2*v2 - g*(g*dx*dx + 2*dy*v2);
        
        if (underSqrt < 0) {
            Serial.println(F("Error: Target unreachable at this RPM"));
            return false;
        }
        
        // Calculate two possible angles
        double theta1 = atan((v2 + sqrt(underSqrt))/(g*dx));
        double theta2 = atan((v2 - sqrt(underSqrt))/(g*dx));
        
        // Convert to degrees
        double angle1 = theta1 * 180.0 / M_PI;
        double angle2 = theta2 * 180.0 / M_PI;
        
        // Choose the smaller angle if both are valid
        if (angle1 >= 0 && angle1 <= 90) {
            angle = angle1;
        } else if (angle2 >= 0 && angle2 <= 90) {
            angle = angle2;
        } else {
            Serial.println(F("Error: No valid angle found"));
            return false;
        }
        
        // Calculate initial velocity components
        double theta = angle * M_PI / 180.0;
        v0 = velocity;
        v0x = v0 * cos(theta);
        v0y = v0 * sin(theta);
        
        return true;
    }
    // ...rest of ProjectileMotion class methods...
};

// Global constants
const int RPM_ARRAY_SIZE = 5;  // Size of the rolling average window for both motors

// Add these constants after other global constants
const int FINS_PER_DISK = 2;  // Number of fins per disk
const float DEGREES_PER_FIN = 360.0 / FINS_PER_DISK;  // Degrees between fins
const float ALIGNMENT_TOLERANCE = 20.0;  // Degrees of acceptable misalignment

// Add global ProjectileMotion object
ProjectileMotion *projectile = nullptr;

// Button debouncing variables
unsigned long lastDebounceTime = 0;

// Motor 1 variables
volatile long encoderPosition = 0;
long lastPosition = 0;
float motorRPM = 0.0;
unsigned long lastSpeedCalc = 0;
float currentSpeedPercent = 0.0;
bool isStable = false;
unsigned long stableStartTime = 0;
unsigned long lastAdjustTime = 0;
float rpmArray[RPM_ARRAY_SIZE];
int rpmIndex = 0;
bool rpmArrayFull = false;

// Motor 2 variables (matching Motor 1's structure)
volatile long encoderPositionB = 0;
long lastPositionB = 0;
float motorRPM_B = 0.0;
unsigned long lastSpeedCalcB = 0;
float currentSpeedPercent2 = 0.0;
bool isMotor2Stable = false;
unsigned long stableStartTime2 = 0;
unsigned long lastAdjustTimeB = 0;
float rpmArrayB[RPM_ARRAY_SIZE];
int rpmIndexB = 0;
bool rpmArrayFullB = false;

// Add these variables with other motor variables
float disk1Position = 0.0;  // Absolute position of disk 1 in degrees
float disk2Position = 0.0;  // Absolute position of disk 2 in degrees
bool disksAligned = false;  // Flag for disk alignment status

// Add these globals after other motor variables
const int ALIGNED_COUNT_THRESHOLD = 3;  // Number of consecutive alignments needed
int consecutiveAlignments = 0;         // Counter for consecutive alignments
bool motor3Running = false;            // Track motor 3 state

// Global control variables (shared)
bool motorRunning = false;
bool motor2Running = false;
float targetRPM = 0.0;
float targetRPM2 = 0.0;

// Add these constants for motor 2's separate adjustment timing
const unsigned long ADJUST_INTERVAL_B = 500;  // Faster base interval for motor 2

// Function Prototypes
void IRAM_ATTR handleEncoder();          // Interrupt handler for encoder state changes
void IRAM_ATTR handleEncoder2();         // Interrupt handler for second encoder state changes
void calculateSpeedMotor1();             // Calculate motor 1 speed in RPM
void calculateSpeedMotor2();             // Calculate motor 2 speed in RPM
float convertToDegrees(long position);   // Convert encoder position to degrees
float calculateAverageRPM(float newRPM); // Calculate rolling average of RPM values
float calculateAverageRPM_B(float newRPM); // Calculate rolling average of RPM values for motor 2
void displayHelp();                      // Display help information for motor control commands
void displayInitialVelocity();           // Display initial velocity calculation
bool checkDiskAlignment();               // Check alignment of disks

// Globals
// ****************************************************************************
// Heartbeat variables
const uint8_t LED = LED_BUILTIN;      // Pin number connected to LED
const uint16_t BLINK_INTERVAL = 1000; // Blink on/off time in milliseconds
bool ledState = true;                 // Default state
unsigned long ledBlinkTime = 0;       // Time of last LED blink

// Motor control pins
const uint8_t IN1 = 25; // L298N input 1
const uint8_t IN2 = 26; // L298N input 2
const uint8_t ENA = 14; // L298N enable A (PWM)

// Add second motor pins (choose output-capable pins)
const uint8_t IN3 = 32;    // L298N input 3 (changed from 34)
const uint8_t IN4 = 27;    // L298N input 4 (changed from 35)
const uint8_t ENB = 33;    // L298N enable B
const uint8_t ENCODER_B = 18;  // Changed from 12 to 18

// Add third motor pins after existing motor pin definitions
const uint8_t IN5 = 22;    // L298N input 5
const uint8_t IN6 = 21;    // L298N input 6
const uint8_t ENC = 23;    // L298N enable C 

// PWM configurations
const uint8_t PWM_CHANNEL = 0;    // PWM channel for motor control
const uint16_t PWM_FREQ = 1800;   // PWM frequency (10Hz-40MHz)
const uint8_t PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

// Add second motor PWM channel
const uint8_t PWM_CHANNEL_B = 1;  // Different channel than first motor

// Add third motor PWM channel after existing PWM definitions
const uint8_t PWM_CHANNEL_C = 2;  // Different channel than first and second motors

// Command processing variables
String command = "";          // To store the complete command
bool commandComplete = false; // Flag to indicate command is complete

// Encoder configuration
const uint8_t ENCODER_A = 13;      // Encoder A channel (single channel)
// const uint8_t ENCODER_B = 12;   // Not used with single channel encoder
const float PULSES_PER_REV = 12.0;  // Update if this differs from your encoder
const float GEAR_RATIO = 1.0;       // Update if using a gearbox

volatile unsigned long lastEncoderTime = 0; // Last encoder update time

unsigned long eStopRPM = 0.0;             // Emergency stop RPM

// Add a global variable to track motor direction
bool isForward = true; // true for forward, false for reverse

// Keep only these essential motor control constants
const float RPM_TO_PWM_RATIO = 0.02;  // Conversion ratio from RPM to PWM percentage
const float MIN_PWM = 180.0;  // Minimum PWM to keep motor running
const float MAX_PWM = 255.0;  // Maximum PWM value
const float MAX_RPM = 10000.0;  // Maximum motor RPM

// Keep RPM printing constants
const unsigned long RPM_PRINT_INTERVAL = 250;  // 250ms between RPM prints
unsigned long lastRpmPrint = 0;  // Track last RPM print time

// Update these constants for stabilization
const float RPM_TOLERANCE = 5.0;         // Reduced from 10.0 RPM deviation
const unsigned long STABLE_TIME = 1000;    // Changed from 4000 to 2000ms (2 seconds stability requirement)
const float ADJUSTMENT_RATE = 0.05;        // Small adjustment rate for smooth control

// Add these new variables with other globals
bool adjustmentCooldown = false;

// Update the adjustment interval constant
const unsigned long ADJUST_INTERVAL = 500;  // Adjust every 500ms (half second)

// Update these constants for even faster adjustments
const float VERY_COARSE_ADJUSTMENT = 4;    // Reduced from 4.0
const float COARSE_ADJUSTMENT = 2.5;         // Reduced from 2.5
const float MEDIUM_ADJUSTMENT = 1.5;        // Reduced from 1.5
const float FINE_ADJUSTMENT = 0.5;          // Reduced from 0.75
const float ULTRA_FINE_ADJUSTMENT = 0.1;     // Reduced from 0.2

const unsigned long VERY_COARSE_ADJUST_INTERVAL = 20;   // Reduced from 25ms
const unsigned long COARSE_ADJUST_INTERVAL = 25;        // Reduced from 35ms
const unsigned long MEDIUM_ADJUST_INTERVAL = 35;        // Reduced from 50ms
const unsigned long FINE_ADJUST_INTERVAL = 50;         // Reduced from 100ms
const unsigned long ULTRA_FINE_ADJUST_INTERVAL = 125;   // Reduced from 125ms

const unsigned long SPEED_INTERVAL = 100;          // Interval for speed calculations in milliseconds

// Convert encoder position to degrees
float convertToDegrees(long position)
{
  return (float)position * (360.0 / PULSES_PER_REV); // Convert to degrees
}

// Calculate rolling average of RPM values
float calculateAverageRPM(float newRPM)
{
  rpmArray[rpmIndex] = newRPM;                // Store new RPM value in the array
  rpmIndex = (rpmIndex + 1) % RPM_ARRAY_SIZE; // Update index
  if (rpmIndex == 0)
  {
    rpmArrayFull = true; // Set flag if array is full
  }

  float sum = 0.0;                                      // Initialize sum for averaging
  int count = rpmArrayFull ? RPM_ARRAY_SIZE : rpmIndex; // Count of valid RPM values
  if (count == 0)
    return 0.0; // Avoid division by zero

  for (int i = 0; i < count; i++)
  {
    sum += rpmArray[i]; // Sum all RPM values
  }
  return sum / count;
}

// Add separate average RPM calculation for motor 2
float calculateAverageRPM_B(float newRPM)
{
    rpmArrayB[rpmIndexB] = newRPM;
    rpmIndexB = (rpmIndexB + 1) % RPM_ARRAY_SIZE;
    if (rpmIndexB == 0)
    {
        rpmArrayFullB = true;
    }

    float sum = 0.0;
    int count = rpmArrayFullB ? RPM_ARRAY_SIZE : rpmIndexB;
    if (count == 0)
        return 0.0;

    for (int i = 0; i < count; i++)
    {
        sum += rpmArrayB[i];
    }
    return sum / count;
}

// Interrupt handler for encoder state changes
void IRAM_ATTR handleEncoder() 
{
    // Only increment on rising edge of channel A
    if (digitalRead(ENCODER_A) == HIGH) 
    {
        encoderPosition = isForward ? encoderPosition + 1 : encoderPosition - 1;
    }
}

// Add interrupt handler for second encoder
void IRAM_ATTR handleEncoder2() {
    if (digitalRead(ENCODER_B) == HIGH) {
        encoderPositionB = isForward ? encoderPositionB + 1 : encoderPositionB - 1;
    }
}

// Split speed calculation into two separate functions
void calculateSpeedMotor1() {
    unsigned long currentTimeM1 = millis();
    if (currentTimeM1 - lastSpeedCalc >= SPEED_INTERVAL) {
        float timeElapsedM1 = (currentTimeM1 - lastSpeedCalc) / 1000.0;
        long positionChangeM1 = abs(encoderPosition - lastPosition);
        float instantRPM_M1 = (positionChangeM1 / PULSES_PER_REV) * (60.0 / timeElapsedM1);
        instantRPM_M1 *= GEAR_RATIO;
        motorRPM = calculateAverageRPM(instantRPM_M1);
        lastSpeedCalc = currentTimeM1;
        lastPosition = encoderPosition;
    }
}

// Replace Motor 2 speed calculation function:
void calculateSpeedMotor2() {
    unsigned long currentTimeM2 = millis();
    if (currentTimeM2 - lastSpeedCalcB >= SPEED_INTERVAL) {
        float timeElapsedM2 = (currentTimeM2 - lastSpeedCalcB) / 1000.0;
        long positionChangeM2 = abs(encoderPositionB - lastPositionB);
        float instantRPM_M2 = (positionChangeM2 / PULSES_PER_REV) * (60.0 / timeElapsedM2);
        instantRPM_M2 *= GEAR_RATIO;
        motorRPM_B = calculateAverageRPM_B(instantRPM_M2);
        lastSpeedCalcB = currentTimeM2;
        lastPositionB = encoderPositionB;
    }
}

// Modify the displayHelp() function
void displayHelp()
{
    Serial.println(F("\nMotor Control Commands:"));
    Serial.println(F("----------------------"));
    Serial.println(F("F<speed> - Forward (0-100%)"));
    Serial.println(F("R<speed> - Reverse (0-100%)"));
    Serial.println(F("S       - Stop motor"));
    Serial.println(F("M<rpm>  - Set motor RPM"));
    Serial.println(F("T<x,y>  - Set target position (x,y)"));
    Serial.println(F("H       - Display this help"));
    Serial.println(F("\nExamples:"));
    Serial.println(F("F40     - Forward at 40%"));
    Serial.println(F("R75     - Reverse at 75%"));
    Serial.println(F("M3000   - Set speed to 3000 RPM"));
    Serial.println(F("T30,5   - Set target to (30m, 5m)"));
    Serial.println(F("----------------------"));
}

// Add the initial velocity display function
void displayInitialVelocity() {
    if (projectile) {
        Serial.println(F("\n*** Initial Velocity Calculation ***"));
        Serial.print(F("Target position: ("));
        Serial.print(projectile->getTargetX(), 3);
        Serial.print(F(", ")); 
        Serial.print(projectile->getTargetY(), 3);
        Serial.println(F(")"));
        Serial.print(F("Launch angle: ")); 
        Serial.print(projectile->getAngle(), 3);
        Serial.println(F(" degrees"));
        Serial.print(F("Required velocity: "));
        Serial.print(projectile->getVelocity(), 3);
        Serial.println(F(" m/s"));
        targetRPM = projectile->velocityToRPM(projectile->getVelocity());
        Serial.print(F("Required motor speed: "));
        Serial.print(targetRPM, 0);
        Serial.println(F(" RPM"));
        Serial.println(F("********************************\n"));
    }
}

// Add this function to check disk alignment
bool checkDiskAlignment() {
    // Calculate current positions in degrees (0-360)
    disk1Position = fmod(abs(convertToDegrees(encoderPosition)), 360.0);
    disk2Position = fmod(abs(convertToDegrees(encoderPositionB)), 360.0);
    
    // Calculate minimum angle difference to nearest fin alignment
    float angleDiff = fmod(abs(disk1Position - disk2Position), DEGREES_PER_FIN);
    if (angleDiff > DEGREES_PER_FIN/2) {
        angleDiff = DEGREES_PER_FIN - angleDiff;
    }
    
    return angleDiff <= ALIGNMENT_TOLERANCE;
}

// Setup Code
// ****************************************************************************
void setup()
{
  Serial.begin(115200); // Start serial communication
  pinMode(LED, OUTPUT); // Set LED pin as output

  // Configure motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initial motor state is off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);

  // Configure PWM for motor speed control - must be done before using ledcWrite()
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);

  // Initial PWM state (stopped) - must be done after ledcSetup and ledcAttachPin
  ledcWrite(PWM_CHANNEL, 0); // 0% duty cycle

  // Configure only encoder channel A as input with pullup
  pinMode(ENCODER_A, INPUT_PULLUP);

  // Only attach interrupt to channel A, and only trigger on RISING edge
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, RISING);

  // Configure second motor control pins
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB, PWM_CHANNEL_B);
  ledcWrite(PWM_CHANNEL_B, 0);

  // Configure second encoder channel B as input with pullup
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), handleEncoder2, RISING);

  // Configure third motor control pins
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  ledcSetup(PWM_CHANNEL_C, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENC, PWM_CHANNEL_C);
  ledcWrite(PWM_CHANNEL_C, 0);

  // Initialize RPM averaging array
  for (int i = 0; i < RPM_ARRAY_SIZE; i++)
  {
    rpmArray[i] = 0.0;
  }

  // Initialize RPM averaging array for motor 2
  for (int i = 0; i < RPM_ARRAY_SIZE; i++)
  {
    rpmArrayB[i] = 0.0;
  }

  // Print initial message to Serial Monitor
  Serial.println(); // Print a blank line for readability
  Serial.println("L298N Motor Control Example");
  Serial.println("---------------------------");
  Serial.println("Commands:");
  Serial.println("F<speed> - Forward (0-100%)");
  Serial.println("R<speed> - Reverse (0-100%)");
  Serial.println("S - Stop");

  Serial.println(F("\nL298N Motor Control Ready"));
  displayHelp();

  // Initialize projectile motion calculation
  projectile = new ProjectileMotion(0.0, 0.0, 10.0, 5.0); // Example target at (30m, 5m)
  
  // Display initial calculations
  displayInitialVelocity();
}

// Main program
// ****************************************************************************
void loop() {
    // Heartbeat LED code
    if (millis() - ledBlinkTime > BLINK_INTERVAL) {
        ledBlinkTime = millis();     // Update time of last LED blink
        ledState = !ledState;        // Toggle LED state
        digitalWrite(LED, ledState); // Set LED state
    }

    // Modify the motor control section in loop():
    if (motorRunning) {
        unsigned long currentTime = millis();
        
        // Calculate speed for both motors simultaneously
        calculateSpeedMotor1();
        calculateSpeedMotor2();
        
        // Motor 1 control logic
        float rpmError = abs(targetRPM - motorRPM);
        unsigned long currentAdjustInterval;
        float adjustmentRate;
        
        if (!isStable) {
            if (rpmError > 1000) {
                currentAdjustInterval = VERY_COARSE_ADJUST_INTERVAL;
                adjustmentRate = VERY_COARSE_ADJUSTMENT;
            } else if (rpmError > 500) {
                currentAdjustInterval = COARSE_ADJUST_INTERVAL;
                adjustmentRate = COARSE_ADJUSTMENT;
            } else if (rpmError > 250) {
                currentAdjustInterval = MEDIUM_ADJUST_INTERVAL;
                adjustmentRate = MEDIUM_ADJUSTMENT;
            } else if (rpmError > 50) {
                currentAdjustInterval = FINE_ADJUST_INTERVAL;
                adjustmentRate = FINE_ADJUSTMENT;
            } else {
                currentAdjustInterval = ULTRA_FINE_ADJUST_INTERVAL;
                adjustmentRate = ULTRA_FINE_ADJUSTMENT;
            }
            
            if (currentTime - lastAdjustTime >= currentAdjustInterval) {
                lastAdjustTime = currentTime;
                
                if (abs(rpmError) <= RPM_TOLERANCE) {
                    if (currentTime - stableStartTime >= STABLE_TIME) {
                        isStable = true;
                        Serial.println(F("\n=== Motor 1 Stabilized ==="));
                    }
                } else {
                    stableStartTime = currentTime;
                    float adjustment = adjustmentRate * ((targetRPM - motorRPM) > 0 ? 1 : -1);
                    currentSpeedPercent += adjustment;
                    currentSpeedPercent = constrain(currentSpeedPercent, 0, 100);
                    ledcWrite(PWM_CHANNEL, map(currentSpeedPercent, 0, 100, MIN_PWM, MAX_PWM));
                }
            }
        }

        // Update Motor 2 control logic section:
        float rpmError2 = abs(targetRPM - motorRPM_B);        
        unsigned long currentAdjustInterval2;
        unsigned long currentTimeM2 = millis();

        if (!isMotor2Stable) {
            // Assign values based on motor 2's error, not
            if (rpmError2 > 1000) {
                currentAdjustInterval2 = VERY_COARSE_ADJUST_INTERVAL;
            } else if (rpmError2 > 500) {
                currentAdjustInterval2 = COARSE_ADJUST_INTERVAL;
            } else if (rpmError2 > 250) {
                currentAdjustInterval2 = MEDIUM_ADJUST_INTERVAL;
            } else if (rpmError2 > 100) {
                currentAdjustInterval2 = FINE_ADJUST_INTERVAL;
            } else {
                currentAdjustInterval2 = ULTRA_FINE_ADJUST_INTERVAL;
            }

            if (currentTimeM2 - lastAdjustTimeB >= currentAdjustInterval2) {
                lastAdjustTimeB = currentTimeM2;
                
                if (abs(rpmError2) <= RPM_TOLERANCE) {
                    if (currentTimeM2 - stableStartTime2 >= STABLE_TIME) {
                        isMotor2Stable = true;
                        Serial.println(F("\n=== Motor 2 Stabilized ==="));
                    }
                } else {
                    stableStartTime2 = currentTimeM2;
                    // Use exact same adjustment rates as motor 1
                    float adjustmentRate2;
                    if (rpmError2 > 1000) adjustmentRate2 = VERY_COARSE_ADJUSTMENT;
                    else if (rpmError2 > 500) adjustmentRate2 = COARSE_ADJUSTMENT;
                    else if (rpmError2 > 250) adjustmentRate2 = MEDIUM_ADJUSTMENT;
                    else if (rpmError2 > 100) adjustmentRate2 = FINE_ADJUSTMENT;
                    else adjustmentRate2 = ULTRA_FINE_ADJUSTMENT;

                    float adjustment2 = adjustmentRate2 * ((targetRPM - motorRPM_B) > 0 ? 1 : -1);
                    currentSpeedPercent2 += adjustment2;
                    currentSpeedPercent2 = constrain(currentSpeedPercent2, 0, 100);
                    ledcWrite(PWM_CHANNEL_B, map(currentSpeedPercent2, 0, 100, MIN_PWM, MAX_PWM));
                }
            }
        }

        // Status display update section:
        if (currentTimeM2 - lastRpmPrint >= RPM_PRINT_INTERVAL) {
            lastRpmPrint = currentTimeM2;
            
            bool currentAlignment = checkDiskAlignment();
            if (currentAlignment) {
                consecutiveAlignments++;
                // Only start motor 3 if both motors are stable and aligned 3 times
                if (consecutiveAlignments >= ALIGNED_COUNT_THRESHOLD && !motor3Running && isStable && isMotor2Stable) {
                    // Start motor 3
                    motor3Running = true;
                    digitalWrite(IN5, HIGH);
                    digitalWrite(IN6, LOW);
                    ledcWrite(PWM_CHANNEL_C, map(35, 0, 100, MIN_PWM, MAX_PWM));
                    Serial.println(F("\n=== Starting Motor 3 at 35% ==="));
                }
            } else {
                consecutiveAlignments = 0;  // Reset counter if not aligned
            }
            
            // Print status including alignment and stability
            Serial.print(F("Target: ")); 
            Serial.print(targetRPM, 1);
            Serial.print(F(" | M1: ")); 
            Serial.print(motorRPM, 1);
            Serial.print(F("("));
            Serial.print(isStable ? "S" : "U");
            Serial.print(F(") | M2: "));
            Serial.print(motorRPM_B, 1);
            Serial.print(F("("));
            Serial.print(isMotor2Stable ? "S" : "U");
            Serial.print(F(") | Aligned: "));
            Serial.print(disksAligned ? F("Yes(") : F("No("));
            Serial.print(consecutiveAlignments);
            Serial.println(F(")"));
        }

        // Initialize both motors when starting
        if (motorRunning && !motor2Running) {
            motor2Running = true;
            targetRPM2 = targetRPM;
            isMotor2Stable = false;  // Explicitly set to false on startup
            
            // Initialize both motors
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            
            // Start both at 50%
            currentSpeedPercent = 50.0;
            currentSpeedPercent2 = 50.0;
            ledcWrite(PWM_CHANNEL, map(50, 0, 100, MIN_PWM, MAX_PWM));
            ledcWrite(PWM_CHANNEL_B, map(50, 0, 100, MIN_PWM, MAX_PWM));
            
            // Reset all variables
            encoderPosition = 0;
            encoderPositionB = 0;
            lastPosition = 0;
            lastPositionB = 0;
            lastSpeedCalc = currentTime;
            lastSpeedCalcB = currentTime;
            stableStartTime = currentTime;
            stableStartTime2 = currentTime;
            lastAdjustTime = currentTime;
            lastAdjustTimeB = currentTime;
            
            Serial.println(F("\n=== Starting Both Motors ==="));
        }
    }

    // Process serial input character by character
    while (Serial.available() > 0)
    {
        char inChar = (char)Serial.read(); // Read the incoming character
        Serial.write(inChar);              // Echo it back immediately

        if (inChar == '\n' || inChar == '\r')
        {
            // End of command, process it
            commandComplete = true;
        }
        else
        {
            // Add character to command string
            command += inChar;
        }
    }

    // Process completed command
    if (commandComplete)
    {
        Serial.println(); // Move to next line after command

        if (command.length() > 0)
        {
            // Convert first character to uppercase for case-insensitive comparison
            char cmdChar = toupper(command.charAt(0));

            switch (cmdChar)
            {
            case 'F': // Forward
            {
                // Reset encoder position when changing direction/speed

                // Extract the speed value as percentage
                int speedPercent = 0;
                if (command.length() > 1)
                {
                    speedPercent = command.substring(1).toInt();

                    // Validate speed range
                    if (speedPercent < 0 || speedPercent > 100)
                    {
                        Serial.println(F("Error: Speed must be between 0-100%"));
                        break;
                    }
                }
                encoderPosition = 0;
                lastPosition = 0;

                // Map percentage to PWM value (128-255)
                uint8_t pwmValue = map(speedPercent, 0, 100, 128, 255);

                isForward = true;                 // Set motor direction to forward
                digitalWrite(IN1, HIGH);          // Set IN1 high for forward direction
                digitalWrite(IN2, LOW);           // Set IN2 low for forward direction
                ledcWrite(PWM_CHANNEL, pwmValue); // Set PWM value for motor speed

                Serial.print(F("Forward: ")); // Print command
                Serial.print(speedPercent);
                Serial.println(F("%"));
            }
            break;

            case 'R': // Reverse
            {
                // Reset encoder position when changing direction/speed

                // Extract the speed value as percentage
                int speedPercent = 0;
                if (command.length() > 1)
                {
                    speedPercent = command.substring(1).toInt(); // Convert to integer

                    // Validate speed range
                    if (speedPercent < 0 || speedPercent > 100)
                    {
                        Serial.println(F("Error: Speed must be between 0-100%"));
                        break;
                    }
                }

                // Map percentage to PWM value (128-255)
                uint8_t pwmValue = map(speedPercent, 0, 100, 1208, 255);

                isForward = false;                // Set motor direction to reverse
                digitalWrite(IN1, LOW);           // Set IN1 low for reverse direction
                digitalWrite(IN2, HIGH);          // Set IN2 high for reverse direction
                ledcWrite(PWM_CHANNEL, pwmValue); // Set PWM value for motor speed

                Serial.print(F("Reverse: "));
                Serial.print(speedPercent);
                Serial.println(F("%"));
            }
            break;

            case 'S': // Stop
            {
                motorRunning = !motorRunning; // Toggle motor state
                motor2Running = false;        // Always stop motor 2
                isStable = false;            // Reset stability flags
                isMotor2Stable = false;

                if (!motorRunning) {
                    // Stop all motors
                    // Motor 1
                    digitalWrite(IN1, LOW);
                    digitalWrite(IN2, LOW);
                    ledcWrite(PWM_CHANNEL, 0);
                    
                    // Motor 2
                    digitalWrite(IN3, LOW);
                    digitalWrite(IN4, LOW);
                    ledcWrite(PWM_CHANNEL_B, 0);
                    
                    // Motor 3
                    digitalWrite(IN5, LOW);
                    digitalWrite(IN6, LOW);
                    ledcWrite(PWM_CHANNEL_C, 0);
                    motor3Running = false;
                    consecutiveAlignments = 0;  // Reset alignment counter
                    disksAligned = false;       // Reset alignment status
                    
                    // Reset variables
                    currentSpeedPercent = 0.0;
                    currentSpeedPercent2 = 0.0;
                    encoderPosition = 0;
                    encoderPositionB = 0;
                    lastPosition = 0;
                    lastPositionB = 0;
                    
                    Serial.println(F("All motors stopped"));
                } else {
                    // Initialize for projectile launch
                    encoderPosition = 0;
                    lastPosition = 0;
                    displayInitialVelocity();
                    Serial.println(F("Motor starting - Projectile launch mode"));
                }
                break;
            }

            case 'A': // Set angle
            {
                if (command.length() > 1) {
                    double newAngle = command.substring(1).toDouble();
                    if (newAngle >= 0 && newAngle <= 90) {
                        projectile->setAngle(newAngle);
                        
                        // Update target RPM
                        targetRPM = projectile->velocityToRPM(projectile->getVelocity());
                        targetRPM2 = targetRPM;
                        
                        // Reset stability for both motors
                        isStable = false;
                        isMotor2Stable = false;
                        
                        // Reset stability timers
                        stableStartTime = millis();
                        stableStartTime2 = millis();
                        
                        // Reset adjustment times
                        lastAdjustTime = millis();
                        lastAdjustTimeB = millis();
                        
                        // Display new calculations
                        displayInitialVelocity();
                    }
                    else {
                        Serial.println(F("Error: Angle must be between 0-90°"));
                    }
                }
                break;
            }

            case 'T': // Set target
            {
                if (command.length() > 1) {
                    int commaIndex = command.indexOf(',', 1);
                    if (commaIndex > 0) {
                        double targetX = command.substring(1, commaIndex).toDouble();
                        double targetY = command.substring(commaIndex + 1).toDouble();
                        
                        if (targetX >= 0 && targetY >= 0) {
                            delete projectile;
                            projectile = new ProjectileMotion(0.0, 0.0, targetX, targetY);
                            Serial.println(F("\nTarget set. Use 'A' command to set launch angle."));
                        } else {
                            Serial.println(F("Error: Target coordinates must be positive"));
                        }
                    } else {
                        Serial.println(F("Error: Use format T<x,y> (e.g., T30,5)"));
                    }
                }
                break;
            }

            case 'M': // Set RPM directly
            {
                if (command.length() > 1) {
                    float desiredRPM = command.substring(1).toFloat();
                    if (desiredRPM > 0 && desiredRPM <= MAX_RPM) {
                        if (projectile && projectile->calculateAngleFromRPM(desiredRPM)) {
                            targetRPM = desiredRPM;
                            targetRPM2 = desiredRPM;
                            
                            // Reset stability flags
                            isStable = false;
                            isMotor2Stable = false;
                            
                            // Reset timers
                            stableStartTime = millis();
                            stableStartTime2 = millis();
                            lastAdjustTime = millis();
                            lastAdjustTimeB = millis();
                            
                            // Display calculations
                            displayInitialVelocity();
                        } else {
                            Serial.println(F("Error: Cannot reach target with this RPM"));
                        }
                    } else {
                        Serial.print(F("Error: RPM must be between 0 and "));
                        Serial.println(MAX_RPM);
                    }
                }
                break;
            }

            case 'H': // Help
            case '?': // Alternative help command
                displayHelp();
                break;

            default:
                Serial.println(F("Unknown command. Type 'H' for help.")); 
                break;
            }
        }

        // Reset for next command
        command = "";
        commandComplete = false;
    }
}

