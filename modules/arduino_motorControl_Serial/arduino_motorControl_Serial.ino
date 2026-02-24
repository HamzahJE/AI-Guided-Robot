// ============================================================================
// AI Driver Car — Smart Motor Controller with I2C LCD & Velocity Persistence
// ============================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display (sometimes 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- Motor Pins ---
const int M1_A = 9;  
const int M1_B = 10; 
const int M2_A = 5;  
const int M2_B = 6;  

// --- Ultrasonic Sensor Pins ---
const int TRIG_F = 7; 
const int ECHO_F = 8; 

// --- Tuning Constants ---
const int STOP_DIST = 15;       // cm — increased slightly for safety
const int SLOW_DIST = 40;       // cm — start slowing down
const int BASE_SPEED = 200;     // PWM 0-255
const int MIN_SPEED = 90;       // PWM — slowest we'll go
const int TURN_SPEED = 180;     
const int SENSOR_INTERVAL = 50; // ms

// --- Velocity Persistence Constants ---
const unsigned long COMMAND_TIMEOUT = 1000;  // ms before slowing down (LLM is thinking)
const unsigned long EMERGENCY_TIMEOUT = 3000; // ms before full stop (LLM crashed)

// --- State Variables ---
char currentGoal = 'S'; 
char lastTurnBias = 'L'; // Remembers last turn direction for smarter evasion
unsigned long lastSensorRead = 0;
unsigned long lastCommandTime = 0;
int distFront = 999;
const int CLEAR_DIST = 90; // cm — distance considered an "open path"
// LCD tracking to prevent flickering
String currentLine1 = "";
String currentLine2 = "";

void setup() {
  delay(2000); 
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  updateLCD("Robot Booting...", "Awaiting Pi");

  pinMode(M1_A, OUTPUT); pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT); pinMode(M2_B, OUTPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);

  stopMotors();
}

void loop() {
  // --- 1. Check for new commands from Pi ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
      currentGoal = cmd;
      lastCommandTime = millis(); // Reset timeout
      
      // Update turn bias for future memory
      if (cmd == 'L' || cmd == 'R') lastTurnBias = cmd; 
      
      Serial.write('A'); // Send ACK back to Pi silently
    }
  }

  // --- 2. Read Sensor ---
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    distFront = readDistance(TRIG_F, ECHO_F);
  }

  // --- 3. Velocity Persistence & Safety Timeouts ---
  unsigned long timeSinceCommand = millis() - lastCommandTime;
  int currentSpeed = BASE_SPEED;

  if (timeSinceCommand > EMERGENCY_TIMEOUT && currentGoal != 'S') {
    currentGoal = 'S';
    updateLCD("ERROR: LLM Lost", "Emergency Stop");
    stopMotors();
    return;
  } else if (timeSinceCommand > COMMAND_TIMEOUT && currentGoal == 'F') {
    // LLM is taking a while, decelerate gracefully
    currentSpeed = MIN_SPEED;
    updateLCD("LLM Thinking...", "Coasting safely");
  } else {
    // Normal operation display
    updateLCD("LLM Goal: " + String(currentGoal), "Dist: " + String(distFront) + "cm");
  }

  // --- 4. Smart Driving & Evasion Logic ---
  if (currentGoal == 'S') {
    stopMotors();
    return;
  }

  if (currentGoal == 'F') {
    if (distFront <= STOP_DIST) {
      // BLOCKED: Smart Pivot & Peek
      updateLCD("BLOCKED!", "Pivoting...");
      stopMotors();
      delay(100);
      
      // Arc backwards based on last known good direction
      if (lastTurnBias == 'L') {
        driveMotors(MIN_SPEED, BASE_SPEED, false); // Arc Left Back
      } else {
        driveMotors(BASE_SPEED, MIN_SPEED, false); // Arc Right Back
      }
      delay(350);
      
      stopMotors();
      currentGoal = 'S'; // Wait for LLM to send new command based on new view
      return;
    }

    // Dynamic braking
    if (distFront < SLOW_DIST) {
      currentSpeed = map(distFront, STOP_DIST, SLOW_DIST, MIN_SPEED, currentSpeed);
      currentSpeed = constrain(currentSpeed, MIN_SPEED, BASE_SPEED);
    }
    driveMotors(currentSpeed, currentSpeed, true);
    return;
  }

  if (currentGoal == 'B') driveMotors(BASE_SPEED, BASE_SPEED, false);
  // --- TURN LEFT OR RIGHT (Acoustic Seeking) ---
  if (currentGoal == 'L' || currentGoal == 'R') {
    
    // Check if the sensor sees a clear gap while we are turning
    if (distFront > CLEAR_DIST) {
      // GAP FOUND! Stop turning and auto-switch to Forward
      currentGoal = 'F'; 
      updateLCD("Gap Found!", "Auto-Forward");
      driveMotors(BASE_SPEED, BASE_SPEED, true);
      return; 
    }

    // Otherwise, keep turning to look for a gap
    if (currentGoal == 'L') {
      setMotor(M1_A, M1_B, TURN_SPEED, false); 
      setMotor(M2_A, M2_B, TURN_SPEED, true);
    } else {
      setMotor(M1_A, M1_B, TURN_SPEED, true); 
      setMotor(M2_A, M2_B, TURN_SPEED, false);
    }
    return;
  }
}

// --- Helper Functions ---

void updateLCD(String line1, String line2) {
  // Only update if text changes to prevent awful screen flickering
  if (line1 != currentLine1 || line2 != currentLine2) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);
    currentLine1 = line1;
    currentLine2 = line2;
  }
}

void setMotor(int pinA, int pinB, int speed, bool forward) {
  if (forward) { analogWrite(pinA, 0); analogWrite(pinB, speed); }
  else         { analogWrite(pinA, speed); analogWrite(pinB, 0); }
}

void driveMotors(int leftSpeed, int rightSpeed, bool forward) {
  setMotor(M1_A, M1_B, leftSpeed, forward);
  setMotor(M2_A, M2_B, rightSpeed, forward);
}

void stopMotors() {
  analogWrite(M1_A, 0); analogWrite(M1_B, 0);
  analogWrite(M2_A, 0); analogWrite(M2_B, 0);
}

int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); 
  if (duration == 0) return 999;                
  return duration * 0.034 / 2; 
}