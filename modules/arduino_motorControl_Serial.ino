// Motor Pins
const int M1_A = 9; // Left Motor Pin 1
const int M1_B = 10; // Left Motor Pin 2
const int M2_A = 5; // Right Motor Pin 1
const int M2_B = 6; // Right Motor Pin 2

void setup() {
  Serial.begin(115200);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);
  
  stopMotors(); // Start in a safe state
  Serial.println("Robot Ready. Send F, B, L, R, S or T for Test.");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    executeCommand(cmd);
  }
}

void executeCommand(char c) {
  switch (c) {
    case 'F': moveForward();  Serial.write('A'); break;
    case 'B': moveBackward(); Serial.write('A'); break;
    case 'L': turnLeft();     Serial.write('A'); break;
    case 'R': turnRight();    Serial.write('A'); break;
    case 'S': stopMotors();    Serial.write('A'); break;
    case 'T': runTestSequence(); break; // Test without needing Pi constant input
    default:  Serial.write('E'); break; // Error
  }
}

// --- Movement Functions ---

void moveForward() {
  digitalWrite(M1_A, LOW);  digitalWrite(M1_B, HIGH);
  digitalWrite(M2_A, LOW);  digitalWrite(M2_B, HIGH);
}

void moveBackward() {
  digitalWrite(M1_A, HIGH); digitalWrite(M1_B, LOW);
  digitalWrite(M2_A, HIGH); digitalWrite(M2_B, LOW);
}

void turnLeft() {
  digitalWrite(M1_A, HIGH); digitalWrite(M1_B, LOW);
  digitalWrite(M2_A, LOW);  digitalWrite(M2_B, HIGH);
}

void turnRight() {
  digitalWrite(M1_A, LOW);  digitalWrite(M1_B, HIGH);
  digitalWrite(M2_A, HIGH); digitalWrite(M2_B, LOW);
}

void stopMotors() {
  digitalWrite(M1_A, LOW);  digitalWrite(M1_B, LOW);
  digitalWrite(M2_A, LOW);  digitalWrite(M2_B, LOW);
}

// --- Self-Test Sequence ---
void runTestSequence() {
  Serial.println("Starting Test Sequence...");
  char tests[] = {'F', 'B', 'L', 'R', 'S'};
  for (int i = 0; i < 5; i++) {
    executeCommand(tests[i]);
    delay(1000); // Move for 1 second for each command
  }
  Serial.println("Test Complete.");
}