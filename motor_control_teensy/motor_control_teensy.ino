#include <Arduino.h>

// DRV8833 pins
#define L_IN1 4
#define L_IN2 5
#define R_IN1 10
#define R_IN2 9

unsigned long motorStartTime = 0;
const unsigned long runDuration = 5000;

int activeMotor = 0;   // 0 = left, 1 = right
int currentSpeed = 0;
bool motorRunning = false;

void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);  // Use analogWrite(0) instead of digitalWrite
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    // BRAKE: Both pins LOW with PWM disabled
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void stopAllMotors() {
  // Set all motor pins to 0 using analogWrite to ensure PWM is disabled
  analogWrite(L_IN1, 0);
  analogWrite(L_IN2, 0);
  analogWrite(R_IN1, 0);
  analogWrite(R_IN2, 0);
  
  currentSpeed = 0;
  motorRunning = false;
  motorStartTime = 0;
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000) {}

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  stopAllMotors();

  Serial.println("=== Motor Control Ready ===");
  Serial.println("Enter speed: 0.1 to 1.0 (e.g., 0.5 or 1.0)");
  Serial.println("Enter 'S' to stop");
  Serial.println("Motors alternate: Left → Right → Left...");
  Serial.println();
}

void loop() {
  // -------- Handle serial input --------
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Debug: Show what was received
    Serial.print("Received: '");
    Serial.print(input);
    Serial.println("'");

    // STOP command
    if (input.equalsIgnoreCase("S")) {
      stopAllMotors();
      Serial.println("✓ Motors stopped manually");
      Serial.println();
      return;
    }

    // Parse input - handle both integer and float
    float val = input.toFloat();
    
    // toFloat() returns 0.0 on failure, so check if input was actually "0"
    if (val == 0.0 && input != "0" && input != "0.0") {
      Serial.println("✗ Parse error - use format like: 0.5 or 1.0");
      Serial.println();
      return;
    }

    // Validate range (allow 0 for explicit stop, or 0.1-1.0 for motor speed)
    if (val < 0.0 || val > 1.0) {
      Serial.println("✗ Out of range (use 0.0–1.0)");
      Serial.println();
      return;
    }

    // Handle explicit zero as stop
    if (val == 0.0) {
      stopAllMotors();
      Serial.println("✓ Motors stopped (speed = 0)");
      Serial.println();
      return;
    }

    // Convert to PWM (0.1-1.0 → 26-255)
    int targetSpeed = constrain((int)(val * 255), 0, 255);
    
    Serial.print("Parsed value: ");
    Serial.print(val, 2);
    Serial.print(" → PWM: ");
    Serial.println(targetSpeed);

    // Brief stop before switching (this resets currentSpeed to 0)
    stopAllMotors();
    delay(50);

    // Now set the target speed
    currentSpeed = targetSpeed;

    // Start next motor
    if (activeMotor == 0) {
      setMotor(L_IN1, L_IN2, currentSpeed);
      Serial.print("→ LEFT motor @ PWM ");
      activeMotor = 1;
    } else {
      setMotor(R_IN1, R_IN2, currentSpeed);
      Serial.print("→ RIGHT motor @ PWM ");
      activeMotor = 0;
    }

    Serial.print(currentSpeed);
    Serial.print(" (");
    Serial.print((val * 100), 0);
    Serial.println("%)");
    Serial.println();
    
    motorStartTime = millis();
    motorRunning = true;
  }

  // -------- Auto-stop after duration --------
  if (motorRunning && millis() - motorStartTime >= runDuration) {
    stopAllMotors();
    Serial.println("⏱ Motor auto-stopped after 5s");
    Serial.println();
  }
}