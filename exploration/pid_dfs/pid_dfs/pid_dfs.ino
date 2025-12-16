#include <Arduino.h>

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define ENCODER_PIN 2  // Single encoder for distance

const int numSensors = 8;
const int sensorPins[numSensors] PROGMEM = {A7, A6, A5, A4, A3, A2, A1, A0};  // Store in flash

bool isBlackLine = 0;  // White line on black floor
const float threshold[numSensors] PROGMEM = {940, 902, 883, 874, 851, 863, 862, 866};  // Store in flash
const int8_t weight[numSensors] PROGMEM = {-8, -4, -2, -1, 1, 2, 4, 8};  // Store in flash, use int8_t

// PID constants
float Kp = 40.0;
float Kd = 25.0;  // Removed Ki since it's 0
float previousError = 0;

// Edge structure - optimized for memory
struct Edge {
    int8_t node;    // Destination node (-1 if no edge) - using int8_t saves memory
    int8_t dir;     // Direction: 1=left, 2=straight, 3=right, 4=back
    uint16_t dist;  // Distance in encoder counts - uint16_t (0-65535 range)
};

// Graph: graph[i][j] = edge from node i in direction j
const int MAX_NODES = 20;  // Reduced from 50 to 20 nodes
Edge graph[MAX_NODES][4];  // 20*4*4 = 320 bytes (was 1200 bytes)
int8_t nodeCount = 0;

// DFS stack: stores [node, direction_to_try_next]
int8_t dfsStack[30][2];  // Reduced from 100 to 30 (120 bytes vs 400 bytes)
int8_t stackTop = -1;

int8_t currentNode = 0;
volatile uint16_t encoderCount = 0;  // uint16_t is enough for most cases
uint16_t lastEncoderCount = 0;

// Turn detection debouncing
bool atJunction = false;
unsigned long junctionDebounce = 0;
const int DEBOUNCE_MS = 800;

void encoderISR() {
    encoderCount++;
}

void setMotor(int pin1, int pin2, int pwm, int speed) {
    speed = constrain(speed, -80, 80);  // Your original constraint
    if (speed > 0) {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, speed);
    } else if (speed < 0) {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, -speed);
    } else {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, 0);
    }
}

void motor1run(int speed) { setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed) { setMotor(BIN1, BIN2, PWMB, speed); }

void initGraph() {
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < 4; j++) {
            graph[i][j].node = -1;
            graph[i][j].dir = j + 1;  // 1=left, 2=straight, 3=right, 4=back
            graph[i][j].dist = 0;
        }
    }
    nodeCount = 1;
    currentNode = 0;
}

void turnLeft() {
    Serial.println(F("LEFT"));
    
    while (true) {
        motor1run(-100);
        motor2run(100);
        int mid1 = analogRead(pgm_read_byte(&sensorPins[3]));
        int mid2 = analogRead(pgm_read_byte(&sensorPins[4]));
        bool online = (isBlackLine) ? (mid1 >= pgm_read_float(&threshold[3]) && mid2 >= pgm_read_float(&threshold[4])) :
                                      (mid1 < pgm_read_float(&threshold[3]) && mid2 < pgm_read_float(&threshold[4]));
        if (online) break;
    }
    motor1run(0);
    motor2run(0);
    delay(20);  // Your original delay
}

void turnRight() {
    Serial.println(F("RIGHT"));
    
    while (true) {
        motor1run(100);
        motor2run(-100);
        int mid1 = analogRead(pgm_read_byte(&sensorPins[3]));
        int mid2 = analogRead(pgm_read_byte(&sensorPins[4]));
        bool online = (isBlackLine) ? (mid1 >= pgm_read_float(&threshold[3]) && mid2 >= pgm_read_float(&threshold[4])) :
                                      (mid1 < pgm_read_float(&threshold[3]) && mid2 < pgm_read_float(&threshold[4]));
        if (online) break;
    }
    motor1run(0);
    motor2run(0);
    delay(20);  // Your original delay
}

void turnBack() {
    Serial.println(F("U-TURN"));
    
    // Spin 180
    unsigned long start = millis();
    while (millis() - start < 800) {
        motor1run(100);
        motor2run(-100);
    }
    
    // Fine tune to line
    while (true) {
        motor1run(100);
        motor2run(-100);
        int mid1 = analogRead(pgm_read_byte(&sensorPins[3]));
        int mid2 = analogRead(pgm_read_byte(&sensorPins[4]));
        bool online = (isBlackLine) ? (mid1 >= pgm_read_float(&threshold[3]) && mid2 >= pgm_read_float(&threshold[4])) :
                                      (mid1 < pgm_read_float(&threshold[3]) && mid2 < pgm_read_float(&threshold[4]));
        if (online) break;
    }
    motor1run(0);
    motor2run(0);
    delay(20);  // Your original delay
}

void detectAvailablePaths(bool &left, bool &straight, bool &right) {
    // Read sensors directly without storing array
    bool s0 = analogRead(pgm_read_byte(&sensorPins[0])) < (isBlackLine ? pgm_read_float(&threshold[0]) : pgm_read_float(&threshold[0]));
    if (isBlackLine) s0 = !s0;
    bool s1 = analogRead(pgm_read_byte(&sensorPins[1])) < (isBlackLine ? pgm_read_float(&threshold[1]) : pgm_read_float(&threshold[1]));
    if (isBlackLine) s1 = !s1;
    bool s6 = analogRead(pgm_read_byte(&sensorPins[6])) < (isBlackLine ? pgm_read_float(&threshold[6]) : pgm_read_float(&threshold[6]));
    if (isBlackLine) s6 = !s6;
    bool s7 = analogRead(pgm_read_byte(&sensorPins[7])) < (isBlackLine ? pgm_read_float(&threshold[7]) : pgm_read_float(&threshold[7]));
    if (isBlackLine) s7 = !s7;
    
    bool s3 = analogRead(pgm_read_byte(&sensorPins[3])) < (isBlackLine ? pgm_read_float(&threshold[3]) : pgm_read_float(&threshold[3]));
    if (isBlackLine) s3 = !s3;
    bool s4 = analogRead(pgm_read_byte(&sensorPins[4])) < (isBlackLine ? pgm_read_float(&threshold[4]) : pgm_read_float(&threshold[4]));
    if (isBlackLine) s4 = !s4;
    
    left = (s0 || s1);
    right = (s7 || s6);
    straight = (s3 || s4);
}

void addEdge(int8_t from, int8_t dirIndex, int8_t to, uint16_t distance) {
    graph[from][dirIndex].node = to;
    graph[from][dirIndex].dist = distance;
    
    Serial.print(F("Edge: Node"));
    Serial.print(from);
    Serial.print(F(" -[dir:"));
    Serial.print(dirIndex + 1);
    Serial.print(F(",dist:"));
    Serial.print(distance);
    Serial.print(F("]-> Node"));
    Serial.println(to);
}

int8_t getNextUnexploredDir(int8_t node) {
    // Returns direction index (0=left, 1=straight, 2=right, 3=back) or -1 if all explored
    for (int8_t i = 0; i < 3; i++) {  // Check left, straight, right
        if (graph[node][i].node == -1) {
            return i;
        }
    }
    return -1;  // All explored
}

void printGraph() {
    Serial.println(F("\n====== GRAPH STRUCTURE ======"));
    Serial.print(F("Nodes: "));
    Serial.println(nodeCount);
    
    for (int8_t i = 0; i < nodeCount; i++) {
        Serial.print(F("Node "));
        Serial.print(i);
        Serial.println(F(":"));
        for (int8_t j = 0; j < 4; j++) {
            if (graph[i][j].node != -1) {
                Serial.print(F("  Dir "));
                Serial.print(j + 1);
                Serial.print(F(": -> Node "));
                Serial.print(graph[i][j].node);
                Serial.print(F(" ("));
                Serial.print(graph[i][j].dist);
                Serial.println(F(" counts)"));
            }
        }
    }
    Serial.println(F("=============================\n"));
}

void setup() {
    Serial.begin(9600);
    
    for (int i = 0; i < numSensors; i++) pinMode(pgm_read_byte(&sensorPins[i]), INPUT);
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
    
    initGraph();
    lastEncoderCount = encoderCount;
    
    Serial.println(F("DFS Explorer Ready!"));
    delay(3000);
}

void loop() {
    // Read sensors for line following
    float error = 0.0;
    int activeCount = 0;
    
    // Read sensors directly without storing in array
    for (int i = 0; i < numSensors; i++) {
        int val = analogRead(pgm_read_byte(&sensorPins[i]));
        bool sensor = (isBlackLine) ? (val >= pgm_read_float(&threshold[i])) : (val < pgm_read_float(&threshold[i]));
        if (sensor) activeCount++;
        error += sensor * pgm_read_byte(&weight[i]);
    }
    
    if (activeCount == 0) {
        motor1run(0);
        motor2run(0);
        Serial.println(F("Line lost - DFS Complete!"));
        printGraph();
        while(1);
        return;
    }
    
    // Junction detection with debounce
    unsigned long now = millis();
    if (!atJunction && (now - junctionDebounce > DEBOUNCE_MS)) {
        bool left, straight, right;
        detectAvailablePaths(left, straight, right);
        
        if (left || right) {  // Junction detected
            atJunction = true;
            junctionDebounce = now;
            
            uint16_t distance = encoderCount - lastEncoderCount;
            
            Serial.print(F("\n=== Junction at Node "));
            Serial.print(currentNode);
            Serial.print(F(" | L:"));
            Serial.print(left);
            Serial.print(F(" S:"));
            Serial.print(straight);
            Serial.print(F(" R:"));
            Serial.print(right);
            Serial.print(F(" | Dist: "));
            Serial.println(distance);
            
            // Determine which direction to take
            int8_t dirToTake = -1;
            
            if (left && graph[currentNode][0].node == -1) dirToTake = 0;      // Left unexplored
            else if (straight && graph[currentNode][1].node == -1) dirToTake = 1;  // Straight unexplored
            else if (right && graph[currentNode][2].node == -1) dirToTake = 2;     // Right unexplored
            
            if (dirToTake != -1) {
                // Explore new path
                int8_t newNode = nodeCount++;
                addEdge(currentNode, dirToTake, newNode, distance);
                
                // Add reverse edge (back to current from new node)
                int8_t reverseDir = 3;  // Back
                addEdge(newNode, reverseDir, currentNode, distance);
                
                // Push current node to stack for backtracking
                dfsStack[++stackTop][0] = currentNode;
                dfsStack[stackTop][1] = dirToTake;
                
                currentNode = newNode;
                
                // Execute turn
                if (dirToTake == 0) turnLeft();
                else if (dirToTake == 2) turnRight();
                // dirToTake == 1 means straight, no turn
                
            } else {
                // Dead end or all explored - backtrack
                Serial.println(F("Backtracking..."));
                
                if (stackTop >= 0) {
                    int8_t prevNode = dfsStack[stackTop][0];
                    stackTop--;
                    
                    currentNode = prevNode;
                    turnBack();
                } else {
                    Serial.println(F("Stack empty - exploration complete!"));
                    printGraph();
                    motor1run(0);
                    motor2run(0);
                    while(1);
                }
            }
            
            lastEncoderCount = encoderCount;
            delay(200);  // Extra delay after junction processing
        }
    }
    
    // Reset junction flag when past junction
    if (atJunction && (now - junctionDebounce > 200)) {
        atJunction = false;
    }
    
    // PID line following (removed Ki since it's 0)
    float derivative = error - previousError;
    previousError = error;
    
    float correction = (Kp * error) + (Kd * derivative);
    
    int baseSpeed = 180;
    int leftSpeed = constrain(baseSpeed + correction, 0, 255);
    int rightSpeed = constrain(baseSpeed - correction, 0, 255);
    
    motor1run(leftSpeed);
    motor2run(rightSpeed);
}