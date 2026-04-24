#include <Arduino.h>

// ---------------- MOTOR & SENSOR DEFINITIONS -----------------
#define leftMotor1 17
#define leftMotor2 16
#define rightMotor1 19
#define rightMotor2 18
#define sensor1 32
#define sensor2 33
#define sensor3 35
#define sensor4 34
#define sensor5 25
#define sensor6 26
#define button 12
#define led 2

#define LEFT_MOTOR1_CH 0
#define LEFT_MOTOR2_CH 1
#define RIGHT_MOTOR1_CH 2
#define RIGHT_MOTOR2_CH 3
#define PWM_FREQ 1000
#define PWM_RES 8 

// ---------------- GLOBAL VARIABLES ---------------------------
const int sensors[6] = {sensor1, sensor2, sensor3, sensor4, sensor5, sensor6};
int threshvalue;
int rawSensorValues[6];
int binarySensor[6];

// Dynamic variables for "Flags"
int currentWeights[6] = {-0, -2, -3, 3, 2, 0}; 
int currentBaseSpeed = 240;

float Kp = 70.0;//60
float Ki = 0.0;
float Kd = 0.6;//.5

float error;
float lastError;
float D, I = 0.0;
float PIDvalue;

unsigned long lastTime = 0;
unsigned long startTime = 0;
unsigned long lastLineSeenTime = 0;
unsigned long allBlackStartTime = 0;
int currentPhase = 0; 

// --- VARIABLES FOR 100ms PATTERN DETECTION ---
unsigned long patternWhiteStartTime = 0;
unsigned long patternBlackStartTime = 0;
#define time_limit 70
#define maxIntegral 70
#define lostLineTimeout 200

// ---------------- MODE SETTINGS ------------------------
int mode = 0; // 0 = BLACK LINE, 1 = WHITE LINE
int donecalibrate = 0;

// ---------------- FUNCTION PROTOTYPES ------------------------
void calculatePID(int activeCount, int weightedSum);
void readSensors(int *array);
void calibrateSensors();
void waitForButtonPress();
int min(int* T);
int max(int* T);
void moveMotors(int leftMotorSpeed, int rightMotorSpeed);

// =============================================================
//                             SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  ledcSetup(LEFT_MOTOR1_CH, PWM_FREQ, PWM_RES);
  ledcSetup(LEFT_MOTOR2_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_MOTOR1_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_MOTOR2_CH, PWM_FREQ, PWM_RES);

  ledcAttachPin(leftMotor1, LEFT_MOTOR1_CH);
  ledcAttachPin(leftMotor2, LEFT_MOTOR2_CH);
  ledcAttachPin(rightMotor1, RIGHT_MOTOR1_CH);
  ledcAttachPin(rightMotor2, RIGHT_MOTOR2_CH);

  for(int i=0; i<6; i++) pinMode(sensors[i], INPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  calibrateSensors();
  delay(1000);
  lastTime = millis();
}

// =============================================================
//                             LOOP
// =============================================================
void loop() {
  if (donecalibrate == 0) return;

  unsigned long now = millis();
  if (startTime == 0) startTime = now; // Initialize run timer

  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.0001;

  readSensors(rawSensorValues);

  // ------------------- TIMER-BASED FLAGS -----------------------
  
  // FLAG 1: moraba3 sghir
  if (now - startTime > 570 && currentPhase == 0) {//550
    currentPhase = 1;
    digitalWrite(led, HIGH);
    currentBaseSpeed = 120; 
    int phase1Weights[6] = {-6, -5, -2, 1, 2, 2}; 
    for(int i=0; i<6; i++) currentWeights[i] = phase1Weights[i];
  }
  
  // FLAG 2: zigzag
  if (now - startTime > 900 && currentPhase == 1) {
    digitalWrite(led, LOW);
    currentPhase = 2;
    currentBaseSpeed = 180;//180 
    int phase2Weights[6] = {-3, -3, -1, 1, 3, 3}; 
    for(int i=0; i<6; i++) currentWeights[i] = phase2Weights[i];
  }
  
  // FLAG 3: dwira
  if (now - startTime > 6500 && currentPhase == 2) {
    digitalWrite(led, LOW);
    currentPhase = 3;
    currentBaseSpeed = 200; 
    int phase3Weights[6] = {-7, -6, -3, 2, 2, 1}; 
    for(int i=0; i<6; i++) currentWeights[i] = phase3Weights[i];
  }

  // ------------------- PATTERNS FOR FLAG 4 & 5 -------------------
  bool patternWhiteLine = (rawSensorValues[1] > threshvalue && 
                           rawSensorValues[4] > threshvalue && 
                           (rawSensorValues[2] < threshvalue || rawSensorValues[3] < threshvalue));

  bool patternBlackLine = (rawSensorValues[1] < threshvalue && 
                           rawSensorValues[4] < threshvalue &&
                           (rawSensorValues[2] > threshvalue || rawSensorValues[3] > threshvalue));

  // FLAG 4: Switch to white line (ACTIVATED ONLY AFTER 11350ms)
  if (now - startTime > 11000 && currentPhase == 3) { //11350
    currentBaseSpeed = 200; 
      int phase88Weights[6] = {0, -2, -3, 3, 2, 0}; 
      for(int i=0; i<6; i++) currentWeights[i] = phase88Weights[i]; 
    if (patternWhiteLine) {
      if (patternWhiteStartTime == 0) patternWhiteStartTime = now; // Start 100ms timer
      
      if (now - patternWhiteStartTime >= time_limit) { 
        digitalWrite(led, HIGH);
        currentPhase = 4;
        mode = 1; // Switch to WHITE line mode
        currentBaseSpeed = 160;
        int phase4Weights[6] = {-5, -3, -1, 1, 2, 2}; 
        for(int i=0; i<6; i++) currentWeights[i] = phase4Weights[i];
        patternWhiteStartTime = 0; // Reset
      }
    } else {
      patternWhiteStartTime = 0; // Reset timer if pattern breaks
    }
  }
  
  // FLAG 5: Return to black line (ACTIVATED ONLY AFTER 14000ms)
  if ( currentPhase == 4) { //now - startTime > 14000 && currentPhase == 4
    if (patternBlackLine) {
      if (patternBlackStartTime == 0) patternBlackStartTime = now; // Start 100ms timer
      
      if (now - patternBlackStartTime >= time_limit) { 
        digitalWrite(led, LOW);
        currentPhase = 5;
        mode = 0; // Switch back to BLACK line mode
        currentBaseSpeed = 240; 
        int phase5Weights[6] = {0, -2, -3, 3, 2, 0}; 
        for(int i=0; i<6; i++) currentWeights[i] = phase5Weights[i]; 
        patternBlackStartTime = 0; // Reset
      }
    } else {
      patternBlackStartTime = 0; // Reset timer if pattern breaks
    }
  }

  // FLAG 6: Stop when all sensors see black for 150ms (only after FLAG5)
  if (currentPhase >= 5 && now - startTime > 15000) {
    bool allSensorsBlack = true;
    for (int i = 0; i < 6; i++) {
      if (rawSensorValues[i] <= threshvalue) { // <= thresh means White, so NOT black
        allSensorsBlack = false;
        break;
      }
    }
    
    if (allSensorsBlack) {
      if (allBlackStartTime == 0) {
        allBlackStartTime = now; // Start tracking when all sensors turn black
      }
      if (now - allBlackStartTime > 150) {
        digitalWrite(led, HIGH);
        currentPhase = 6;
        moveMotors(0, 0); // Stop the robot
        while(true); // Halt further execution
      }
    } else {
      allBlackStartTime = 0; // Reset if any sensor sees a line
    }
  }

  // ------------------ READ LINE ACCORDING TO MODE --------------
  int activeCount = 0;
  int weightedSum = 0;

  for (int i = 0; i < 6; i++) {
    if (mode == 0) { // Black Line = 1
      binarySensor[i] = (rawSensorValues[i] > threshvalue) ? 1 : 0;
    } else {         // White Line = 1
      binarySensor[i] = (rawSensorValues[i] < threshvalue) ? 1 : 0;
    }

    if (binarySensor[i]) {
      activeCount++;
      weightedSum += currentWeights[i];
    }
  }

  // ------------------- LOST LINE HANDLING -----------------------
  if (activeCount > 0) lastLineSeenTime = now;

  if (activeCount == 0) {
    error = (lastError > 0) ? 50 : -50;
    if (now - lastLineSeenTime > lostLineTimeout) {
      moveMotors(100, 100);
      lastError = error;
      lastTime = now;
      return;
    }
  }

  // ------------------- PID & MOTOR CONTROL ----------------------
  calculatePID(activeCount, weightedSum);
  
  // Directly calculating motor speeds to respect currentBaseSpeed
  int leftSpeed = constrain(currentBaseSpeed - (int)PIDvalue, -255, 255);
  int rightSpeed = constrain(currentBaseSpeed + (int)PIDvalue, -255, 255);
  moveMotors(leftSpeed, rightSpeed);
  
  lastTime = now;
}

// ------------------------- FUNCTIONS ----------------------------

void readSensors(int *array) {
  for (int i = 0; i < 6; i++) array[i] = analogRead(sensors[i]);
}

void calibrateSensors() {
  int sumblack[6] = {0}, sumwhite[6] = {0};
  const int samples = 50;
  
  Serial.println("Place on BLACK line, press button...");
  waitForButtonPress();
  digitalWrite(led, HIGH);
  for (int j = 0; j < samples; j++) {
    for (int i = 0; i < 6; i++) sumblack[i] += analogRead(sensors[i]);
    delay(20);
  }
  digitalWrite(led, LOW);
  
  delay(1000);
  Serial.println("Place on WHITE surface, press button...");
  waitForButtonPress();
  digitalWrite(led, HIGH);
  for (int j = 0; j < samples; j++) {
    for (int i = 0; i < 6; i++) sumwhite[i] += analogRead(sensors[i]);
    delay(20);
  }
  digitalWrite(led, LOW);

  for (int i = 0; i < 6; i++) {
    sumblack[i] /= samples;
    sumwhite[i] /= samples;
  }
  
  threshvalue = (min(sumblack) + max(sumwhite)) / 2;
  Serial.print("Threshold set to: "); Serial.println(threshvalue);
  
  waitForButtonPress();
  donecalibrate = 1;
}

void calculatePID(int activeCount, int weightedSum) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.0001;

  if (activeCount > 0) error = (float)weightedSum / activeCount;
  
  I += error * dt;
  I = constrain(I, -maxIntegral, maxIntegral);
  D = (error - lastError) / dt;
  D = constrain(D, -100, 100);

  PIDvalue = (Kp * error) + (Ki * I) + (Kd * D);
  PIDvalue = constrain(PIDvalue, -255, 255);
  lastError = error;
}

void moveMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed >= 0) {
    ledcWrite(LEFT_MOTOR1_CH, leftMotorSpeed);
    ledcWrite(LEFT_MOTOR2_CH, 0);
  } else {
    ledcWrite(LEFT_MOTOR1_CH, 0);
    ledcWrite(LEFT_MOTOR2_CH, -leftMotorSpeed);
  }

  if (rightMotorSpeed >= 0) {
    ledcWrite(RIGHT_MOTOR1_CH, rightMotorSpeed);
    ledcWrite(RIGHT_MOTOR2_CH, 0);
  } else {
    ledcWrite(RIGHT_MOTOR1_CH, 0);
    ledcWrite(RIGHT_MOTOR2_CH, -rightMotorSpeed);
  }
}

int min(int* T) {
  int m = T[0];
  for (int i = 1; i < 6; i++) if (T[i] < m) m = T[i];
  return m;
}

int max(int* T) {
  int M = T[0];
  for (int i = 1; i < 6; i++) if (T[i] > M) M = T[i];
  return M;
}

void waitForButtonPress() {
  while (digitalRead(button) == HIGH) delay(5);
  delay(50);
  while (digitalRead(button) == LOW) delay(5);
  delay(50);
}
