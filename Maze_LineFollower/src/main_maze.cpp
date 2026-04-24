#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <NewPing.h>

// --- PINS CAPTEURS ULTRASONS ---
#define TRIGGER_PINL  33
#define ECHO_PINL     14
#define TRIGGER_PINF  27
#define ECHO_PINF     26
#define TRIGGER_PINR  25
#define ECHO_PINR     32
#define button 12
#define led 2

#define MAX_DISTANCE 100

// --- DIRECTIONS ---
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

// --- CONSTANTES PID GLOBALES (Pour 2 murs) ---
float P = 12.5; 
float D = 0.5;
float I = 0.0;
float oldErrorP;
float totalError; 
float errorI = 0;

// --- SEUILS ET DISTANCES CIBLES ---
float wall_threshold = 15.1;
float front_threshold = 12.50;
float target_distance_single = 10.0;
float stuck_threshold = 2.0; // Seuil critique pour détecter un blocage

boolean frontwall, leftwall, rightwall;

// --- GYRO OFFSET ---
float gyroOffsetZ = 0.0; 

// --- PINS MOTEURS ---
#define leftMotor1 16  
#define leftMotor2 17  
#define rightMotor1 19 
#define rightMotor2 18 

#define LEFT_MOTOR1_CH 0
#define LEFT_MOTOR2_CH 1
#define RIGHT_MOTOR1_CH 2
#define RIGHT_MOTOR2_CH 3
#define PWM_FREQ 1000
#define PWM_RES 8 

int baseSpeed = 100; 

// --- OBJETS ---
NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

MPU6050 mpu(Wire);

float oldLeftSensor = 0, oldRightSensor = 0, oldFrontSensor = 0;
float leftSensor = 0, rightSensor = 0, frontSensor = 0;

// Function prototypes
void setup();
void loop();
void move(int dir, int speedL, int speedR);
void pid_start();
void pid_single_wall(boolean followLeft);
void ReadSensors();
void walls();
void turnWithGyro(float angleDelta);
void waitForButtonPress();
void unjamRobot(); 

// ==============================================================================
// SETUP
// ==============================================================================
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

  pinMode(TRIGGER_PINL, OUTPUT);
  pinMode(ECHO_PINL, INPUT);
  pinMode(TRIGGER_PINR, OUTPUT);
  pinMode(ECHO_PINR, INPUT);
  pinMode(TRIGGER_PINF, OUTPUT);
  pinMode(ECHO_PINF, INPUT);

  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Wire.begin();
  byte status = mpu.begin();
  if(status != 0) {
    while(1); 
  }
  
  delay(1000);
  mpu.calcOffsets(); 

  waitForButtonPress();
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================
void loop() {
  mpu.update(); 
  ReadSensors();
  walls();

  // --- STUCK / UNJAM LOGIC ---
  if (frontSensor < stuck_threshold || leftSensor < stuck_threshold || rightSensor < stuck_threshold) {
    unjamRobot();
  }
  else if (frontwall) {
    move(STOP, 0, 0);
    delay(200);    
    ReadSensors();   
    walls();
    
    if (leftwall && rightwall) {
       move(STOP, 0, 0); delay(50);
       turnWithGyro(180.0); 
    } 
    else if (leftwall && !rightwall) {
       move(STOP, 0, 0); delay(50);
       turnWithGyro(-90.0); 
    } 
    else if (rightwall && !leftwall) {
       move(STOP, 0, 0); delay(50);
       turnWithGyro(90.0);  
    } 
    else {
       move(STOP, 0, 0); delay(50);
       turnWithGyro(-85.0); 
    }

    errorI = 0;
    oldErrorP = 0;
  } 
  else {
    delay(30); 
    ReadSensors();
    
    if (!rightwall) {
      pid_single_wall(false);
      errorI = 0;
      oldErrorP = 0;
    } 
    else {
      if (leftwall && rightwall) {
        pid_start(); 
      } 
      else if (rightwall && !leftwall) {
        pid_single_wall(false); 
      } 
      else if (!rightwall && leftwall) {
        pid_single_wall(true); 
      } 
      else {
        pid_start();
      }
    }
  }
}

// ==============================================================================
// UNJAM ROBOT (SMART ESCAPE)
// ==============================================================================
void unjamRobot() {
  boolean stuckLeft = (leftSensor < stuck_threshold);
  boolean stuckRight = (rightSensor < stuck_threshold);
  
  move(STOP, 0, 0);
  delay(100);
  
  move(BACKWARD, baseSpeed, baseSpeed);
  delay(100); 
  
  move(STOP, 0, 0);
  delay(100);

  if (stuckLeft && !stuckRight) {
    move(RIGHT, baseSpeed, baseSpeed);
    delay(150); 
  } 
  else if (stuckRight && !stuckLeft) {
    move(LEFT, baseSpeed, baseSpeed);
    delay(150); 
  }

  move(FORWARD, baseSpeed, baseSpeed);
  delay(250); 

  errorI = 0;
  oldErrorP = 0;
}

// ==============================================================================
// GYRO TURNING FUNCTION
// ==============================================================================
void turnWithGyro(float angleDelta) {
  float anticipationOffset = 10.0;
  float targetAngle = abs(angleDelta) - anticipationOffset;
  float accumulated = 0.0;
  unsigned long lastTime = micros();
  unsigned long turnStartTime = millis();
  int turnDir = (angleDelta > 0) ? LEFT : RIGHT;

  while (true) {
    mpu.update();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    
    accumulated += abs(mpu.getGyroZ()) * dt;
    
    if (accumulated >= targetAngle) {
      move(STOP, 0, 0);
      break;
    }

    if (millis() - turnStartTime > 5000) {
      move(STOP, 0, 0);
      break;
    }

    float remaining = targetAngle - accumulated;
    int turnSpeed = constrain(remaining * 1.8 + 65, 80, 120);
    move(turnDir, turnSpeed, turnSpeed);
  }

  delay(300); 
  gyroOffsetZ = 0; 
}

// ==============================================================================
// PID (Centrage entre DEUX murs)
// ==============================================================================
void pid_start() {
  float errorP = leftSensor - rightSensor;
  float errorD = errorP - oldErrorP;
  errorI = (2.0 / 3.0) * errorI + errorP;
  totalError = P * errorP + D * errorD + I * errorI;
  oldErrorP = errorP;
  
  int speedR = constrain(baseSpeed + totalError, 0, 255);
  int speedL = constrain(baseSpeed - totalError, 0, 255);

  move(FORWARD, speedL, speedR);
}

// ==============================================================================
// PID (Suivi d'UN SEUL mur) - SMOOTHED & CAPPED
// ==============================================================================
void pid_single_wall(boolean followLeft) {
  float errorP;
  
  // 1. THE BLINDERS: Ignore wild readings above 20cm so it doesn't panic
  float safeLeft = min(leftSensor, 20.0f);   
  float safeRight = min(rightSensor, 20.0f); 

  if (followLeft) {
    errorP = safeLeft - target_distance_single;
  } else {
    errorP = target_distance_single - safeRight;
  }

  float errorD = errorP - oldErrorP;
  errorI = (2.0 / 3.0) * errorI + errorP;
  
  // 2. ISOLATED PID CONSTANTS (Softer values just for single wall)
  float p_single = 5.5; //5.5
  float d_single = 0.5; 
  
  totalError = (p_single * errorP) + (d_single * errorD) + (I * errorI);
  oldErrorP = errorP;
  
  // 3. THE STEERING CAP: Prevents rapid spinning
  totalError = constrain(totalError, -40, 40);
  
  // 4. APPLY TO MOTORS
  int speedR = constrain(baseSpeed + totalError, 50, 150); 
  int speedL = constrain(baseSpeed - totalError, 50, 150);

  move(FORWARD, speedL, speedR);
}

// ==============================================================================
// LECTURE DES CAPTEURS
// ==============================================================================
void ReadSensors() {
  float l = sonarLeft.ping_cm();
  float r = sonarRight.ping_cm();
  float f = sonarFront.ping_cm();

  if (l == 0) l = MAX_DISTANCE;
  if (r == 0) r = MAX_DISTANCE;
  if (f == 0) f = MAX_DISTANCE;

  leftSensor = (l + oldLeftSensor) / 2;
  rightSensor = (r + oldRightSensor) / 2;
  frontSensor = (f + oldFrontSensor) / 2;

  oldLeftSensor = leftSensor;
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;
}

void walls() {
  leftwall = (leftSensor < wall_threshold);
  rightwall = (rightSensor < wall_threshold);
  frontwall = (frontSensor < front_threshold);
}

// ==============================================================================
// CONTROLE DES MOTEURS
// ==============================================================================
void move(int dir, int speedL, int speedR) {
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  switch (dir) {
    case FORWARD:
      ledcWrite(LEFT_MOTOR1_CH, speedL);
      ledcWrite(LEFT_MOTOR2_CH, 0);
      ledcWrite(RIGHT_MOTOR1_CH, speedR); ledcWrite(RIGHT_MOTOR2_CH, 0);
      break;
    case BACKWARD:
      ledcWrite(LEFT_MOTOR1_CH, 0); ledcWrite(LEFT_MOTOR2_CH, speedL);
      ledcWrite(RIGHT_MOTOR1_CH, 0);
      ledcWrite(RIGHT_MOTOR2_CH, speedR);
      break;
    case LEFT: 
      ledcWrite(LEFT_MOTOR1_CH, 0); ledcWrite(LEFT_MOTOR2_CH, speedL);
      ledcWrite(RIGHT_MOTOR1_CH, speedR); ledcWrite(RIGHT_MOTOR2_CH, 0);
      break;
    case RIGHT: 
      ledcWrite(LEFT_MOTOR1_CH, speedL); ledcWrite(LEFT_MOTOR2_CH, 0);
      ledcWrite(RIGHT_MOTOR1_CH, 0); ledcWrite(RIGHT_MOTOR2_CH, speedR);
      break;
    case STOP:
      ledcWrite(LEFT_MOTOR1_CH, 0); ledcWrite(LEFT_MOTOR2_CH, 0);
      ledcWrite(RIGHT_MOTOR1_CH, 0); ledcWrite(RIGHT_MOTOR2_CH, 0);
      break;
  }
}

// ==============================================================================
// BOUTON DE DÉMARRAGE
// ==============================================================================
void waitForButtonPress() {
  digitalWrite(led, HIGH);
  while (digitalRead(button) == HIGH) delay(10); 
  while (digitalRead(button) == LOW) delay(10);  
  digitalWrite(led, LOW);
  delay(500); 
}