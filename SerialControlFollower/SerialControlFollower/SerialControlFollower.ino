#include <PWMServo.h>

#define FRONT_STEERING_SERVO_PWM_MIN_US 9000
#define FRONT_STEERING_SERVO_PWM_MAX_US 2100
#define FRONT_STEERING_SERVO_MIN_ANGLE_DEG 30
#define FRONT_STEERING_SERVO_MAX_ANGLE_DEG 90
const int FRONT_STEERING_SERVO_NEUTRAL_ANGLE_DEG =(FRONT_STEERING_SERVO_MAX_ANGLE_DEG+FRONT_STEERING_SERVO_MIN_ANGLE_DEG)/2;
#define FRONT_STEERING_SERVO_PIN 23
PWMServo frontSteeringServo;

#define BACK_STEERING_SERVO_PWM_MIN_US 1000
#define BACK_STEERING_SERVO_PWM_MAX_US 2000
#define BACK_STEERING_SERVO_MIN_ANGLE_DEG 50
#define BACK_STEERING_SERVO_MAX_ANGLE_DEG 130
const int BACK_STEERING_SERVO_NEUTRAL_ANGLE_DEG =(BACK_STEERING_SERVO_MAX_ANGLE_DEG+BACK_STEERING_SERVO_MIN_ANGLE_DEG)/2;
#define BACK_STEERING_SERVO_PIN 22
PWMServo backSteeringServo;

#define DRIVE_MOTOR_SPEED_PIN 5
#define DRIVE_MOTOR_DIR_PIN 6
#define RAGEBRIDGE_PWM_MIN_US 1100
#define RAGEBRIDGE_PWM_MAX_US 1900
#define DRIVE_MOTOR_SERVO_MIN_ANGLE_DEG 0
#define DRIVE_MOTOR_SERVO_MAX_ANGLE_DEG 180
PWMServo driveMotorSpeedServo;
PWMServo driveMotorCH2Servo;

#define EYE_LED_PIN 20

// TODO: what our our physical min/max turning angles?
#define STEERING_ANGLE_MIN_DEG -45.0f
#define STEERING_ANGLE_MAX_DEG 45.0f

// Abstract current speeds
double drive_speed_deg = 0.0f; // between 0 and 180 (like degrees)
double turn_speed_deg = 0.0f; // actual degrees, also between STEERING_ANGLE_MIN_DEG and STEERING_ANGLE_MAX_DEG

// Position Error
double distance_error_mm = 0.0f; // Speed up or slow down to adjust distance to target
double angle_error_deg = 0.0f;   // turn left/right to adjust angle to target



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// PIDs /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Following PID 
#include <PID_v1.h>
#define PID_SAMPLE_TIME_MS 17 // 60fps = 17ms/frame

// Position/drive/speed control
double distance_p = 0.1f;
double distance_i = 0.000f;
double distance_d = 0.000f;//0.0001 still too fast
double distance_setpoint_mm = 1000.0f;  // THIS IS HOW FAR THE ROBOT TRIES TO FOLLOW BEHIND US

// Angle/rotation/turning control
double angle_p = 0.1f;
double angle_i = 0.000f;
double angle_d = 0.000f;
double angle_setpoint_deg = 0.0f; // THIS IS WHAT ANGLE THE ROVOT TRIES TO MAKE WITH US

// We receive 2 numbers, (distance error, angle error, each 2 bytes)
#define BYTES_PER_DRIVE_MESSAGE 4
#define COMMUNICATION_INT16_MAX 32767.0f
const double COMMUNICATION_DRIVE_UNIT_TO_MM = 0.1;  // We communicate in units of 100um
const double COMMUNICATION_ROTATION_UNIT_TO_RADIANS = (PI / COMMUNICATION_INT16_MAX); // we communicate in units such that INT_16_MAX = PI and INT_16_MIN = -180

// PID Controllers
PID speedPID(&distance_error_mm, &drive_speed_deg, &distance_setpoint_mm, distance_p, distance_i, distance_d, DIRECT);
PID steerPID(&angle_error_deg, &turn_speed_deg, &angle_setpoint_deg, angle_p, angle_i, angle_d, DIRECT);
void setupPIDs() {
  speedPID.SetOutputLimits(DRIVE_MOTOR_SERVO_MIN_ANGLE_DEG, DRIVE_MOTOR_SERVO_MAX_ANGLE_DEG);
  speedPID.SetSampleTime(PID_SAMPLE_TIME_MS);

  steerPID.SetOutputLimits(STEERING_ANGLE_MIN_DEG, STEERING_ANGLE_MAX_DEG);
  steerPID.SetSampleTime(PID_SAMPLE_TIME_MS);
}
void startPIDs() {
  speedPID.SetMode(AUTOMATIC);
  steerPID.SetMode(AUTOMATIC);
}
void updatePIDs(){
  speedPID.Compute();
  steerPID.Compute();
}
void stopPIDs() {
  speedPID.SetMode(MANUAL);
  steerPID.SetMode(MANUAL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// COBs Serial //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Packet serial communication via COBS encoded messages
#include "PacketSerial.h"
#define SERIAL_BAUD 9600
PacketSerial packetSerial;
#define DISCONNECT_TIMEOUT_MS 1000 // Assume disconnection after this much time, in ms.
// NOTE: Could be physically dangerous for a mobile robot to keep driving too long without feedback.
unsigned long lastCommandTime = 0;

void packetHandler (const uint8_t* buf, size_t s) {
  lastCommandTime = millis();

#ifdef PRINT_DRIVE_DEBUG
  Serial.print("Drive<");
#endif

  if (s >= BYTES_PER_DRIVE_MESSAGE) {
    //    for (size_t i=0;i<s;i++){
    //      Serial.print(buf[i], HEX);
    //      Serial.print('-');
    //    }

    if (s == BYTES_PER_DRIVE_MESSAGE) {
      int16_t valDistance = 0;
      int16_t valAngle = 0;
      // val |= buf[0];
      // val |= buf[1] >> 8;
      valDistance |= buf[0] << 8; // used to be valX
      valDistance |= buf[1];
      valAngle |= buf[2] << 8; // used to be valY
      valAngle |= buf[3];
      //      Serial.print('(');
      //      Serial.print((int)valX);
      //      Serial.print(')');

      distance_error_mm = (double)valDistance * COMMUNICATION_DRIVE_UNIT_TO_MM;
      angle_error_deg = (double)valAngle * COMMUNICATION_ROTATION_UNIT_TO_RADIANS;

#ifdef PRINT_DRIVE_DEBUG
      Serial.print(x_error_mm);
      Serial.print('\t');
      Serial.print(y_error_mm);
      Serial.print('\t');
      Serial.print(r_error_radians * 180.0f / PI);
#endif
    }
  }

#ifdef PRINT_DRIVE_DEBUG
  Serial.println(">");
#endif
}

void setupSerial(){
  Serial.begin(9600);
  packetSerial.setPacketHandler(packetHandler);
  packetSerial.begin(SERIAL_BAUD);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// SETUP ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setupSteering(){
  frontSteeringServo.attach(FRONT_STEERING_SERVO_PIN, FRONT_STEERING_SERVO_PWM_MIN_US, FRONT_STEERING_SERVO_PWM_MAX_US);
  backSteeringServo.attach(BACK_STEERING_SERVO_PIN, BACK_STEERING_SERVO_PWM_MIN_US, BACK_STEERING_SERVO_PWM_MAX_US);
}

void setupEye(){
  pinMode(EYE_LED_PIN, OUTPUT);
  digitalWrite(EYE_LED_PIN, LOW);
}

void setupDriveMotor(){
  driveMotorSpeedServo.attach(DRIVE_MOTOR_SPEED_PIN, RAGEBRIDGE_PWM_MIN_US, RAGEBRIDGE_PWM_MAX_US);
  driveMotorCH2Servo.attach(DRIVE_MOTOR_DIR_PIN, RAGEBRIDGE_PWM_MIN_US, RAGEBRIDGE_PWM_MAX_US);
  driveMotorCH2Servo.write(90); // appease the RAGEBRIDGE by giving it a signalon CH2
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  Serial.println("ROBOCAR\nSetup...");
  setupSteering();
  setupEye();
  setupDriveMotor();
  setupPIDs();
}

// maps a float between [-1.0f, 1.0f] to in int in the given range
//int mapf(float f, int out_min, int out_max){
//  return out_min + int( (out_max - out_min) * ((f - (-1.0f) )/(2.0f) ));
//}

void drive(float s_deg){
  driveMotorSpeedServo.write(s_deg);//mapf(s, DRIVE_MOTOR_SERVO_MIN_ANGLE_DEG, DRIVE_MOTOR_SERVO_MAX_ANGLE_DEG));
}

void turn(float s_deg){
  frontSteeringServo.write(map(s_deg, STEERING_ANGLE_MIN_DEG, STEERING_ANGLE_MAX_DEG, FRONT_STEERING_SERVO_MIN_ANGLE_DEG, FRONT_STEERING_SERVO_MAX_ANGLE_DEG));
  frontSteeringServo.write(map(s_deg, STEERING_ANGLE_MIN_DEG, STEERING_ANGLE_MAX_DEG, BACK_STEERING_SERVO_MIN_ANGLE_DEG, BACK_STEERING_SERVO_MAX_ANGLE_DEG));
}

void updateDriving(){
  drive(drive_speed_deg);
  turn(turn_speed_deg);
}

void stopAll(){
  distance_error_mm = 0.0f;
  angle_error_deg = 0.0f;
  updatePIDs();
  
  drive_speed_deg = 0.0f;
  turn_speed_deg = 0.0f;
  updateDriving();
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// TESTS ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void testServo(PWMServo* servo, int min_angle=0, int max_angle=180, int neutral_angle = 90){
  Serial.println("\tTest servo...");
  int deg = 0;
  for (deg = neutral_angle; deg>=min_angle;deg--){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
  delay(1000);
  
  for (deg = min_angle; deg<=max_angle;deg++){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
  delay(1000);
  
  for (deg=max_angle;deg>=neutral_angle;deg--){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
}

void testFrontSteering(){
  Serial.println("Test front steering...");
  testServo(&frontSteeringServo, FRONT_STEERING_SERVO_MIN_ANGLE_DEG, FRONT_STEERING_SERVO_MAX_ANGLE_DEG, FRONT_STEERING_SERVO_NEUTRAL_ANGLE_DEG);
}
void testBackSteering(){
  Serial.println("Test back steering...");
  testServo(&backSteeringServo, BACK_STEERING_SERVO_MIN_ANGLE_DEG, BACK_STEERING_SERVO_MAX_ANGLE_DEG, BACK_STEERING_SERVO_NEUTRAL_ANGLE_DEG);
}
void testDriveMotor(){
  int inc = 1;
  Serial.println("Test drive motor...");
  testServo(&driveMotorSpeedServo);
}

void testLEDFade(int LEDPin, int inc=4){
  int brightness = 0;
  Serial.println("\tTesting LED...");
  for (brightness = 0; brightness<255;brightness+=inc){
    analogWrite(LEDPin, brightness);
    delay(100);
    Serial.print('\t');
    Serial.println(brightness);
  }
  delay(1000);
  for (brightness = 255; brightness>=0;brightness-=inc){
    analogWrite(LEDPin, brightness);
    delay(100);
    Serial.print('\t');
    Serial.println(brightness);
  }
}
void testEye(){
  Serial.println("Test eye");
  testLEDFade(EYE_LED_PIN);
}
void testAll(){
  //testFrontSteering();
  //testBackSteering();
  //testEye();
  testDriveMotor();
}

bool idleAndWait = true;
void loop() {
  //testAll();

  // Communication disruption timout
  if (millis() - lastCommandTime >= DISCONNECT_TIMEOUT_MS) {
    if (!idleAndWait){
      stopAll();
      idleAndWait = true;
    }
  }else{
    idleAndWait = false;
  }

  // receive new commands
  packetSerial.update();

  // No point calculating anything if we don't have feedback
  //  if (received_feedback){
  // calculate target speeds for acceleration/deceleration
  updatePIDs();

  updateDriving();

  delay(10);

}
