#define PRINT_COBS_DEBUG
//#define PRINT_LED_DEBUG
//#define LED_DEBUG
//#define PRINT_DEBUG

#ifdef CORE_TEENSY
  #include <PWMServo.h>
  typedef PWMServo MotorServo;
#else
  #include <Servo.h>
  typedef Servo MotorServo;
#endif

#define FRONT_STEERING_SERVO_PWM_MIN_US 1000
#define FRONT_STEERING_SERVO_PWM_MAX_US 2000
#define FRONT_STEERING_SERVO_ANGLE_NEUTRAL_DEG 90
#define FRONT_STEERING_SERVO_ANGLE_RANGE_DEG 60
#define FRONT_STEERING_SERVO_PIN 23
MotorServo frontSteeringServo;

#define BACK_STEERING_SERVO_PWM_MIN_US 1000
#define BACK_STEERING_SERVO_PWM_MAX_US 2000
#define BACK_STEERING_SERVO_ANGLE_NEUTRAL_DEG 90
#define BACK_STEERING_SERVO_ANGLE_RANGE_DEG 90
#define BACK_STEERING_SERVO_PIN 22
MotorServo backSteeringServo;
//#define INVERT_BACK_STEERING

#define DRIVE_MOTOR_SPEED_PIN 5
#define DRIVE_MOTOR_DIR_PIN 6
#define RAGEBRIDGE_PWM_MIN_US 1100
#define RAGEBRIDGE_PWM_MAX_US 1900
#define DRIVE_MOTOR_SERVO_ANGLE_NEUTRAL_DEG 90.0f   // standard servo goes 0 through 180, so midpoint is 90 and range away from midpoint is 90
#define DRIVE_MOTOR_SERVO_ANGLE_RANGE_DEG 90.0f
MotorServo driveMotorSpeedServo;
MotorServo driveMotorCH2Servo;

#define EYE_LED_PIN 20

// Abstract current speeds (abstract units between physical min and max, [-1.0f, 1.0f])
double drive_speed = 0.0f;
double turn_speed = 0.0f;

// Position Error
double distance_error_mm = 0.0f; // Speed up or slow down to adjust distance to target
double angle_error_degrees = 0.0f;   // turn left/right to adjust angle to target




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// COBs Serial //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Packet serial communication via COBS encoded messages
#define ADDRESS_POLL_LOOP_PERIOD_MS 200
#define ACK "ACK"
#define TEST "TEST"
#define DRIVE_ARDUINO_ADDRESS 255
#include "PacketSerial.h"
#define SERIAL_BAUD 9600
PacketSerial packetSerial;
#define DISCONNECT_TIMEOUT_MS 5000 // Assume disconnection after this much time, in ms.
//bool idleAndWait = true;

// NOTE: Could be physically dangerous for a mobile robot to keep driving too long without feedback.
unsigned long long lastCommandTime = 0;
bool newValuesReceived = false;

// We receive 2 numbers, (distance error, angle error, each 2 bytes)
#define BYTES_PER_DRIVE_MESSAGE 4
#define COMMUNICATION_INT16_MAX 32767.0f
const double COMMUNICATION_DRIVE_UNIT_TO_MM = 1.0;                                        // We communicate distance in units of 1mm (gives us 32.77m/100ft before overflow
const double COMMUNICATION_ROTATION_UNIT_TO_DEGREES = (180.0f / COMMUNICATION_INT16_MAX);  // we communicate angles in units such that INT_16_MAX = PI/180 and INT_16_MIN = -PI/180

void packetHandler (const uint8_t* buf, size_t s) {

  #ifdef PRINT_COBS_DEBUG
    String debugString ="Drive<";
  #endif

  if (s >= BYTES_PER_DRIVE_MESSAGE) {
    int16_t valDistance = 0;
    int16_t valAngle = 0;

    valDistance |= buf[0] << 8;
    valDistance |= buf[1];
    valAngle |= buf[2] << 8;
    valAngle |= buf[3];
    
    distance_error_mm = (double)valDistance * COMMUNICATION_DRIVE_UNIT_TO_MM;
    angle_error_degrees =  (double)(valAngle * COMMUNICATION_ROTATION_UNIT_TO_DEGREES);

    #ifdef PRINT_DRIVE_DEBUG
    debugString += String(distance_error_mm);
    debugString += '\t';
    debugString += String(angle_error_degrees);
    debugString += '>';
    Serial.println(debugString);
    #endif

    newValuesReceived = true;
    lastCommandTime = millis();
  }
}
void setupSerial(){
  Serial.begin(9600);
  packetSerial.setPacketHandler(packetHandler);
  packetSerial.begin(SERIAL_BAUD);
}

void spamID() {
  stopAll();

  String input_buffer = "";
  boolean received_ack = false;

  uint8_t address = DRIVE_ARDUINO_ADDRESS;

  // Keep looping until we receive the characters of ACK in succession.
  while (!received_ack) {

    // Read and print our physical address.
    Serial.println(address);

    while (Serial.available()) { // If there are bytes to process, we process one at a time.
      char c = Serial.read();
      input_buffer += c;
      //     if (input_buffer.length() > 3) {   // We only need the last three characters received
      //       input_buffer = input_buffer.substring(input_buffer.length()-3);
      //     }
      //     if (input_buffer.equals(ACK)) {
      if (input_buffer.endsWith(ACK)) {
        received_ack = true;
        while (Serial.available()) {
          Serial.read();
        }
        break;
      } else if (input_buffer.endsWith(TEST)) {
        testAll();

        while (Serial.available()) {
          Serial.read();
        }
        break;
      }
    }

    toggle_orange_led();
    delay(ADDRESS_POLL_LOOP_PERIOD_MS);
  }
  Serial.println("ARDUINO");
  startPIDs();
  
//  idleAndWait = false;
  lastCommandTime = millis();
}

// Orange LED - used to indicate serial communication
#define ORANGE_LED_PIN 13
bool orange_led_on = false;
void setup_orange_led() {
  pinMode(ORANGE_LED_PIN, OUTPUT);
  digitalWrite(ORANGE_LED_PIN, LOW);
  orange_led_on = false;
}
void toggle_orange_led() {
  if (orange_led_on) {
    digitalWrite(ORANGE_LED_PIN, LOW);
    orange_led_on = false;
  } else {
    digitalWrite(ORANGE_LED_PIN, HIGH);
    orange_led_on = true;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// PIDs /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Following PID 
#include <PID_v1.h>
#define PID_SAMPLE_TIME_MS 17 // 60fps = 17ms/frame

// Position/drive/speed control
double distance_p = 0.0005f;
double distance_i = 0.000f;
double distance_d = 0.000f;//0.0001 still too fast
double distance_setpoint_mm = 0.0f; // The actual following goal distance is already subtracted from the tracker distance in Unity 

// Angle/rotation/turning control
double angle_p = 0.05f;
double angle_i = 0.000f;
double angle_d = 0.000f;
double angle_setpoint_deg = 0.0f;

// PID Controllers
PID speedPID(&distance_error_mm, &drive_speed, &distance_setpoint_mm, distance_p, distance_i, distance_d, DIRECT);
PID steerPID(&angle_error_degrees, &turn_speed, &angle_setpoint_deg, angle_p, angle_i, angle_d, DIRECT);
void setupPIDs() {
  speedPID.SetOutputLimits(-1.0f, 1.0f);
  speedPID.SetSampleTime(PID_SAMPLE_TIME_MS);

  steerPID.SetOutputLimits(-1.0f, 1.0f);
  steerPID.SetSampleTime(PID_SAMPLE_TIME_MS);
}
void startPIDs() {
  speedPID.SetMode(AUTOMATIC);
  steerPID.SetMode(AUTOMATIC);
}
void updatePIDs(){
  speedPID.Compute();
  steerPID.Compute();
  newValuesReceived = false;
}
void stopPIDs() {
  speedPID.SetMode(MANUAL);
  steerPID.SetMode(MANUAL);
  newValuesReceived = false;
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
  driveMotorCH2Servo.write(DRIVE_MOTOR_SERVO_ANGLE_NEUTRAL_DEG); // appease the RAGEBRIDGE by giving it a signalon CH2
}

void setup() {
  setupSerial();

  Serial.println("ROBOCAR\nSetup...");
  
  setupSteering();
  setupEye();
  setupDriveMotor();
  setupPIDs();

  #ifdef LED_DEBUG
    setupDebugLEDs();
  #endif

  drive(0.0f);
  turn(0.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// DEBUG ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define FORWARD_LED_PIN 11
#define BACKWARD_LED_PIN 3
#define RIGHT_LED_PIN 6
#define LEFT_LED_PIN 5
#define DEBUG_LED_MAX_FOLLOW_ERROR_MM 800.0f
#define DEBUG_LED_MAX_TURN_ANGLE_DEG 25.0f
void setDebugLEDPair(double value, double maxValue, int posativeLEDpin, int negativeLEDpin){
  int led_val = (int)constrain(0.5f+abs(255.0f * (value/maxValue)), 0.0, 255.0);

  if (value >= 0.0f){
    analogWrite(posativeLEDpin, led_val);
    analogWrite(negativeLEDpin, 0);
  }else{
    analogWrite(negativeLEDpin, led_val);
    analogWrite(posativeLEDpin, 0);
  }
}
void setupDebugLEDs(){
  pinMode(FORWARD_LED_PIN, OUTPUT);
  digitalWrite(FORWARD_LED_PIN, LOW);
  
  pinMode(BACKWARD_LED_PIN, OUTPUT);
  digitalWrite(BACKWARD_LED_PIN, LOW);
  
  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(RIGHT_LED_PIN, LOW);
  
  pinMode(LEFT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, LOW);
}
void updateDebugLEDs(){
  // Shows the position error
  setDebugLEDPair(distance_error_mm, DEBUG_LED_MAX_FOLLOW_ERROR_MM, FORWARD_LED_PIN, BACKWARD_LED_PIN);
  setDebugLEDPair(angle_error_degrees, DEBUG_LED_MAX_TURN_ANGLE_DEG, RIGHT_LED_PIN, LEFT_LED_PIN);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// DRIVING //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drive(double drive_velocity){
  driveMotorSpeedServo.write(DRIVE_MOTOR_SERVO_ANGLE_NEUTRAL_DEG + (drive_velocity * DRIVE_MOTOR_SERVO_ANGLE_RANGE_DEG));//mapf(s, DRIVE_MOTOR_SERVO_MIN_ANGLE_DEG, DRIVE_MOTOR_SERVO_MAX_ANGLE_DEG));
}

void turn(double turn_velocity){
  frontSteeringServo.write(FRONT_STEERING_SERVO_ANGLE_NEUTRAL_DEG + (turn_velocity * FRONT_STEERING_SERVO_ANGLE_RANGE_DEG));
  backSteeringServo.write(BACK_STEERING_SERVO_ANGLE_NEUTRAL_DEG 
  #ifdef INVERT_BACK_STEERING
  - 
  #else
  +
  #endif
  (turn_velocity * BACK_STEERING_SERVO_ANGLE_RANGE_DEG));
}

void updateDriving(){
  #ifdef PRINT_DEBUG
    Serial.println("D" + String(drive_speed) + "\t" + String(turn_speed));
  #endif
  drive(drive_speed);
  turn(turn_speed);
}

void stopAll(){
  distance_error_mm = 0.0f;
  angle_error_degrees = 0.0f;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// TESTS ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void testServo(MotorServo* servo, int neutral_angle = 90, int angle_range=90){
  Serial.println("\tTest servo...");
  int deg = 0;
  int min_angle = neutral_angle - angle_range;
  int max_angle = neutral_angle + angle_range;
  
  for (deg = neutral_angle; deg<=max_angle;deg++){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
  delay(1000);
  
  for (deg = max_angle; deg>=neutral_angle;deg--){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
  delay(1000);

  for (deg = neutral_angle; deg>=min_angle;deg--){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
  delay(1000);

  for (deg=min_angle;deg<=neutral_angle;deg++){
    servo->write(deg);
    Serial.print('\t');
    Serial.println(deg);
    delay(100);
  }
}

void testFrontSteering(){
  Serial.println("Test front steering...");
  testServo(&frontSteeringServo, FRONT_STEERING_SERVO_ANGLE_NEUTRAL_DEG, FRONT_STEERING_SERVO_ANGLE_RANGE_DEG);
}
void testBackSteering(){
  Serial.println("Test back steering...");
  testServo(&backSteeringServo, BACK_STEERING_SERVO_ANGLE_NEUTRAL_DEG, BACK_STEERING_SERVO_ANGLE_RANGE_DEG);
}
void testTurn(){
  double steerAngle = 0.0f;
  const double inc = 0.05;
  const int delayTimeMs = 10;
  Serial.println("Steering Test...");
  for (steerAngle = 0.0f; steerAngle <= 1.0f; steerAngle += inc){
    turn(steerAngle);
    delay(delayTimeMs);
    Serial.println("\t" + String(steerAngle));
  }
  delay(1000);
  for (steerAngle = 1.0f; steerAngle >=0.0f; steerAngle -= inc){
    turn(steerAngle);
    delay(delayTimeMs);
    Serial.println("\t" + String(steerAngle));
  }
  delay(1000);
  for (steerAngle = 0.0f; steerAngle >=-1.0f; steerAngle -= inc){
    turn(steerAngle);
    delay(delayTimeMs);
    Serial.println("\t" + String(steerAngle));
  }
  delay(1000);
    for (steerAngle = -1.0f; steerAngle <= 0.0f; steerAngle += inc){
    turn(steerAngle);
    delay(delayTimeMs);
    Serial.println("\t" + String(steerAngle));
  }
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
  testFrontSteering();
  testBackSteering();
  testTurn();
  //testEye();
  //testDriveMotor();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// LOOP ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // receive new commands
  packetSerial.update();
  
  //testAll();

  // Communication disruption timout
  if (millis() - lastCommandTime >= DISCONNECT_TIMEOUT_MS) {
//    if (!idleAndWait){
//     idleAndWait = true;
      spamID();
//    }
  }else{
//    if (!idleAndeWait){
      if (newValuesReceived){
        // No point calculating anything if we don't have feedback
        //  if (received_feedback){
        // calculate target speeds for acceleration/deceleration
        updatePIDs();
        #ifdef LED_DEBUG
          updateDebugLEDs();
        #endif
      
        updateDriving();
    }
//    }
  }

  //delay(10);
}
