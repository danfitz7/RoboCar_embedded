/*
  Car Control
  Daniel Fitzgerald
  2016
 */

#include "config.h"

#include <PacketSerial.h> // Using this library for Consistent Overhead Byte Stuffing (COBS) https://github.com/bakercp/PacketSerial
#include "Servo.h"


//#include <SoftwareSerial.h>  
//int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
//int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
void updateControl();
Servo steering_servo_front;
Servo steering_servo_back;
void setupControls(){
  // TODO: supply min and max pulse widths
  steering_servo_front.attach(STEERING_SERVO_FRONT_PIN);
  steering_servo_back.attach(STEERING_SERVO_BACK_PIN);
  
  pinMode(MOTOR_SPEED_PIN,OUTPUT);
  #ifdef BIDIRECTIONAL_DRIVE
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
  #endif

  updateControl();
}

SteerAngle steer_angle = 0;
DriveSpeed drive_speed = 0;
void updateControl(){
  // TODO: use .writeMicroseconds() for better steering accuracy
  steering_servo_front.write(map(steer_angle, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX, STEERING_SERVO_FRONT_MIN, STEERING_SERVO_FRONT_MAX));
  steering_servo_back.write(map(steer_angle, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX, STEERING_SERVO_BACK_MIN, STEERING_SERVO_BACK_MAX));

  if (drive_speed > 0){
    #ifdef BIDIRECTIONAL_DRIVE
      digitalWrite(MOTOR_DIRECTION_PIN, FORWARD);
    #endif
    analogWrite(MOTOR_SPEED_PIN, map(drive_speed, 0, DRIVE_SPEED_MAX, 0, 255));
  }else{
    #ifdef BIDIRECTIONAL_DRIVE
      digitalWrite(MOTOR_DIRECTION_PIN, BACKWARD);
      analogWrite(MOTOR_SPEED_PIN, map(-drive_speed, 0, DRIVE_SPEED_MAX, 0, 255));
    #else
      digitalWrite(MOTOR_SPEED_PIN, LOW);
    #endif
  }
}

#ifdef INDICATOR_LEDS
  void setupControlIndicators(){
    pinMode(LEFT_TURN_LED_PIN, OUTPUT);
    pinMode(RIGHT_TURN_LED_PIN, OUTPUT);
    pinMode(FORWARDSPEED_LED_PIN, OUTPUT);
    #ifdef BIDIRECTIONAL_DRIVE
      pinMode(BACKWARDSPEED_LED_PIN, OUTPUT);
    #endif
    updateControlIndication();
  }
  void updateControlIndication(){
    if (steer_angle >= 0){
      analogWrite(RIGHT_TURN_LED_PIN, map(steer_angle, 0, STEERING_ANGLE_MAX, 0, 255));
      digitalWrite(LEFT_TURN_LED_PIN, LOW);
    }else{
      analogWrite(LEFT_TURN_LED_PIN, map(-steer_angle, 0, STEERING_ANGLE_MAX, 0, 255));
      digitalWrite(RIGHT_TURN_LED_PIN, LOW);
    }
  
    if (drive_speed >= 0){
      analogWrite(FORWARDSPEED_LED_PIN, map(drive_speed, 0, DRIVE_SPEED_MAX, 0, 255));
      #ifdef BIDIRECTIONAL_DRIVE
        digitalWrite(BACKWARDSPEED_LED_PIN, LOW);
      #endif
    }else{
      digitalWrite(FORWARDSPEED_LED_PIN, LOW);
      #ifdef BIDIRECTIONAL_DRIVE
        analogWrite(BACKWARDSPEED_LED_PIN, map(-drive_speed, 0, DRIVE_SPEED_MAX, 0, 255));
      #endif
    }
  }
#endif

bool headlight_on = false;
void setupHeadlight(){
  pinMode(HEADLIGHT_PIN, OUTPUT);
  analogWrite(HEADLIGHT_PIN, HEADLIGHT_OFF>0?HIGH:LOW);
}
void turnHeadlightOn(){
  if (!headlight_on){
    digitalWrite(HEADLIGHT_PIN, HEADLIGHT_ON>0?HIGH:LOW);
    headlight_on = true;
  }
}
void turnHeadlightOff(){
  if (headlight_on){
    digitalWrite(HEADLIGHT_PIN, HEADLIGHT_OFF);
    headlight_on = false;
  }
}
void setHeadlight(uint8_t val){
  analogWrite(HEADLIGHT_PIN, map(val, 0, 255, HEADLIGHT_OFF, HEADLIGHT_ON));
}

// COBS encoding, 0 is packet delimiter, 1024-byte buffer size
PacketSerial_<COBS, 0, 1024> _packet_serial;
unsigned long last_command_receive_time_ms = 0;
bool received_commands = false;
void packetHandler(const uint8_t *msgBuffer, size_t buffer_size){
  if (buffer_size >= EXPECTED_COMMAND_SIZE){
      last_command_receive_time_ms = millis();
      received_commands = true;
      
      // Unpack packet to command values
      memcpy(&steer_angle, &(msgBuffer[0]), sizeof(steer_angle));
      memcpy(&drive_speed, &(msgBuffer[sizeof(steer_angle)]), sizeof(drive_speed));
  }else{
    Serial.println("WARNING: Received short packet of size: " + String(buffer_size));
  }
}
void setupSerialCommunication(){
  _packet_serial.begin(SERIAL_BAUDRATE);
  _packet_serial.setPacketHandler(*packetHandler);
}
void updateSerial(){
  _packet_serial.update();
}





void setup(){
  setupControls();
  #ifdef INDICATOR_LEDS
    setupControlIndicators();
  #endif
  setupHeadlight();
  setupSerialCommunication();
}

unsigned long last_debug_msg_time = 0;
unsigned long cur_time = 0;

void loop(){
  // Check if we have new command message packets
  updateSerial();

  cur_time = millis();
  if (cur_time - last_debug_msg_time > DEBUG_MSG_PERIOD_MS){
    Serial.println("S" + String(steer_angle) + "\tD" + String(drive_speed) + "\tH" + (headlight_on?"On":"Off"));
    last_debug_msg_time = cur_time;
  }

  // If we have a new command, process it
  if (received_commands){
    updateControl();
    
    turnHeadlightOn();
    
    #ifdef INDICATOR_LEDS
      updateControlIndication();
    #endif
    
    received_commands = false;

  // If there's no new commands...  
  }else{

    // Safety timeout!
    // If we go too long without receiving any commands, stop the car
    if (cur_time - last_command_receive_time_ms > SERIAL_UPDATE_TIMOUT_MS){
      drive_speed = 0;
      turnHeadlightOff();
      updateControl();
      #ifdef INDICATOR_LEDS
        updateControlIndication();
      #endif
    }
  }
}


