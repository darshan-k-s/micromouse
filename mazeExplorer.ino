#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include "IMU.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "LIDAR.hpp"

#include "SharedMemory.hpp"
#include "MovementController.hpp"


// Motors and their encoders
mickeymouse::Motor m1(MOT1PWM,MOT1DIR);
mickeymouse::Encoder encoder1(EN1_A, EN1_B);
mickeymouse::Motor m2(MOT2PWM,MOT2DIR);
mickeymouse::Encoder encoder2(EN2_A, EN2_B);
 
// IMU
// mickeymouse::IMU imu;
mickeymouse::IMU* imu = new mickeymouse::IMU;

// LiDARs
mickeymouse::LIDAR lidarLeft(LIDAR_LEFT_EN, LIDAR_LEFT_ADDR);
mickeymouse::LIDAR lidarFront(LIDAR_FRONT_EN, LIDAR_FRONT_ADDR);
mickeymouse::LIDAR lidarRight(LIDAR_RIGHT_EN, LIDAR_RIGHT_ADDR);


// // PID controllers for both motors
mickeymouse::PIDController* pidL = new mickeymouse::PIDController(1.1, 0, 0);
mickeymouse::PIDController* pidR = new mickeymouse::PIDController(1.1, 0, 0);


// mickeymouse::PIDController* pidYaw = new mickeymouse::PIDController(1.0, 0.2, 0.3); // Works for chaining
mickeymouse::PIDController* pidYaw = new mickeymouse::PIDController(3.5, 0, 0); // Works amazing for TASK 2 


mickeymouse::PIDController* pidFront = new mickeymouse::PIDController(2, 0, 0); // Only P Zeigler-Nichols - Works for TASK 1 
// mickeymouse::PIDController* pidFront = new mickeymouse::PIDController(1.1, 0.5, 0.08); // Zeigler-Nichols - Not Great


mickeymouse::MovementController moveController;
mickeymouse::PIDController* pidLane = new mickeymouse::PIDController(0.12, 0, 0); 

mickeymouse::PIDController* pidIMU = new mickeymouse::PIDController(1, 0, 0); 
 
unsigned long lastIMUUpdate = 0;
unsigned long lastLIDARUpdate = 0;


// Auto-generated command string - Generated on 2025-08-12 14:29:44
// Command length: 9 characters
// String cmd = "fffflflfrrfl"; 


// char* cmd = "$A(28)F(54)A(-29)F(330)A(65)F(142)A(16)F(53)A(-17)F(77)A(-26)F(55)A(-24)F(266)$"; 

// Scare String
// String cmd = "$A(-20)F(-789)$"; 
// String cmd = "fffflflfffffrflfrfrflflfrfrflffrffflfrffrflfrffffffrr"; 
// String cmd = "fffflflfffffrflfrfrflf"; 

// String cmd = "fffffffffrjbgdvnjcadhfioqeahfnaoudlbaodlifhapidjaeoufuhbeuoabgaoecnajeiaffbcjdfbaojfdbajabdajldnfadfpjbjdcldnfafdkbjbouihfiaepbfoaejfbadljvbadljvbdlajvbadvjadbvjdbv";


int i = 0;

int angle = 0;
int distance = 0;
int modeInContinuous = 0;  // 1 - turn, 2 - forwards

void setup() {
  Serial.begin(9600);
  // while(!Serial);
  Wire.begin();
  
  imu->calibrate();

  lidarLeft.beginLidar();
  lidarFront.beginLidar();
  lidarRight.beginLidar();

  Serial.println(F("Initialized"));

}


// Motor 2 is Faster than 1
void loop() {


  // //////////////////////////////////////////////////////////////
  // UPDATE SENSOR DATA
  encoder1.getDistance();
  encoder2.getDistance();
  // Read IMU data
  unsigned long now = millis();
  if (now - lastIMUUpdate >= IMU_INTERVAL ) {
    lastIMUUpdate = now;
    imu->update();
  }
  // Read LiDAR data
  if (now - lastLIDARUpdate >= LIDAR_SAMPLING_PERIOD) {
    lastLIDARUpdate = now;
    lidarLeft.getDistance();
    lidarFront.getDistance();
    lidarRight.getDistance();
  }







  // If in shift movement mode
    if(mickeymouse::getShiftMode()){
      if(mickeymouse::getContinuousMode()){
        modeInContinuous = 0;

        mickeymouse::setShiftMode();

      }

      modeInContinuous = 0;
      switch(mickeymouse::wheelState.cmd){
        case 'f':
          Serial.println(F("FRONT"));
          // Shift mode logic
          mickeymouse::setChainMode(1,0,0, 0);
          ++i;
          mickeymouse::setShiftMode();
          break;

        case 'l':
          Serial.println(F("LEFT"));
          // Shift mode logic
          mickeymouse::setChainMode(0,1,0, 0);
          ++i;
          mickeymouse::setShiftMode();
          break;

        case 'r':
          Serial.println(F("RIGHT"));
          // Shift mode logic
          mickeymouse::setChainMode(0,0,1, 0);
          ++i;
          mickeymouse::setShiftMode();
          break;

        case '$':
          Serial.println(F("CONTINUOUS MODE TOGGLED"));
          mickeymouse::setContinuousMode();
          ++i;
          mickeymouse::setShiftMode();

          break;

        default:
          mickeymouse::setChainMode(0,0,0, 0);
          // Serial.println(cmd);
          break;
      }
    }

//////////////////////////////////////////////////////////////////////////

  // Stationary mode
  if (!(mickeymouse::getChainMode().front || mickeymouse::getChainMode().left || mickeymouse::getChainMode().right)){
    mickeymouse::setWheelPWM(); // Zero
  }

  // Robot driving front mode
  if(mickeymouse::getChainMode().front){
    if(moveController.state != 1){
      uint8_t counter = 1;
      pidL->zeroAndSetTarget(mickeymouse::wheelState.leftWheelLinPos, counter * 180);
      pidR->zeroAndSetTarget(mickeymouse::wheelState.rightWheelLinPos, counter * 180);
      pidLane->zeroAndSetTarget(0, 50);
      pidIMU->zeroAndSetTarget(0, 0);
    }
    moveController.moveFront(pidL, pidR, pidYaw);
  }

  // Robot turn left mode
  if(mickeymouse::getChainMode().left){
    if(moveController.state != 2){
      pidYaw->zeroAndSetTarget(0, -90);
    }
    moveController.turnLeft(pidYaw, imu);
  }

  // Robot turn right mode
  if(mickeymouse::getChainMode().right){
    if(moveController.state != 3){
      pidYaw->zeroAndSetTarget(0, 90);
    }
    moveController.turnRight(pidYaw, imu);
  }




/////////////////////////////////////////////////////////////////////////
  // Reckonings

  // Possible changes here
  if(mickeymouse::getChainMode().front){ 
    moveController.frontWallStopper();
// Get back to og heading
    if(mickeymouse::getChainMode().adjustYaw == 1){
      moveController.headingControl(pidIMU);
    }
    else{
      // Close to left wall
      if (mickeymouse::getLIDARState().distLeft < 40) {
        moveController.laneCentering(pidLane, 0);
      }
      else if(mickeymouse::getLIDARState().distRight < 40){ // Close to right wall
        moveController.laneCentering(pidLane, 1);
      }
      else{ 
        moveController.headingControl(pidIMU);
      }
    }


  }




/////////////////////////////////////////////////////////////////////////

// Actuation
  m1.setPWM(mickeymouse::getLeftPWM());
  m2.setPWM(mickeymouse::getRightPWM());
}

//
// TO DO:
// Decide if you need reckonings(IMU control andlanecentering(i.e. steer away from obstacles)) in continuous mode
// Tune for continuous mode
// Check edge cases for continuous task with small inputs, ig done
