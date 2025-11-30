#include "HardwareSerial.h"
#pragma once

#include "math.h"


namespace mickeymouse {
 
class MovementController{

public:
  // Using default constructor

  // Change up logic
  void laneCentering(mickeymouse::PIDController* pidLane, bool dir){
    //
    //  0 - LEFT
    //  1 - RIGHT
    //
    // 30 mm to left wall
    if(!dir){ // LEFT
      float error = getLIDARState().distLeft - 40;

      float centeringEffort = pidLane->compute(error);
      mickeymouse::setWheelPWM(getLeftPWM() + centeringEffort, getRightPWM() - centeringEffort);
    }
    else{
      float error = getLIDARState().distRight - 40;

      float centeringEffort = pidLane->compute(error);
      mickeymouse::setWheelPWM(getLeftPWM() - centeringEffort, getRightPWM() + centeringEffort);
      // mickeymouse::setWheelPWM(centeringEffort * -2, centeringEffort * 2);

      Serial.print("DIST================"); Serial.println(getLIDARState().distRight);
      Serial.print("CENTERING ERROR================"); Serial.println(error);
      Serial.print("C EFFORT================"); Serial.println(centeringEffort); 
    }



  }

  // Useful to minimise translational errors
  void frontWallStopper(){
    // Front wall stop condition at 50mm
    // Serial.print("Lidar FRONT======================"); Serial.println(mickeymouse::lidarState.distFront);
    if(mickeymouse::getLIDARState().distFront <= 50){
      Serial.println("WALLLLLLL");
      mickeymouse::setWheelPWM(); // Set PWM to 0
      Serial.println(F("Shifting MODE"));
      mickeymouse::setShiftMode();
      state = 0;
      mickeymouse::setChainMode(0,0,0,0);
    }
  } 


  void headingControl(mickeymouse::PIDController* pidIMU){
    float headingOutput = pidIMU->compute(mickeymouse::getIMUState().yaw);
    headingOutput = constrain(headingOutput, -20, 20);
    
    Serial.print(F("Heading correction: "));Serial.println(headingOutput);

    if(mickeymouse::getChainMode().adjustYaw){
      mickeymouse::setWheelPWM(headingOutput * 4, -1 * headingOutput * 4);
      Serial.print("==================HEADINGOUTPUT: "); Serial.println(headingOutput * 4);

    }
    else{
      mickeymouse::setWheelPWM(getLeftPWM() + headingOutput, getRightPWM() - headingOutput);

    }
    if(headingOutput*4 < 6 && mickeymouse::getChainMode().adjustYaw){ /// CHANGE BOUNDS IF NEEDED

      mickeymouse::setChainMode(0,0,0,0);

      Serial.println("Shifting MODE");
      mickeymouse::setShiftMode();
      state = 0;
      mickeymouse::setWheelPWM(); // Set PWM to 0
    }
  } 

  void moveFront(mickeymouse::PIDController* pidL, mickeymouse::PIDController* pidR, mickeymouse::PIDController* pidYaw){
    if(state != 1){ // Change state in different modes of move
      state = 1;
    }

    Serial.print(" |L| "); Serial.print(mickeymouse::wheelState.leftWheelLinPos);
    Serial.print(" |R| "); Serial.println(mickeymouse::wheelState.rightWheelLinPos);

    float leftEffort = pidL->compute(mickeymouse::wheelState.leftWheelLinPos);
    leftEffort = constrain(leftEffort, -100, 100);

    if (leftEffort > -2 && leftEffort < 2) {
      leftEffort = 0;
    } else if (leftEffort >= 0) {
      leftEffort = constrain(leftEffort, 12, 100);
    } else{
      leftEffort = constrain(leftEffort, -100, -12);
    }

    float rightEffort = pidR->compute(mickeymouse::wheelState.rightWheelLinPos);
    rightEffort = constrain(rightEffort, -100, 100);

    if (rightEffort > -2 && rightEffort < 2) {
      rightEffort = 0;
    } else if (rightEffort >= 0) {
      rightEffort = constrain(rightEffort, 13, 100);
    } else{
      rightEffort = constrain(rightEffort, -100, -13);
    }

    Serial.print(" |LE| "); Serial.print(leftEffort);
    Serial.print(" |RE| "); Serial.println(rightEffort);

    mickeymouse::setWheelPWM(leftEffort, rightEffort);

    // Shift mode after goal reached
    if(leftEffort <= 1 || rightEffort <= 1){
      Serial.println("==================FRONT DONEEE==============================");
      mickeymouse::setChainMode(1,0,0,1);

      // Shifted shifting to headingControl

    }
  }


  void turnLeft(mickeymouse::PIDController* controller, mickeymouse::IMU* imu){
    if(state != 2){ // Change state in different modes of move
      state = 2;
    }

    Serial.println(mickeymouse::getIMUState().yaw);
    float yawOutput = controller->compute(mickeymouse::getIMUState().yaw);
    yawOutput = constrain(yawOutput, -90, 90);

    if (yawOutput > -9 && yawOutput < 9) {
      yawOutput = 0;
    } else if (yawOutput >= 0) {
      yawOutput = constrain(yawOutput, 12, 90);
    } else{
      yawOutput = constrain(yawOutput, -90, -12);
    }

    if(mickeymouse::getContinuousMode() && (controller->getError() < 15)){
        yawOutput = 1.5 * yawOutput;
      }

    Serial.println(yawOutput);

    mickeymouse::setWheelPWM(yawOutput, -1*yawOutput);

    // Shift mode after goal reached
    if(yawOutput == 0 ){
      Serial.println("Shifting MODE");
      mickeymouse::setShiftMode();
      state = 0;
      mickeymouse::setChainMode(0,0,0,0);
      mickeymouse::setWheelPWM(); // Set PWM to 0

      // Reset heading
      imu->zeroYaw();
    }

  }


  void turnRight(mickeymouse::PIDController* controller, mickeymouse::IMU* imu){
    if(state != 3){ // Change state in different modes of move
      state = 3;
    }

    Serial.println(mickeymouse::getIMUState().yaw);
    float yawOutput = controller->compute(mickeymouse::getIMUState().yaw);
    yawOutput = constrain(yawOutput, -90, 90);

    if (yawOutput > -9 && yawOutput < 9) {
      yawOutput = 0;
    } else if (yawOutput >= 0) {
      yawOutput = constrain(yawOutput, 12, 90);
    } else{
      yawOutput = constrain(yawOutput, -90, -12);
    }

    Serial.println(yawOutput);

    mickeymouse::setWheelPWM(yawOutput, -1 * yawOutput);

    // Shift mode after goal reached
    if(yawOutput == 0 ){
      Serial.println("Shifting MODE");
      mickeymouse::setShiftMode();
      state = 0;
      mickeymouse::setChainMode(0,0,0,0);
      mickeymouse::setWheelPWM();
      
      // Reset heading
      imu->zeroYaw();
    }

  }



  void moveFollow(mickeymouse::PIDController* controller){
      if(state != 5){ // Change state in different modes of move
        state = 5;
      }

      // Serial.println(mickeymouse::getLIDARState().distFront);
      float followEffort = controller->compute((mickeymouse::wheelState.leftWheelLinPos + mickeymouse::wheelState.rightWheelLinPos) / 2);
      followEffort = constrain(followEffort, -90, 90);
      Serial.println(followEffort);
      
      if(mickeymouse::getContinuousMode() && (controller->getError() < 30)){
        followEffort = 2 * followEffort;
      }

      mickeymouse::setWheelPWM( followEffort * 0.99, followEffort);
      Serial.print(mickeymouse::getLeftPWM()); Serial.print(" | "); Serial.println(getRightPWM());

      // Shift mode after goal reached
      if(followEffort < 5){
        Serial.println("Shifting MODE");
        mickeymouse::setShiftMode();
        state = 0;
        mickeymouse::setChainMode(0,0,0,0);

      }
    }

public:
    int state = 0;
};

} // namespace mickeymouse