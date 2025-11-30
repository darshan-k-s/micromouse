#pragma once

#include <util/atomic.h>

/*
Robot Dynamics Constants
*/
// Wheel radius
#define WHEEL_RADIUS 16 // In mm


/*
Motor constants
*/
// Anticlockwise is -ve angles
// Motor 1, i.e Left
#define MOT1PWM 11
#define MOT1DIR 12
#define EN1_A 2
#define EN1_B 7

// Motor 2, i.e Right
#define MOT2PWM 9 
#define MOT2DIR 10
#define EN2_A 3
#define EN2_B 8


/*
LiDAR constants
*/
#define LIDAR_LEFT_EN A0
#define LIDAR_FRONT_EN A1
#define LIDAR_RIGHT_EN A2

#define LIDAR_LEFT_ADDR 0x30
#define LIDAR_FRONT_ADDR 0x35
#define LIDAR_RIGHT_ADDR 0x38

#define LIDAR_SAMPLING_PERIOD 100 // 100ms sampling rate in continuous mode

/*
IMU constants
*/
#define IMU_INTERVAL 10  // 10ms = 100Hz

namespace mickeymouse {
  // To tell what the robot is doing in command chain
  struct chainMode{
    bool front = 0;
    bool left = 0;
    bool right = 0;

    bool adjustYaw = 0;

    bool continuousMode = 0;
    bool shiftMode = 1;
  };
  inline chainMode ChainMode;
  inline void setChainMode(bool front=0, bool left=0, bool right=0, bool adjustYaw = 0){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ChainMode.front = front;
      ChainMode.left = left;
      ChainMode.right = right;
      ChainMode.adjustYaw = adjustYaw;
    }
  }
  inline chainMode getChainMode(){
    chainMode copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      copy = ChainMode;
    }
    return copy;
  }
  inline void setShiftMode(){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ChainMode.shiftMode = !ChainMode.shiftMode;
    }
  }
  inline bool getShiftMode(){
    chainMode copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      copy = ChainMode;
    }
    return copy.shiftMode;
  }
  
  inline void setContinuousMode(){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      ChainMode.continuousMode = !ChainMode.continuousMode;
    }
  }
  inline bool getContinuousMode(){
    chainMode copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      copy = ChainMode;
    }
    return copy.continuousMode;
  }


  /*
  // Shared IMU readings
  */
  struct IMUState{
    float yaw = 0;
    float yawError = NULL;
  };
  inline IMUState imuState;
  inline void setIMUState(float latestYaw, float latestError = 0) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      imuState.yaw = -1 * latestYaw;
      imuState.yawError = latestError;
    }
  }
  inline IMUState getIMUState(){
    IMUState copy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copy = imuState;
  }
  return copy;
  }

/*
  // Shared LiDAR readings
*/
  struct LIDARState{
    // All in mm
    float distLeft = NULL;
    float distFront = NULL;
    float distRight = NULL;
  };
  inline LIDARState lidarState;
  inline void setLIDARState(float dist, uint8_t addr){
    if(addr == LIDAR_LEFT_ADDR){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lidarState.distLeft = dist;
      }
    }
    else if(addr == LIDAR_FRONT_ADDR){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lidarState.distFront = dist;
      }
    }
    else if(addr == LIDAR_RIGHT_ADDR){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lidarState.distRight = dist;
      }
    }
  }
  inline LIDARState getLIDARState(){
    LIDARState copy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copy = lidarState;
  }
  return copy;
  }

/*
  // Shared Encoder readings
*/
  struct EncoderState{
    float leftEncoderCount = 0.0;
    float rightEncoderCount = 0.0;
  };
  inline EncoderState encoderState;
  inline void setEncoderState(float counts, uint8_t pinNum){
    if(pinNum == 2){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      encoderState.leftEncoderCount = counts;
      }  
    }
    else if(pinNum == 3){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      encoderState.rightEncoderCount = counts;
     }
    }
  }
  inline EncoderState getEncoderState(){
    EncoderState copy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copy = encoderState;
  }
  return copy;
  }

/*
  // Wheel State
*/
  struct WheelState{
    float leftWheelAnglePos;
    float rightWheelAnglePos;
    float leftWheelLinPos;
    float rightWheelLinPos;
  };
  inline WheelState wheelState;
  inline void setWheelState(float anglePos, float distPos, uint8_t pinNum){
    // 2 is left
    if(pinNum == 2){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      wheelState.leftWheelAnglePos = anglePos;
      wheelState.leftWheelLinPos = distPos;
    }
    }
    else if(pinNum == 3){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      wheelState.rightWheelAnglePos = anglePos;
      wheelState.rightWheelLinPos = distPos;
    }
    }
  }
  inline WheelState getWheelState(){
    WheelState copy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copy = wheelState;
  }
  return copy;
  }


/*
// Wheel PWM states
*/
  struct WheelPWM{
    int leftPWM = 0;
    int rightPWM = 0;
  };
  inline WheelPWM wheelPWM;
  inline int getLeftPWM(){
      WheelPWM copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      copy = wheelPWM;
    }
    return copy.leftPWM;
    }
  inline int getRightPWM(){
      WheelPWM copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      copy = wheelPWM;
    }
    return copy.rightPWM;
    }
  inline void setWheelPWM(int left = 0, int right = 0){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      wheelPWM.leftPWM = left;
      wheelPWM.rightPWM = right;
    }
    }

  /*
  // Movement Controllers State
  // Storing state for setpoints in angle and translation
  */
  struct SetpointState{
    float angleSetpoint = NULL;
    float linearSetpoint = NULL;
  };
  inline SetpointState setpointState;
  inline void setSetpointState(float angle, float displacement){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      setpointState.angleSetpoint = angle;
      setpointState.linearSetpoint = displacement;
    }
  }  
  inline SetpointState getSetpointState(){
    SetpointState copy;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    copy = setpointState;
  }
  return copy;
  }


} // namespace mickeymouse
