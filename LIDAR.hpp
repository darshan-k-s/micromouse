#pragma once


#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

#include "SharedMemory.hpp"

namespace mickeymouse {

class LIDAR {
public:
  LIDAR(uint8_t en, uint8_t address){
    enablePin = en;
    lidarAddr = address;
  }

  // destructor to stop continuous
  ~LIDAR(){
    lidar.stopContinuous();  
  }

  /**
   * Power on and configure the sensor.
   * Returns true on completion (you can still check timeoutOccurred()
   * or zero/reading==0 in loop() to detect comms errors).
   */
  bool beginLidar() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    delay(10);                       // allow VL6180X to boot

    lidar.init();                    // load required settings (void) :contentReference[oaicite:1]{index=1}
    lidar.configureDefault();        // set up default range & ALS params (void) :contentReference[oaicite:2]{index=2}
    lidar.setTimeout(250);           // abort reads after 250 ms if no response
    lidar.setAddress(lidarAddr);     // change from default (0x29) to user address
    lidar.startRangeContinuous(LIDAR_SAMPLING_PERIOD);

    return true;
  }

  void getDistance() {
    uint16_t d = lidar.readRangeContinuousMillimeters();
    // if (lidar.timeoutOccurred()) {
    //   // recover: re-init? For now return out-of-range
    //   return 0xFFFF;
    // }
    // Write to shared mem using lidar addr
    mickeymouse::setLIDARState(d, lidarAddr);
  }


private:
  uint8_t enablePin;
  uint8_t lidarAddr;
  VL6180X lidar;
};

} // namespace mickeymouse
