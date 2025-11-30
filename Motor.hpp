#pragma once

#include "math.h"

namespace mickeymouse {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        // TODO: Set both pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(in2, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {
      // TODO: Output digital direction pin based on if input signal is positive or negative.
      // Not working when DIR pin is LOW
      if(pwm >= 0){
        digitalWrite(dir_pin, HIGH);
      }
      else{
        digitalWrite(dir_pin, LOW);
      }

      if(pwm == 0){
        analogWrite(pwm_pin, 0);
      }
      else{
        int finalPWM = constrain(abs(pwm), 8, 255);
        analogWrite(pwm_pin, finalPWM);
        // Serial.print(pwm); Serial.print(" | "); Serial.println(finalPWM);
      }

    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mickeymouse
