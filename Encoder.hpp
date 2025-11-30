#pragma once

#include "SharedMemory.hpp"

namespace mickeymouse {

// Forward declaration of encoder instances used in ISRs
class Encoder;
extern Encoder* encoder1;
extern Encoder* encoder2;
// Global ISRs
void encoder1_ISR();
void encoder2_ISR();

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // Assign instance to global pointer
        if (encoder1_pin == 3) {
            encoder1 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), encoder1_ISR, RISING);
        } else if (encoder1_pin == 2) {
            encoder2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), encoder2_ISR, RISING);
        }
    }

    // This function is called on interrupt to update count
    void readEncoder() {
        noInterrupts();

        // Read direction based on encoder2 pin
        direction = digitalRead(encoder2_pin);

        // Decide which is clockwise and anti
        if (direction == HIGH) {
            count++;   // e.g. clockwise
        } 
        else {
            count--;
        }

        interrupts();
    }


    // Converts encoder count to radians
    float getRotation() {
        float riyalCounts = 0;
        // Left Encoder works in negative counts. Casting it to positive.
        if(encoder1_pin == 2){
            riyalCounts = static_cast<float>(count) * -1;
            mickeymouse::setEncoderState(riyalCounts, 2);
        }
        else if(encoder1_pin == 3){
            riyalCounts = static_cast<float>(count);
            mickeymouse::setEncoderState(riyalCounts, 3);
        }
        
        float anglePos = ( riyalCounts / counts_per_revolution) *  2 * PI;

        return anglePos;
    }

    // Converts angles to distance covered
    float getDistance(){
        float angle = this->getRotation();
        float dist = WHEEL_RADIUS * angle;

        if(encoder1_pin == 2){
            mickeymouse::setWheelState(angle, dist, 2);
        }
        else if(encoder1_pin == 3){
            mickeymouse::setWheelState(angle, dist, 3);
        }
        return dist;
    }


public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    volatile long count = 0;
    uint16_t counts_per_revolution = 700;  // Seen on Ed forum
    bool read = false;

};

// Global pointers to link ISRs with Encoder objects
Encoder* encoder1 = nullptr;
Encoder* encoder2 = nullptr;

// ISR wrappers for each encoder
void encoder1_ISR() {
    if (encoder1) encoder1->readEncoder();
}

void encoder2_ISR() {
    if (encoder2) encoder2->readEncoder();
}

}  // namespace mickeymouse