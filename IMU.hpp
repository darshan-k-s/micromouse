#pragma once

#include <MPU6050_light.h>
#include <Wire.h>

#include "SharedMemory.hpp"

//––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// 1-D Kalman filter for angle + bias estimation
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––
class Kalman {
public:
    Kalman() {
        Q_angle   = 0.001f;   // process noise variance for the accelerometer
        Q_bias    = 0.004f;   // process noise variance for the gyro bias
        R_measure = 0.03f;    // measurement noise variance
        angle     = 0.0f;
        bias      = 0.0f;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
    }

    // newAngle  = accel-based angle (deg)
    // newRate   = gyro rate      (deg/s)
    // dt        = timestep       (s)
    float getAngle(float newAngle, float newRate, float dt) {
        // 1) Predict
        float rate = newRate - bias;
        angle += dt * rate;

        // 2) Update error covariance
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 3) Compute Kalman gain
        float S   = P[0][0] + R_measure;
        float K0  = P[0][0] / S;
        float K1  = P[1][0] / S;

        // 4) Update with measurement
        float y   = newAngle - angle;
        angle    += K0 * y;
        bias     += K1 * y;

        // 5) Update covariance
        float P00 = P[0][0], P01 = P[0][1];
        P[0][0] -= K0 * P00;
        P[0][1] -= K0 * P01;
        P[1][0] -= K1 * P00;
        P[1][1] -= K1 * P01;

        return angle;
    }

private:
    // filter tuning parameters
    float Q_angle, Q_bias, R_measure;
    // filter state
    float angle, bias;
    // error covariance matrix
    float P[2][2];
};


//––––––––––––––––––––––––––––––––––––––––––––––––––––––––
// IMU class using MPU6050_light + Kalman on X/Y + yaw bias correction
//––––––––––––––––––––––––––––––––––––––––––––––––––––––––
namespace mickeymouse {

class IMU {
public:
    IMU() {
        lastMicros = micros();
        yawRaw     = yawOffset = yaw = 0.0f;
        gyroBiasZ  = 0.0f;
    }

    // call once at startup
    void calibrate() {
        // init MPU
        byte status = mpu.begin();
        if (status != 0) {
            Serial.print(F("MPU6050 init failed, code "));
            Serial.println(status);
            while (1);
        }
        Serial.println(F("Calibrating IMU… keep still"));
        mpu.calcOffsets(true, true);
        delay(500);

        // estimate Z-gyro bias
        const uint8_t samples = 250;
        float sumZ = 0.0f;
        for (uint8_t i = 0; i < samples; i++) {
            mpu.update();
            sumZ += mpu.getGyroZ();
            delay(5);
        }
        gyroBiasZ = sumZ / samples;
        Serial.print(F("Gyro Z bias = ")); Serial.println(gyroBiasZ, 3);

        // reset timers & angles
        lastMicros = micros();
        yawRaw = yawOffset = yaw = 0.0f;
        kalmanX = Kalman();
        kalmanY = Kalman();
    }

    // call in your loop or a regular timer ISR
    void update() {
        mpu.update();

        // compute dt (s)
        uint32_t now = micros();
        float dt = (now - lastMicros) * 1e-6f;
        lastMicros = now;

        // raw gyro rate (deg/s), correct Z by bias
        float gZ = mpu.getGyroZ() - gyroBiasZ;

        // integrate yaw (deg)
        yawRaw += gZ * dt;
        yaw = yawRaw - yawOffset;

        mickeymouse::setIMUState(yaw);
    }

    // call immediately after achieving each turn to reset heading to zero
    void zeroYaw() {
        yawOffset = yawRaw;
    }

    // getters
    float getYaw()   const { return yaw;   }

private:
    MPU6050 mpu{Wire};
    Kalman  kalmanX, kalmanY;

    uint32_t lastMicros;
    float    gyroBiasZ;
    float    yawRaw, yawOffset, yaw;
};

}  // namespace mickeymouse
