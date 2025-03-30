//Code modified from madgwicks paper + some arduino stuff
//-------------------------------------------------------------------------------------------
#include "MadgwickFilter.h"
#include <math.h>
#include <Arduino.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define BETA_INITIAL    3.0f    // High initial beta for fast convergence
#define BETA_FINAL      0.04f    // Lower beta for stable operation
#define CONVERGENCE_SAMPLES 100  // Number of samples before reducing beta
#define BETA_DECAY_RATE 0.95f   // Optional: for gradual decay instead of step change

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Madgwick::Madgwick() {
  beta = BETA_INITIAL;
  //initial starting values
  q0 = 0.0f;
  q1 = 0.0f;
  q2 = -0.70f;
  q3 = -0.70f;
  
  // Add counter for tracking samples processed
  sampleCount = 0;
}


void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float &Q1, float &Q2, float &Q3, float &Q4, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3, _8bx, _8bz;

  // Update sample count and adjust beta if needed
  sampleCount++;
  if(sampleCount <= CONVERGENCE_SAMPLES) {
    // Option 1: Step change after X samples
    if(sampleCount == CONVERGENCE_SAMPLES) {
      beta = BETA_FINAL;
    }
    
    // Option 2: Gradual decay (uncomment to use instead of step change)
    // beta = BETA_INITIAL * pow(BETA_DECAY_RATE, sampleCount) + BETA_FINAL * (1 - pow(BETA_DECAY_RATE, sampleCount));
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    float mzq3 = mz * q3;
    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mzq3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mzq3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _2bx *= 2.0f;
    _2bz *= 2.0f;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;


    //helper variables to speed up math
    float v1 = 2.0f * q1q3 - _2q0q2 - ax;
    float v2 = 2.0f * q0q1 + _2q2q3 - ay;
    float v3 = 0.5f - q2q2 - q3q3;
    float v4 = q1q3 - q0q2;
    float v5 = _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my;
    float v6 = _2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz;
    float v7 = q3 + _2bz * q1;
    float v8 = _2bz * q2;
    float v9 = 1 - 2.0f * q1q1 - 2.0f * q2q2 - az;
    float v10 = _2bz * q0;
    float v11 = _2bz * q3;
    float v12 = _2bx * q0;
    float v13 = _2bx * (v3) + _2bz * (v4) - mx;
    float v14 = _2bx * q2;
    float v15 = _2bx * q1;


    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (v1) + _2q1 * (v2) - v8 * (v12) + (-_2bx * v7) * (v5) + v14 * (v6);
    s1 = _2q3 * (v1) + _2q0 * (v2) - 4.0f * q1 * (v9) + v11 * (v12) + (v14 + v10) * (v5) + (_2bx * q3 - _4bz * q1) * (v6);
    s2 = -_2q0 * (v1) + _2q3 * (v2) - 4.0f * q2 * (v9) + (-_4bx * q2 - v10) * (v12) + (v15 + v11) * (v5) + (v12 - _4bz * q2) * (v6);
    s3 = _2q1 * (v1) + _2q2 * (v2) + (-_4bx * v7) * (v12) + (-v12 + v8) * (v5) + v15 * (v6);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Set output quaternion
  Q1 = q0;
  Q2 = q1;
  Q3 = q2;
  Q4 = q3;
}

float Madgwick::invSqrt(float x) {
  float halfx = 0.5f * x;
  int i = *(int*)&x;
  i = 0x5f3759df - (i >> 1);
  x = *(float*)&i;
  x = x * (1.5f - halfx * x * x);
  return x;
}

//-------------------------------------------------------------------------------------------
/*
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
*/
