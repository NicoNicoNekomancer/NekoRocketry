#ifndef MahonyFilter_h
#define MahonyFilter_h
#include <math.h>

//--------------------------------------------------------------------------------------------
class Mahony{
  public:
      Mahony(void);
      void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float &Q1, float &Q2, float &Q3, float &Q4, float dt);
  //-------------------------------------------------------------------------------------------
  private:
      float beta;				// algorithm gain
      float q0;
      float q1;
      float q2;
      float q3;	// quaternion of sensor frame relative to auxiliary frame
      

      float twoKp; // 2 * proportional gain (Kp)
      float twoKi;  // 2 * integral gain (Ki)
      float integralFBx;
      float integralFBy;
      float integralFBz;

      static float invSqrt(float x);
  };
  #endif