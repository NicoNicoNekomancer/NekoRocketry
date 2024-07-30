//Code modified from madgwicks paper + some arduino stuff
#ifndef MadgwickFilter_h
#define MadgwickFilter_h
#include <math.h>

//--------------------------------------------------------------------------------------------
class Madgwick{
public:
    Madgwick(void);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float &Q1, float &Q2, float &Q3, float &Q4);
//-------------------------------------------------------------------------------------------
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    unsigned long previousMicros = 0;
    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;
    int   biasIndex = 0;
    float sumBiasX = 0.0f;
    float sumBiasY = 0.0f;
    float sumBiasZ = 0.0f;
};
#endif

