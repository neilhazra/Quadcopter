#ifndef IMU_H
#define IMU_H
#include <phidget22.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
using namespace Eigen;

class IMU
{
public:
    IMU();
    PhidgetSpatialHandle spatial0;
    Quaterniond initial_orientation;
    void zeroOrientation();
    void setBeta(double newBeta);
    double* getRollPitchYaw();
    void join();
};
#endif