#include "IMU.h"
#include <chrono>
#include <mutex>          // std::mutex
using namespace std::chrono;
using namespace std;
using namespace Eigen;
mutex mtx3;
double beta  = 0.15;
Quaterniond q;
const double DEGTORAD = 3.141592/180.0;

static void CCONV onSpatialData(PhidgetSpatialHandle ch, void * ctx, const double acceleration[3], const double angularRate[3], const double magneticField[3], double timestamp) {
    static double prevTime = timestamp;
    Vector3d accel(acceleration[0], acceleration[1], acceleration[2]);
    accel.normalize();
    Vector3d mag(magneticField[0], magneticField[1], magneticField[2]);
    mag.normalize();
    Quaterniond ang(0, angularRate[0]*DEGTORAD,angularRate[1]*DEGTORAD,angularRate[2]*DEGTORAD);
    Vector3d h = q._transformVector(mag);
    Quaterniond b(0,sqrt(pow(h(0),2) + pow(h(1),2)),0,h(2));
    Matrix<double,6,1> F;
    F<<  2 * (q.x() * q.z() - q.w() * q.y()) - accel(0),
         2 * (q.w() * q.x() + q.y() * q.z()) - accel(1),
         2 * (0.5 - pow(q.x(), 2) - pow(q.y(),2)) - accel(2),
         2 * b.x() * (0.5 - pow(q.y(),2) - pow(q.z(),2)) + 2 * b.z() * (q.x() * q.z() - q.w() * q.y()) - mag(0),
         2 * b.x() * (q.x() * q.y() - q.w() * q.z()) + 2 * b.z() * (q.w() * q.x() + q.y() * q.z()) - mag(1),
         2 * b.x() * (q.w() * q.y() + q.x() * q.z()) + 2 * b.z() * (0.5 - pow(q.x(), 2) - pow(q.y(),2)) - mag(2);
    Matrix<double, 4, 6> J;
    J<< -2 * q.y(),  2 * q.x(),  0,         -2 * b.z() * q.y(),                     -2 * b.x() * q.z() + 2 * b.z() * q.x(),  2 * b.x() * q.y(),
         2 * q.z(),  2 * q.w(), -4 * q.x(),  2 * b.z() * q.z(),                      2 * b.x() * q.y() + 2 * b.z() * q.w(),  2 * b.x() * q.z() - 4 * b.z() * q.x(),
        -2 * q.w(),  2 * q.z(), -4 * q.y(), -4 * b.x() * q.y() - 2 * b.z() * q.w(),  2 * b.x() * q.x() + 2 * b.z() * q.z(),  2 * b.x() * q.w() - 4 * b.z() * q.y(),
         2 * q.x(),  2 * q.y(),  0,         -4 * b.x() * q.z() + 2 * b.z() * q.x(), -2 * b.x() * q.w() + 2 * b.z() * q.y(),  2 * b.x() * q.x();
    Matrix<double, 4, 1> step = J * F;
    step.normalize();
    Quaterniond rotated = q * ang;
    Vector4d temp; temp << rotated.w(), rotated.x(), rotated.y(), rotated.z(); temp *= 0.5;
    Vector4d qDot = temp - beta * step;
    double dt = (timestamp - prevTime)/1000;
    qDot *= dt;
    Quaterniond q_temp(q.w()+qDot(0),q.x()+qDot(1),q.y()+qDot(2),q.z()+qDot(3)); q_temp.normalize();
    mtx3.lock();
    q = q_temp;
    mtx3.unlock();
    //printf("Quaternion: \t%lf  |  %lf  |  %lf| %lf| %lf\n", q.w(), q.x(), q.y(), q.z(), timestamp);
	prevTime = timestamp;
}

IMU::IMU()  {
    PhidgetSpatial_create(&spatial0);
    PhidgetSpatial_setOnSpatialDataHandler(spatial0, onSpatialData, NULL);
    Phidget_openWaitForAttachment((PhidgetHandle)spatial0, 2000);
    PhidgetSpatial_zeroGyro(spatial0);
    PhidgetSpatial_setDataInterval(spatial0, 4); // make this 4
    initial_orientation = Quaterniond(1,0,0,0);
    q = Quaterniond(1,0,0,0);
}

void IMU::zeroOrientation(void)   {
    initial_orientation = q.conjugate();
};

double* IMU::getRollPitchYaw(void)  {
    static double RPY[3];
    mtx3.lock();
    Quaterniond cR =  initial_orientation * q;
    mtx3.unlock();
    RPY[1] = atan2(2*(cR.w()*cR.x()+cR.y()*cR.z()) ,1 - 2 * (pow(cR.x(),2) + pow(cR.y(),2)));
    RPY[0] =  asin(2*(cR.w()*cR.y()-cR.z()*cR.x()));
    RPY[2] = atan2(2*(cR.w()*cR.z()+cR.x()*cR.y()) ,1 - 2 * (pow(cR.y(),2) + pow(cR.z(),2)));
    return RPY;
}

void IMU::setBeta(double newBeta)    {
    beta = newBeta;
}

void IMU:: join(void)   {
    Phidget_close((PhidgetHandle)spatial0);
    PhidgetSpatial_delete(&spatial0);
}