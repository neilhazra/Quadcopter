#ifndef PID_H
#define PID_H
#include <chrono>
using namespace std::chrono;

class PID   {
public:
    PID(double, double, double, double);
    double kp, ki, kd, N;
    unsigned long long prevTimeMillis;
    double filter_coefficient, filter_dState;
    double integral;
    double getControllerOutput(double error);
};

PID::PID(double _kp, double _ki, double _kd, double _N)  {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    N = _N;
    integral = 0;
    filter_coefficient = 0;
    filter_dState = 0;
    prevTimeMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
double PID::getControllerOutput(double error)   {
    unsigned long long currentTimeMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    double dt = (currentTimeMillis - prevTimeMillis)/1000.0; //dt in seconds now
    filter_coefficient = (kd * error - filter_dState) * N;
    double output = kp * error + integral + filter_coefficient;
    integral += ki * error * dt;
    filter_dState += dt * filter_coefficient;
    prevTimeMillis = currentTimeMillis;
    return output;
}

#endif