#ifndef OUTPUTDATA_H
#define OUTPUTDATA_H
#include <chrono>
using namespace std;
#include <cstring>
#include <iostream>
using namespace std::chrono;

class OutputData   {
public:
    OutputData();
    void toByteArray(uint8_t[]);
    void update(double, double, double, double);
    float roll;
    float pitch;
    float yaw;
    float isStopping;

};

OutputData::OutputData()  {
    roll = 0;
    pitch = 0;
    yaw = 0;
    isStopping = 0;
}
void OutputData::update(double m_roll, double m_pitch, double m_yaw, double m_isStopping)   {
    roll = (float) m_roll;
    pitch = (float) m_pitch;
    yaw = (float) m_yaw;
    isStopping = (float) m_isStopping;
}

void OutputData::toByteArray(uint8_t buffer[]) {
    float data[4] = {roll, pitch, yaw, isStopping};
    memcpy(buffer, data, 16);
}

#endif