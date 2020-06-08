#ifndef INPUTDATA_H
#define INPUTDATA_H
#include <chrono>
using namespace std;
#include <cstring>
#include <iostream>
#include <atomic>
using namespace std::chrono;

class InputData   {
public:
    InputData();
    void update(uint8_t[]);
    double throttle;
    double rotate;
    double roll;
    double pitch;
    double kill;
    atomic_ullong lastUpdated;

};

InputData::InputData()  {
    lastUpdated = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    throttle = 1.0;
    rotate = 0.0;
    roll = 0.0;
    pitch = 0.0;
    kill = 0.0;
}

void InputData::update(uint8_t buffer[]) {
    float data[5];
    memcpy(data, buffer, sizeof(data));
    throttle = (data[0]+1.0)/2.0;
    rotate = data[1];
    roll = data[2];
    pitch = data[3];
    kill = data[4];
    lastUpdated = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

#endif