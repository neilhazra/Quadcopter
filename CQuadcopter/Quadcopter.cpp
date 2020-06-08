#include <stdio.h>
#include "IMU.h"
#include <thread>
#include <chrono>
#include "PID.h"
#include "InputData.h"
#include "OutputData.h"
#include <thread>
#include <iostream>
#include <cstdio>
#include <atomic>
#include <mutex>          // std::mutex
using namespace std;
const double RADTODEG = 180.0/3.141592;
mutex mtx;
mutex mtx1;
//Data Wrappers
//###########################
InputData input;
OutputData output;
//##########################
//Socket Programming
//############################
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
int sockfd; // socket file descriptor
int portno = 3000; // port number
struct sockaddr_in serv_addr;
struct hostent *server;
//###########################
//############GPIO/ESC#############
#include <pigpio.h>
#define ESC1    17
#define ESC2    27
#define ESC3    22
#define ESC4    4
double constrain(double value, double min, double max) {
    if(value < min) return min;
    else if (value > max) return max;
    else return value;
}
#define MAX_VOLTAGE 12.5
//Data Transfer Function
//###########################
void transferThread()   {
    while(true) {
        u_char recvBuffer[20];
        int rbytes = recv(sockfd, recvBuffer, sizeof(recvBuffer), 0);
        if(rbytes != 20) {
            cout << "Didn't recieve Enough Bytes" << "\n";
            gpioPWM(ESC1, 700);
            gpioPWM(ESC2, 700);
            gpioPWM(ESC3, 700);
            gpioPWM(ESC4, 700);
            exit(0);
        }
        mtx.lock();
        input.update(recvBuffer);
        mtx.unlock();
        u_char send_buffer[16];
        mtx1.lock();
        output.toByteArray(send_buffer);
        mtx1.unlock();
        write(sockfd, send_buffer, 16);
    }
}
//###########PID##################
double P_roll_pitch;
double I_roll_pitch;
double D_roll_pitch;
double N_roll_pitch;
double P_yaw;
double I_yaw;
double D_yaw;
double N_yaw;
//################################

int main(int argc, char *argv[]) {
    ///#####################################################
    if(argc != 9) {
        cout << "entered incorrect number of arguments\n";
        exit(0);
    }
    sscanf(argv[1],"%lf",&P_roll_pitch);
    sscanf(argv[2],"%lf",&I_roll_pitch);
    sscanf(argv[3],"%lf",&D_roll_pitch);
    sscanf(argv[4],"%lf",&N_roll_pitch);
    sscanf(argv[5],"%lf",&P_yaw);
    sscanf(argv[6],"%lf",&I_yaw);
    sscanf(argv[7],"%lf",&D_yaw);
    sscanf(argv[8],"%lf",&N_yaw);

    cout << "Loaded Following PID ROLL PITCH Values: " << P_roll_pitch << " " << I_roll_pitch << " " <<  D_roll_pitch << " " << N_roll_pitch << "\n";
    cout << "Loaded Following PID YAW Values: " << P_yaw << " " << I_yaw << " " <<  D_yaw << " " << N_yaw << "\n";

    cout << "press enter to continue\n";
    getchar();
    //#######################GPIOD###########################
    if (gpioInitialise() < 0) cout << "Failed GPIO Initialization\n";
    gpioSetPWMfrequency(ESC1, 400);
    gpioSetPWMrange(ESC1, 2500);
    gpioSetPWMfrequency(ESC2, 400);
    gpioSetPWMrange(ESC2, 2500);
    gpioSetPWMfrequency(ESC3, 400);
    gpioSetPWMrange(ESC3, 2500);
    gpioSetPWMfrequency(ESC4, 400);
    gpioSetPWMrange(ESC4, 2500);
    //################SOCKETS################################
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // generate file descriptor
    if (sockfd < 0)
        perror("ERROR opening socket");
    server = gethostbyname("10.0.0.22");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
        perror("ERROR connecting");
    thread transfer (transferThread);
    //################IMU################################
    IMU imu;
    //double yawOffset = 0.0;
    imu.setBeta(0.9);
    //################PID###############################
    float throttle, rotate, roll, pitch, kill, roll_measured, pitch_measured, yaw_measured;
    throttle = 0.01f;
    cout << "Throttle Down To Start Calibrating, Place Drone in Level Surface" << "\n";
    while(throttle != 0) {
        mtx.lock();
        throttle = input.throttle;
        kill = input.kill;
        mtx.unlock();
        double* rpy = imu.getRollPitchYaw();
        roll_measured = rpy[0];
        pitch_measured = rpy[1];
        yaw_measured = rpy[2];
        mtx1.lock();
        output.update(roll_measured, pitch_measured, yaw_measured, kill);
        mtx1.unlock();
        cout << "Throttle" << throttle << "\n";
        this_thread::sleep_for(chrono::milliseconds(30));
        gpioPWM(ESC1, 700);
        gpioPWM(ESC2, 700);
        gpioPWM(ESC3, 700);
        gpioPWM(ESC4, 700);
        if(kill == 1.0f) {
            this_thread::sleep_for(chrono::milliseconds(30));
            exit(0);
        }
    }
    imu.setBeta(0.12);
    //yawOffset = imu.getRollPitchYaw()[2];
    imu.zeroOrientation();
    cout << "Right Stick Down To Start Flying" << "\n";
    while(pitch != -1) {
        mtx.lock();
        throttle = input.throttle;
        pitch = input.pitch;
        roll = input.roll;
        kill = input.kill;
        mtx.unlock();
        double* rpy = imu.getRollPitchYaw();
        roll_measured = rpy[0];
        pitch_measured = rpy[1];
        yaw_measured = rpy[2];
        mtx1.lock();
        output.update(roll_measured, pitch_measured, yaw_measured, kill);
        mtx1.unlock();
        cout << "Throttle" << throttle << " Pitch " << pitch << "\n";
        unsigned long long currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        if(currentTime - input.lastUpdated > 750)   {
            cout << "Slow Communications" << "\n";
            while(true) {
                gpioPWM(ESC1, 700);
                gpioPWM(ESC2, 700);
                gpioPWM(ESC3, 700);
                gpioPWM(ESC4, 700);
            }
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(30));
        gpioPWM(ESC1, (int) (700+1300*throttle));
        gpioPWM(ESC2, (int) (700+1300*throttle));
        gpioPWM(ESC3, (int) (700+1300*throttle));
        gpioPWM(ESC4, (int) (700+1300*throttle));
        if(kill == 1.0f) {
            gpioPWM(ESC1, 700);
            gpioPWM(ESC2, 700);
            gpioPWM(ESC3, 700);
            gpioPWM(ESC4, 700);
            this_thread::sleep_for(chrono::milliseconds(300));
            exit(0);
        }
    }

    imu.zeroOrientation();
    //yawOffset = imu.getRollPitchYaw()[2];
    PID pidroll(P_roll_pitch,I_roll_pitch,D_roll_pitch,N_roll_pitch);
    PID pidpitch(P_roll_pitch,I_roll_pitch,D_roll_pitch,N_roll_pitch);
    PID pidyaw(P_yaw,I_yaw,D_yaw,N_yaw);

    unsigned long long startTime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    while(true) {
        mtx.lock();
        throttle = input.throttle;
        rotate = input.rotate;
        roll = input.roll * 0.25;
        pitch = input.pitch * 0.25;
        kill = input.kill;
        mtx.unlock();
        double* rpy = imu.getRollPitchYaw();
        roll_measured = rpy[0];
        pitch_measured = rpy[1];
        yaw_measured = rpy[2];
        mtx1.lock();
        output.update(roll_measured, pitch_measured, yaw_measured, kill);
        mtx1.unlock();
        unsigned long long currentTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        if(currentTime - input.lastUpdated > 750)   {
            cout << "Slow Communications" << "\n";
            break;
        }
        //#####################Update Motors###############
        double rollOutput = pidroll.getControllerOutput(roll_measured + roll) / MAX_VOLTAGE;
        double pitchOutput = pidpitch.getControllerOutput(pitch_measured + pitch) / MAX_VOLTAGE;
        double yawOutput = pidyaw.getControllerOutput(yaw_measured) / MAX_VOLTAGE;
        double CCW1 = constrain(throttle - yawOutput - pitchOutput, 0, 1);
        double CCW2 = constrain(throttle - yawOutput + pitchOutput, 0, 1);
        double CW3 = constrain(throttle + yawOutput + rollOutput, 0, 1);
        double CW4 = constrain(throttle + yawOutput - rollOutput, 0, 1);
        gpioPWM(ESC1, (int) (700+1300*CCW1));
        gpioPWM(ESC2, (int) (700+1300*CCW2));
        gpioPWM(ESC3, (int) (700+1300*CW3));
        gpioPWM(ESC4, (int) (700+1300*CW4));

        cout << (duration_cast<microseconds>(system_clock::now().time_since_epoch()).count() -startTime)/1000000.0 << " ";
        cout << roll_measured << " " << pitch_measured << " " << yaw_measured << " ";
        cout << roll << " " << pitch << " ";
        cout << CCW1 << " " << CCW2 << " " << CW3 << " " << CW4 << "\n";

        this_thread::sleep_for(chrono::microseconds(3800));
        //#################################################
        if(kill == 1.0f) {
            gpioPWM(ESC1, 700);
            gpioPWM(ESC2, 700);
            gpioPWM(ESC3, 700);
            gpioPWM(ESC4, 700);
            this_thread::sleep_for(chrono::milliseconds(300));
            exit(0);
        }
    }
    gpioPWM(ESC1, 700);
    gpioPWM(ESC2, 700);
    gpioPWM(ESC3, 700);
    gpioPWM(ESC4, 700);
    transfer.join();
}