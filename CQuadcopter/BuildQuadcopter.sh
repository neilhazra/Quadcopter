g++ -c IMU.cpp -lphidget22
g++ -c Quadcopter.cpp -pthread -lpigpio -lrt
g++ -o run Quadcopter.o IMU.o -lphidget22 -pthread -latomic -lpigpio -lrt
