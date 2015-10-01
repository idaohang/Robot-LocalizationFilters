EIGEN_INCLUDE_DIR=/usr/include/eigen3/

all: main.cpp KalmanFilter.cpp
	g++ main.cpp KalmanFilter.cpp -o tester
