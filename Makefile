LD_LIBRARIES=-lglut -lGL -lGLU 
INCLUDE_DIR=-I/usr/include/eigen3

UKF: ukf_main.cc UnscentedKalmanFilter.h
	g++ -std=c++11 ukf_main.cc -o ukf_demo $(LD_LIBRARIES) $(INCLUDE_DIR)

KF: kf_main.cc KalmanFilter.h
	g++ -std=c++11 kf_main.cc -o kf_demo $(LD_LIBRARIES) $(INCLUDE_DIR)

clean:
	rm -rf ./ukf_demo
	rm -rf ./kf_demo

.PHONY: clean
