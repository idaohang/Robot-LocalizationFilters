LD_LIBRARIES=-lglut -lGL -lGLU 
INCLUDE_DIR=-I/usr/include/eigen3

UKF: ukf_main.cpp UnscentedKalmanFilter.h
	g++ -std=c++11 ukf_main.cpp -o ukf_demo $(LD_LIBRARIES) $(INCLUDE_DIR) -D DEBUG=1 

KF: kf_main.cpp KalmanFilter.h
	g++ -std=c++11 kf_main.cpp -o kf_demo $(LD_LIBRARIES) $(INCLUDE_DIR)

clean:
	rm -rf ./ukf_demo
	rm -rf ./kf_demo

.PHONY: clean
