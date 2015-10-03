UKF: ukf_main.cpp UnscentedKalmanFilter.h
	g++ -std=c++11 UnscentedKalmanFilter.h ukf_main.cpp -o ukf_demo -lglut -lGL -lGLU -D DEBUG=1

debug: kf_main.cpp KalmanFilter.h
	g++ -std=c++11 KalmanFilter.h kf_main.cpp -o kf_demo -lglut -lGL -lGLU -D DEBUG=1

silent: kf_main.cpp KalmanFilter.h
	g++ -std=c++11 KalmanFilter.h kf_main.cpp -o kf_demo -lglut -lGL -lGLU -D DEBUG=0

clean:
	rm -rf ./ukf_demo
	rm -rf ./kf_demo

.PHONY: clean
