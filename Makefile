debug: main.cpp KalmanFilter.h
	g++ -std=c++11 KalmanFilter.h main.cpp -o tester -lglut -lGL -lGLU -D DEBUG=1

silent: main.cpp KalmanFilter.h
	g++ -std=c++11 KalmanFilter.h main.cpp -o tester -lglut -lGL -lGLU -D DEBUG=0

clean:
	rm -rf ./tester

.PHONY: clean
