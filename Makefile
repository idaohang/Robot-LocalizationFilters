all: main.cpp KalmanFilter.h
	g++ KalmanFilter.h main.cpp -o tester -lglut -lGL -lGLU

clean:
	rm -rf ./tester

.PHONY: clean
