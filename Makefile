INCLUDE+=-I/usr/include/qt5/
INCLUDE+=-I/usr/include/qt/

pfapp: pfapp.cc ParticleFilter.h
	g++ -g -std=c++11 $(INCLUDE) -o pfapp pfapp.cc -lQt5Core -lQt5Gui -lQt5Widgets -fPIC
