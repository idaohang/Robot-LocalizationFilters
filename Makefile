INCLUDE+=-I/usr/include/qt5/
INCLUDE+=-I/usr/include/qt/
INCLUDE+=-I/usr/include/qt4/
DNSDOMAINNAME := $(shell dnsdomainname)
ifeq (csres.utexas.edu, $(DNSDOMAINNAME))
LIB=-lQtCore -lQtGui
else
LIB=-lQt5Core -lQt5Gui -lQt5Widgets
endif

kfapp: kfapp.cc KalmanFilter.h
	g++ -g -std=c++11 kfapp.cc -o kfapp -lglut -lGL -lGLU -I/usr/include/eigen3

ukfapp: ukfapp.cc UnscentedKalmanFilter.h
	g++ -g -std=c++11 ukfapp.cc -o ukfapp -lglut -lGL -lGLU -I/usr/include/eigen3

pfapp: test_pf.cc ParticleFilter.h MeanWindow.h MeanShift.h PFHelper.h
	g++ -g -std=c++11 -fPIC $(INCLUDE) -o pfapp test_pf.cc $(LIB)
