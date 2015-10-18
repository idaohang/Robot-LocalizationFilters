INCLUDE+=-I/usr/include/qt5/
INCLUDE+=-I/usr/include/qt/
INCLUDE+=-I/usr/include/qt4/
DNSDOMAINNAME := $(shell dnsdomainname)
ifeq (csres.utexas.edu, $(DNSDOMAINNAME))
LIB=-lQtCore -lQtGui
else
LIB=-lQt5Core -lQt5Gui -lQt5Widgets
endif

pfapp: test_pf.cc ParticleFilter.h MeanWindow.h MeanShift.h PFHelper.h
	g++ -g -std=c++11 -fPIC $(INCLUDE) -o pfapp test_pf.cc $(LIB)
