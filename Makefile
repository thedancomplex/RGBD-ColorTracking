default: all

BINARIES := openni-capture houghlines
all : $(BINARIES)

CFLAGS = `pkg-config --cflags opencv` -I/usr/local/include/opencv2  -g3 -Wall -c
LIBS = `pkg-config --libs opencv` -L/usr/lib -lpthread -ldl -lm -std=gnu++0x -std=c++0x -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect

% : %.cpp
	g++ $(CFLAGS) $(LIBS) -o $@ $<

openni-capture : src/openni_capture.cpp
	g++  $(CFLAGS) $(LIBS) -o openni-capture src/openni_capture.cpp

houghlines : src/houghlines.cpp
	g++  $(CFLAGS) $(LIBS) -o houchlines src/houghlines.cpp
clean:
	rm -f $(BINARIES) src/*.o
                                     
