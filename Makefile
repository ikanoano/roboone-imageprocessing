OBJS        =\
	Mikiri.o\
	OpponentUnit.o
LIBS        = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi
CPPFLAGS    = -std=c++17 -I /usr/include/opencv4 -O3 -mtune=native -march=native -mfpmath=both

all: $(OBJS)

%.o: src/%.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -c

imtest: $(OBJS) test/main.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -o $@

clean:
	rm -f $(OBJS) imtest
