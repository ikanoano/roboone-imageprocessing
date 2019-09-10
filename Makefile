OBJS        =\
	Mikiri.o\
	OpponentUnit.o
LIBS        = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi
CPPFLAGS    = -std=c++17 -Wall -I /usr/include/opencv4 -mtune=native -march=native -mfpmath=both

ifdef PROF
	CPPFLAGS+=-pg -O2 -g
else
	CPPFLAGS+=-O3
endif

all: $(OBJS)

%.o: src/%.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -c

imtest: $(OBJS) test/main.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -o $@

clean:
	rm -f $(OBJS) imtest
