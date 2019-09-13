ifdef CAMERA
	OBJS        =\
		Mikagiri.o\
		Mikiri.o\
		OpponentUnit.o
	LIBS        = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi -pthread
	CPPFLAGS    = -std=c++17 -Wall -I /usr/include/opencv4 -mtune=native -march=native -mfpmath=both
	CPPFLAGS   += -DUSE_CAMERA
else
	OBJS        =\
		Mikagiri.o\
		OpponentUnit.o
	CPPFLAGS    = -std=c++17 -Wall
endif

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
