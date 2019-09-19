CPPFLAGS    = -std=c++17 -Wall -g -I/usr/include/opencv4
ifdef CAMERA
	OBJS        =\
		Mikagiri.o\
		Mikiri.o\
		PolyFit.o\
		OpponentUnit.o
	LIBS        = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi -pthread
	CPPFLAGS   += -mtune=native -march=native -mfpmath=both
	CPPFLAGS   += -DUSE_CAMERA
else
	OBJS        =\
		Mikagiri.o\
		PolyFit.o\
		OpponentUnit.o
endif

ifdef DEBUG
	CPPFLAGS+=-DDEBUG_PREDICT
endif

ifdef EVAL
	CPPFLAGS+=-DEVAL_PREDICTION
endif

ifdef PROF
	CPPFLAGS+=-pg -O2
else
	CPPFLAGS+=-O3
endif

all: $(OBJS)

%.o: src/%.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -c

imtest: $(OBJS) test/main.cpp
	g++ $(CPPFLAGS) $(LIBS) $^ -o $@

clean:
	rm -f *.o imtest
