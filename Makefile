CPPFLAGS:=-std=c++17 -Wall -Wextra -Wdisabled-optimization -g -I/usr/include/opencv4 -MMD -MP
BLDDIR  :=build
OBJS:=\
	Mikagiri.o\
	PolyFit.o\
	OpponentUnit.o
ifdef CAMERA
  OBJS    +=Mikiri.o
  LIBS    :=-lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi -pthread
  CPPFLAGS+=-mtune=native -march=native -mfpmath=both
  CPPFLAGS+=-DUSE_CAMERA
endif
OBJS:=$(addprefix $(BLDDIR)/, $(OBJS))
DEPS:=$(patsubst %.o,%.d, $(OBJS))

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

$(BLDDIR)/%.o: src/%.cpp
	-mkdir -p $(BLDDIR)
	g++ $(CPPFLAGS) $(LIBS) $< -c -o $@

$(BLDDIR)/%.o: test/%.cpp
	-mkdir -p $(BLDDIR)
	g++ $(CPPFLAGS) $(LIBS) $< -c -o $@

imtest: $(OBJS) $(BLDDIR)/main.o
	g++ $(CPPFLAGS) $(LIBS) $^ -o $@

-include $(DEPS)

clean:
	rm -rf $(BLDDIR) imtest gmon.out
