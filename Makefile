CPPFLAGS:=-std=c++17 -Wall -Wextra -Wdisabled-optimization -g -I/usr/include/opencv4 -MMD -MP
BLDDIR  :=build
TARGET  :=librobokenip.a
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
  CPPFLAGS+=-O2
endif

all: $(TARGET)

$(TARGET): $(OBJS)
	ar rcs $@ $^

$(BLDDIR)/%.o: src/%.cpp
	-mkdir -p $(BLDDIR)
	g++ $(CPPFLAGS) $(LIBS) $< -c -o $@

$(BLDDIR)/%.o: test/%.cpp
	-mkdir -p $(BLDDIR)
	g++ $(CPPFLAGS) $(LIBS) $< -c -o $@

imtest: $(BLDDIR)/main.o $(TARGET)
	g++ $(CPPFLAGS) $(LIBS) $^ -o $@
imtestrun: imtest
	./imtest

-include $(DEPS)

clean:
	rm -rf $(BLDDIR) $(TARGET) imtest gmon.out
