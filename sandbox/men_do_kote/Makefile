TARGET      = /tmp/men_do_kote
SRCS        = men_do_kote.cpp
LIBRARYS    = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gapi
CPPFLAGS    = -std=c++17 -I /usr/include/opencv4 

all:	$(TARGET)

$(TARGET): $(SRCS)
	g++ $(CPPFLAGS) $(LIBRARYS) $^ -o $@

clean:
	rm -rf $(TARGET)
