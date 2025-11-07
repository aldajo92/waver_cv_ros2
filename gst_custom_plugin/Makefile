CC = g++
CFLAGS = `pkg-config --cflags gstreamer-1.0 gstreamer-video-1.0`
LIBS = `pkg-config --libs gstreamer-1.0 gstreamer-video-1.0`
TARGET = libgstmyfilter.so
INSTALL_PATH = /usr/lib/aarch64-linux-gnu/gstreamer-1.0/

all: $(TARGET)

$(TARGET): gstmyfilter.cpp
	$(CC) -shared -fPIC $(CFLAGS) gstmyfilter.cpp -o $(TARGET) $(LIBS)

install: $(TARGET)
	sudo cp $(TARGET) $(INSTALL_PATH)
	sudo ldconfig

test:
	gst-inspect-1.0 myfilter

clean:
	rm -f $(TARGET)

.PHONY: all install test clean

