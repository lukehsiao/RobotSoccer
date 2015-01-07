# This Makefile works on my mac. Depending on where your opencv files
# are located, you'll need to change the directory specified below.

CC=g++
SANDBOX_DIR=/usr/local/Cellar/opencv/2.4.7.1
INCLUDE_FLAGS=-I$(SANDBOX_DIR)/include
CFLAGS= -Wall $(INCLUDE_FLAGS)
LDFLAGS= -L$(SANDBOX_DIR)/lib -lopencv_core -lopencv_highgui -lopencv_imgproc

all:	testvideo

testvideo:	testvideo.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) testvideo.cpp -o $@

clean :
	rm -f testvideo.o testvideo
