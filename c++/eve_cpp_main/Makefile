# Variables
CC = g++
CFLAGS = -Wall -g
LIBS = -lwiringPi -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lrealsense2

# Include directories for OpenCV and librealsense
INC_DIRS = -I/usr/local/include/opencv4 -I/usr/local/include/librealsense2

# Library directories for OpenCV and librealsense
LIB_DIRS = -L/usr/local/lib

# Directories
POS_DIR = positioning

# Source files
SRC = main.cpp $(POS_DIR)/MotorConfig.cpp $(POS_DIR)/EndEffectorConfig.cpp
HEADERS = $(POS_DIR)/MotorConfig.h $(POS_DIR)/EndEffectorConfig.h
OBJ = main.o $(POS_DIR)/MotorConfig.o $(POS_DIR)/EndEffectorConfig.o

# Target
TARGET = main

# Rules
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LIB_DIRS) $(LIBS)

main.o: main.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(INC_DIRS) -I$(POS_DIR) -c $< -o $@

$(POS_DIR)/%.o: $(POS_DIR)/%.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(INC_DIRS) -I$(POS_DIR) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
