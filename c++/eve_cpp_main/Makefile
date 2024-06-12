# Variables
CC = g++
CFLAGS = -Wall -g $(INCLUDES) -L/home/edengreen/DynamixelSDK/c++/build/linux64

# Include directories for librealsense
INC_DIRS = -I/usr/local/include/librealsense2 -I/home/edengreen/DynamixelSDK/c++/include/dynamixel_sdk `pkg-config --cflags opencv4`

# Library directories for OpenCV and librealsense
LIBS = -lwiringPi -lrealsense2 -ldxl_x64_cpp -lrt `pkg-config --libs opencv4`

# Directories
POS_DIR = positioning
GRIP_DIR = gripping
DIR_DXL = /home/edengreen/DynamixelSDK/c++

# Source files
SRC = main.cpp $(POS_DIR)/MotorConfig.cpp $(POS_DIR)/EndEffectorConfig.cpp $(GRIP_DIR)/Grip.cpp
HEADERS = $(POS_DIR)/MotorConfig.h $(POS_DIR)/EndEffectorConfig.h $(GRIP_DIR)/Grip.h
OBJ = main.o $(POS_DIR)/MotorConfig.o $(POS_DIR)/EndEffectorConfig.o $(GRIP_DIR)/.objects/Grip.o

INCLUDES   += -I$(DIR_DXL)/include/dynamixel_sdk


# Target
TARGET = main

# Rules
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

main.o: main.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(INC_DIRS) -I$(POS_DIR) -I$(GRIP_DIR) -c $< -o $@

$(POS_DIR)/%.o: $(POS_DIR)/%.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(INC_DIRS) -I$(POS_DIR) -c $< -o $@

$(GRIP_DIR)/.objects/%.o: $(GRIP_DIR)/%.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(INC_DIRS) -I$(GRIP_DIR) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean
