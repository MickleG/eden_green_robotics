#include <DynamixelSDK.h>

// Control table addresses for XM430
#define ADDR_TORQUE_ENABLE                  64
#define ADDR_GOAL_POSITION                  116
#define ADDR_PRESENT_POSITION               132
#define ADDR_GOAL_VELOCITY                  104
#define ADDR_GOAL_TORQUE                    102

// Protocol version
#define PROTOCOL_VERSION                    2.0

// Default setting
#define DXL_ID_LEFT                         1
#define DXL_ID_RIGHT                        2
#define BAUDRATE                            57600
#define DEVICENAME                          "/dev/ttyUSB0"  // Check which port is being used on your controller

#define TORQUE_ENABLE                       1
#define TORQUE_DISABLE                      0
#define DXL_LEFT_MINIMUM_POSITION_VALUE     2600
#define DXL_RIGHT_MINIMUM_POSITION_VALUE    3550
#define DXL_LEFT_MAXIMUM_POSITION_VALUE     3050
#define DXL_RIGHT_MAXIMUM_POSITION_VALUE    3100
#define DXL_MOVING_STATUS_THRESHOLD         20

#define MAX_ALLOWABLE_TORQUE                1023  // Max allowable torque
#define GOAL_SPEED                          100   // Goal speed (in Dynamixel units)


int main() {
    // Initialize PortHandler and PacketHandler instances
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler-&gt;openPort()) {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        return 0;
    }

    // Set port baud rate
    if (portHandler-&gt;setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        return 0;
    }

    // Enable Dynamixel Torque
    int dxl_comm_result = packetHandler-&gt;write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Dynamixel has been successfully connected\n");
    }

    // Set maximum allowable torque
    dxl_comm_result = packetHandler-&gt;write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_TORQUE, MAX_ALLOWABLE_TORQUE);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Maximum allowable torque set successfully\n");
    }

    // Set goal speed
    dxl_comm_result = packetHandler-&gt;write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, GOAL_SPEED);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Goal speed set successfully\n");
    }

    // Write goal position (360 degrees)
    int goal_position = 4095; // 4095 is one full rotation (360 degrees)
    dxl_comm_result = packetHandler-&gt;write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Goal position set to 360 degrees\n");
    }

    // Read present position
    int32_t present_position;
    dxl_comm_result = packetHandler-&gt;read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&amp;present_position);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Present position is %d\n", present_position);
    }

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler-&gt;write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler-&gt;getTxRxResult(dxl_comm_result));
    } else {
        printf("Torque disabled successfully\n");
    }

    // Close port
    portHandler-&gt;closePort();

    return 0;
}