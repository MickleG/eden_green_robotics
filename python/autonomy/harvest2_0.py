from dynamixel_sdk import * 
import os
import time
from gpiozero import OutputDevice
import sys, tty, termios

def harvest():
    import os
    import time
    from gpiozero import OutputDevice

    valve = OutputDevice(pin=16)
    cut = False

    if os.name == 'nt':
        import msvcrt
        def getch():
            return msvcrt.getch().decode()
    else:
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        def getch():
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    #***** (Use only one definition at a time) *****
    MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

    # Control table address
    ADDR_TORQUE_ENABLE= 64
    ADDR_GOAL_POSITION= 116
    ADDR_PRESENT_POSITION= 132
    DXL_MINIMUM_POSITION_VALUE= 2600         #closed position DXL1
    DXL_MAXIMUM_POSITION_VALUE= 3050      # opened position DXL1
    DXL2_MINIMUM_POSITION_VALUE= 3550  #closed position DXL2
    DXL2_MAXIMUM_POSITION_VALUE= 3100  #opened position DXL2
    BAUDRATE= 57600

    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Factory default ID of all DYNAMIXEL is 1
    DXL_ID                      = 1
    DXL_ID2                     = 2

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

    index = 0 # 0 for closing 1 for opening
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl2_goal_position = [DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE] 


    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    portHandler.openPort()

    portHandler.setBaudRate(BAUDRATE)
    
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        #print("Press any key to continue! (or press ESC to quit!)")
        #if getch() == chr(0x1b):
    #   if(not(input("press g to continue") == 'g')):
    #      continue

        # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
    dxl2_comm_result, dxl2_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION, dxl2_goal_position[index])

    while 1:
            # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        dxl2_present_position, dxl2_comm_result, dxl2_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION)

        # print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID2, dxl2_goal_position[index], dxl2_present_position))

        if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break
        if not abs(dxl2_goal_position[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break

    time.sleep(0.5)

    while not cut: 
        valve.on()
        cut_done = input("is cut made? (y/n)")
        if cut_done == "y":
            cut = True
            valve.off()
            break
        else: 
            cut = False
        valve.off()
        time.sleep(0.7)
    #    while(True):
    #       result = input("perform cut? (y/n)")
    #      if(result == "y"):
    #         valve.on()
        #        time.sleep(0.5)
        #       valve.off()

        #      result = input("done with cut? (y/n)")
        #     if(result == "y"):
            #        break



    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    # Close port
    portHandler.closePort()

