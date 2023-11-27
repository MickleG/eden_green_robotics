# coordinate system of RealSense (as mounted): X to the camera's right, Y to the camera's bottom, Z away from the camera
# 80mm between back ee plate and horizontal tangent of sheath
# 23mm thickness of realsense
# distance between front of realsense and horizontal tangent of sheath = 57mm
# target e_z = 65mm?

import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time
import sys
import gpiozero as gpio

sys.path.append("../../vision")

from vision_main import depth_stream
from vision_main import collect_vine_mask
from limit_switch import check_bounds

import socket

button1 = gpio.Button(2)
button2 = gpio.Button(5)
button3 = gpio.Button(3)
button4 = gpio.Button(6)

# setting up comms between pi and computer
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s.connect(("128.46.190.198", 9000)) # replace IP address with Raspberry Pi IP address
addr = ("128.46.190.198", 9000)
median_filter_buffer_u = []
median_filter_buffer_v = []

median_filter_size = 1
record_min_z = False
avg_min_z_u = 0
avg_min_z_v = 0
TARGET_Z = 65

resolution = [424, 240]
finding_cup = False

# make connection to webcam
pipe = rs.pipeline()
# make variable for initiation calls
cfg = rs.config()

# set up rgb streaming (size: 424x240, format: 8 bit, fps: 30)
cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 15)
# set up depth streaming (16-bit format)
cfg.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 15)

# start streaming
pipe.start(cfg)

while(True):
    if(check_bounds(button1, button2, button3, button4)):
        break

	depth_image, color_image, cup_mask, cup_u_indices, cup_v_indices = depth_stream(pipe)

	if(len(cup_v_indices) > 900 and finding_cup): # cup detected, code for cup operations goes here
		avg_cup_u = int(statistics.mean(cup_u_indices))
		avg_cup_v = int(statistics.mean(cup_v_indices))
		avg_min_z_u = None
		avg_min_z_v = None

		# s.send(bytes(str(avg_cup_u) + ";" + str(avg_cup_v), "utf-8"))
		s.sendto(bytes(str(avg_cup_u) + ":" + str(avg_cup_v), "utf-8"), addr) #signifying cup command with ":" to limit size of message


	else: # cup not detected, hugging vine code here
		avg_cup_u = None
		avg_cup_v = None

		min_val = np.min(depth_image[np.nonzero(depth_image)])
		min_z_index = list(zip(*np.where(depth_image == min_val)))

		min_z_v = min_z_index[0][0]
		min_z_u = min_z_index[0][1]

		avg_min_z_u = min_z_u
		avg_min_z_v = int(resolution[1] / 2)


		# calculating error and sending result to raspberry pi to run motors
		e_x = int(resolution[0] / 2 - avg_min_z_u)
		e_z = min_val - TARGET_Z
		s.sendto(bytes(str(e_x) + ";" + str(int(e_z / 10)), "utf-8"), addr) #signifying hug command with ";" to limit size of message
		

	# color_image = cv2.circle(color_image, (avg_cup_u, avg_cup_v), radius=2, color=[0, 255, 0], thickness=-1)
	color_image = cv2.circle(color_image, (avg_min_z_u, avg_min_z_v), radius=2, color=[0, 0, 255], thickness=-1)

	# result = cv2.bitwise_and(depth_image, depth_image, mask=cup_mask)


	# cv2.imshow('Masked image', result)
	# cv2.imshow('mask', cup_mask)
	cv2.imshow('marked centroid', color_image)
	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break


pipe.stop()
