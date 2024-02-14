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

sys.path.append("../../vision")

from vision_main import depth_stream
from vision_main import collect_vine_mask

### import fiducial functions from vision_main here ###

import socket

### start webcam stream here ###


# setting up comms between pi and computer
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s.connect(("128.46.190.198", 9000)) # replace IP address with Raspberry Pi IP address
addr = ("128.46.190.170", 9000)

avg_min_z_u = None
avg_min_z_v = None
blue_detected = False
TARGET_Z = 75

cup_offset = 50 # value used to determine where to filter z

resolution = [424, 240]

# make connection to webcam
pipe = rs.pipeline()
# make variable for initiation calls
cfg = rs.config()


# set up rgb streaming (size: 424x240, format: 8 bit, fps: 30)
cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
# set up depth streaming (16-bit format)
cfg.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)

# start streaming
pipe.start(cfg)

while(True):

	depth_image, color_image, cup_mask, cup_u_indices, cup_v_indices = depth_stream(pipe)

	depth_image = depth_image[0:resolution[1], int(resolution[0] / 3):int(resolution[0] * 2 / 3)]
	color_image = color_image[0:resolution[1], int(resolution[0] / 3):int(resolution[0] * 2 / 3)]

	new_resolution = depth_image.shape

	if(len(cup_v_indices) > 900): # cup detected, code for cup operations goes here
		blue_detected = True

	else:
		blue_detected = False

	avg_cup_u = None
	avg_cup_v = None

	min_val = np.min(depth_image[np.nonzero(depth_image)]) + cup_offset * 10 # this value shows the cutoff for points to take the x-centroid at
	filtered_depth_image = np.where(np.array(depth_image) < min_val, np.array(depth_image), 0)
	avg_min_z_u = int(np.array(list(map(list, zip(*np.nonzero(filtered_depth_image))))).mean(axis=0)[1])
	avg_min_z_v = int(new_resolution[0] / 2)


	# calculating error and sending result to raspberry pi to run motors
	e_x = int(new_resolution[1] / 2 - avg_min_z_u)
	e_z = min_val
	# print(e_z)
	s.sendto(bytes(str(e_x) + ";" + str(int(e_z / 10)) + ";" + str(blue_detected), "utf-8"), addr) #signifying hug command with ";" to limit size of message

	color_image = cv2.circle(color_image, (avg_cup_u, avg_cup_v), radius=2, color=[0, 255, 0], thickness=-1)
	color_image = cv2.circle(color_image, (avg_min_z_u, avg_min_z_v), radius=2, color=[0, 0, 255], thickness=-1)

	cv2.imshow('marked centroid', color_image)
	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break


pipe.stop()
