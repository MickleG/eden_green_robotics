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
addr = ("128.46.190.198", 9000)
median_filter_buffer_u = []
median_filter_buffer_v = []

median_filter_size = 1
record_min_z = False
avg_min_z_u = None
avg_min_z_v = None
TARGET_Z = 75

run_calibration = True

resolution = [424, 240]
finding_cup = False

# make connection to webcam
pipe = rs.pipeline()
# make variable for initiation calls
cfg = rs.config()

min_z_u_storage = []

# set up rgb streaming (size: 424x240, format: 8 bit, fps: 30)
cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
# set up depth streaming (16-bit format)
cfg.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, 30)

# start streaming
pipe.start(cfg)

while(True):

	depth_image, color_image, cup_mask, cup_u_indices, cup_v_indices = depth_stream(pipe)

	### collect fiducial image ###

	### if(run_calibration):
	###		do calibration
	###		run_calibration = False
	###send message to rpi to perform cut using s.sendto() otherwise known as socket

	### if(gripped):
	###		send true message
	###		check if cut
	###		if(cut_success):



	# contours, hierarchy = cv2.findContours(cup_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

	depth_image = depth_image[0:resolution[1], int(resolution[0] / 3):int(resolution[0] * 2 / 3)]
	color_image = color_image[0:resolution[1], int(resolution[0] / 3):int(resolution[0] * 2 / 3)]

	# cv2.imshow('depth image', depth_image)

	# depth_image = cv2.GaussianBlur(depth_image, (5, 5), 0)

	# cv2.imshow('blurred image', depth_image)

	new_resolution = depth_image.shape

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

		if(len(min_z_u_storage) < 1):
			min_z_u_storage.append(min_z_u)
			continue
		if(len(min_z_u_storage) == 1):
			min_z_u = int((min_z_u + min_z_u_storage[0]) / 2)

		avg_min_z_u = min_z_u
		# avg_min_z_v = int(resolution[1] / 2)
		avg_min_z_v = min_z_v


		# calculating error and sending result to raspberry pi to run motors
		e_x = int(new_resolution[1] / 2 - avg_min_z_u)
		e_z = min_val
		s.sendto(bytes(str(e_x) + ";" + str(int(e_z / 10)), "utf-8"), addr) #signifying hug command with ";" to limit size of message
	# for contour in contours:
	# 	mean_u = int(np.mean(contour[:, 0, 0]))
	# 	mean_v = int(np.mean(contour[:, 0, 1]))
	# 	color_image = cv2.circle(color_image, (mean_u, mean_v), radius=3, color=[0, 0, 255], thickness=-1)
	color_image = cv2.circle(color_image, (avg_cup_u, avg_cup_v), radius=2, color=[0, 255, 0], thickness=-1)
	color_image = cv2.circle(color_image, (avg_min_z_u, avg_min_z_v), radius=2, color=[0, 0, 255], thickness=-1)

	# result = cv2.bitwise_and(depth_image, depth_image, mask=cup_mask)


	# cv2.imshow('Masked image', result)
	# cv2.imshow('mask', cup_mask)
	cv2.imshow('marked centroid', color_image)
	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break


pipe.stop()
