# coordinate system of RealSense (as mounted): X to the camera's right, Y to the camera's bottom, Z away from the camera

import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time
import sys

sys.path.append("../../vision")

from depth_stream import depth_stream

median_filter_buffer_u = []
median_filter_buffer_v = []

median_filter_size = 3
record_min_z = False
avg_min_z_u = 0
avg_min_z_v = 0

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

	depth_image, color_image, mask, u_indices, v_indices = depth_stream(pipe)

	if(len(v_indices) > 900): # cup detected, code for cup operations goes here
		avg_cup_u = int(statistics.mean(u_indices))
		avg_cup_v = int(statistics.mean(v_indices))
		avg_min_z_u = None
		avg_min_z_v = None
	else: # cup not detected, hugging vine code here
		avg_cup_u = None
		avg_cup_v = None

		min_val = np.min(depth_image[np.nonzero(depth_image)])
		min_z_index = list(zip(*np.where(depth_image == min_val)))

		min_z_v = min_z_index[0][0]
		min_z_u = min_z_index[0][1]


		avg_min_z_u = min_z_u
		avg_min_z_v = int(resolution[1] / 2)


		##median filtering the min_z indices to achieve less noise
		if(len(median_filter_buffer_u) <= median_filter_size):
			median_filter_buffer_u.append(min_z_u)
			# median_filter_buffer_v.append(min_z_v)
		else:
			avg_min_z_u = int(statistics.mean(median_filter_buffer_u))
			# avg_min_z_v = int(statistics.mean(median_filter_buffer_v))
			median_filter_buffer_u.pop(0)
			# median_filter_buffer_v.pop(0)
			median_filter_buffer_u.append(min_z_u)
			# median_filter_buffer_v.append(min_z_v)

		e = int(resolution[0] / 2 - avg_min_z_u)
		print(e)

	color_image = cv2.circle(color_image, (avg_cup_u, avg_cup_v), radius=2, color=[0, 255, 0], thickness=-1)
	color_image = cv2.circle(color_image, (avg_min_z_u, avg_min_z_v), radius=2, color=[0, 0, 255], thickness=-1)

	result = cv2.bitwise_and(depth_image, depth_image, mask=mask)


	cv2.imshow('Masked image', result)
	cv2.imshow('mask', mask)
	cv2.imshow('marked centroid', color_image)
	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break


pipe.stop()