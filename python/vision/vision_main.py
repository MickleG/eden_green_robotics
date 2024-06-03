import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time
import math


def distance(pt1, pt2):
	return(math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

def depth_stream(pipe):

	## wait for frames to become available
	frame = pipe.wait_for_frames()
	depth_frame = frame.get_depth_frame()
	color_frame = frame.get_color_frame()

	## convert images to numpy arrays
	color_image = np.asanyarray(color_frame.get_data())
	depth_image = np.asarray(depth_frame.get_data())

	dimensions = depth_image.shape
	height = dimensions[0]
	width = dimensions[1]

	mask = cv2.inRange(color_image, np.array([50, 0, 0]), np.array([255, 50, 30]))

	isolated_mask = np.nonzero(mask)

	kernel = np.ones((2, 2), np.uint8)
	mask = cv2.erode(mask, kernel, iterations=3)
	mask = cv2.dilate(mask, kernel, iterations=3)

	u_indices = isolated_mask[1]
	v_indices = isolated_mask[0]

	return(depth_image, color_image, mask, u_indices, v_indices)

def fiducials(frame):
	brighter = cv2.convertScaleAbs(frame, alpha = 3, beta = 10)
	hsv = frame

	pt1 = (0,0)
	pt2 = (0,0)

	height, width, _ = frame.shape
	roi_start = int(0.5*width)
	roi = hsv[:, roi_start:]

	lower_red = np.array([0, 0, 100])
	upper_red = np.array([70, 70, 255])

	mask = cv2.inRange(roi, lower_red, upper_red)

	v_indices_r,u_indices_r = np.nonzero(mask)

	if u_indices_r.size > 0:

		avg_ur = int(statistics.mean(u_indices_r)+ (0.5*width)) 
		avg_vr = int(statistics.mean(v_indices_r))
		pt1 = (avg_ur, avg_vr)


	#left
	roi_l = hsv[:,:roi_start]

	mask_l = cv2.inRange(roi_l, lower_red, upper_red)

	v_indices_l,u_indices_l = np.nonzero(mask_l)

	if u_indices_l.size > 0:

		avg_ul = int(statistics.mean(u_indices_l)+ int(width/10)) 
		avg_vl = int(statistics.mean(v_indices_l))

		pt2 = (avg_ul, avg_vl)

	red_dist_avg = int(distance(pt1, pt2))

	if(red_dist_avg < 250):
		cut_complete = 1
	else:
		cut_complete = 0
		
	return(cut_complete)

	cv2.imshow("color", hsv)