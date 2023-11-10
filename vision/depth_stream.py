import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time

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

	# print(color_image)

	mask = cv2.inRange(color_image, np.array([30, 0, 0]), np.array([255, 50, 30]))

	isolated_mask = np.nonzero(mask)

	kernel = np.ones((4, 4), np.uint8)
	mask = cv2.erode(mask, kernel, iterations=1)
	mask = cv2.dilate(mask, kernel, iterations=1)

	u_indices = isolated_mask[1]
	v_indices = isolated_mask[0]

	return(depth_image, color_image, mask, u_indices, v_indices)
