import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time

## make connection to webcam
pipe = rs.pipeline()
## make variable for initiation calls
cfg = rs.config()

## set up rgb streaming (size: 640x480, format: 8 bit, fps: 30)
cfg.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
## set up depth streaming (16-bit format)
cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)

## start streaming
pipe.start(cfg)

collection_min_z_u = []
collection_min_z_v = []

target_frame_count = 5
record_min_z = False
avg_min_z_u = 0
avg_min_z_v = 0


while True: 
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

	if(len(v_indices) > 900): # cup detected, code for cup operations goes here
		avg_u = int(statistics.mean(u_indices))
		avg_v = int(statistics.mean(v_indices))
		min_z_u = None
		min_z_v = None
	else: # cup not detected, hugging vine code here
		avg_u = None
		avg_v = None

		min_val = np.min(depth_image[np.nonzero(depth_image)])
		min_z_index = list(zip(*np.where(depth_image == min_val)))

		min_z_v = min_z_index[0][0]
		min_z_u = min_z_index[0][1]

		if(len(collection_min_z_u) <= target_frame_count):
			collection_min_z_u.append(min_z_u)
			collection_min_z_v.append(min_z_v)
			record_min_z = True
		else:
			avg_min_z_u = int(statistics.mean(collection_min_z_u))
			avg_min_z_v = int(statistics.mean(collection_min_z_v))
			collection_min_z_u = []
			collection_min_z_v = []
			record_min_z = False

		# for i in range(len(depth_image)):
		# 	try:
		# 		min_val = np.min(depth_image[i][np.nonzero(depth_image[i])])
		# 		if(min_val < min_z):
		# 			min_z = min_val
		# 			min_z_v = i
		# 	except Exception as e:
		# 		print(e)

		# min_z_u = np.where(depth_image == min_z)[0][0]

	color_image = cv2.circle(color_image, (avg_u, avg_v), radius=2, color=[0, 255, 0], thickness=-1)
	if(record_min_z):
		color_image = cv2.circle(color_image, (avg_min_z_u, avg_min_z_v), radius=2, color=[0, 0, 255], thickness=-1)



	result = cv2.bitwise_and(depth_image, depth_image, mask= mask)

	cv2.imshow('Masked image', result)
	cv2.imshow('mask', mask)
	cv2.imshow('marked centroid', color_image)

	
	# depth_image_filtered = depth_image * mask


	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break

pipe.stop()
