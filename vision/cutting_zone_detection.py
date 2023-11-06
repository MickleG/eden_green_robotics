import pyrealsense2 as rs
import numpy as np
import cv2
import statistics

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


while True: 
	## wait for frames to become available
	frame = pipe.wait_for_frames()
	depth_frame = frame.get_depth_frame()
	color_frame = frame.get_color_frame()

	## convert images to numpy arrays
	color_image = np.asanyarray(color_frame.get_data())
	depth_image = np.asanyarray(depth_frame.get_data())

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

	if(len(v_indices) > 100 and len(u_indices) > 100):
		avg_u = int(statistics.mean(u_indices))
		avg_v = int(statistics.mean(v_indices))
	else:
		avg_u = None
		avg_v = None

	color_image = cv2.circle(color_image, (avg_u, avg_v), radius=2, color=[0, 255, 0], thickness=-1)



	result = cv2.bitwise_and(depth_image, depth_image, mask= mask)

	

	# for i in range(height):
	# 	for j in range(width):
	# 		print(depth_image[i][j])

	# break
	cv2.imshow('Masked image', result)
	cv2.imshow('mask', mask)
	cv2.imshow('marked centroid', color_image)

	
	# depth_image_filtered = depth_image * mask

	## Finds minimum z distance in frame
	# min_z = np.min(result[np.nonzero(result)]) / 10000
	# print(min_z)
	## Finds minimum y distance in frame
	# min_y = np.min(result[np.nonzero(result)]) / 10000
	# print(min_y)

	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break

pipe.stop()
