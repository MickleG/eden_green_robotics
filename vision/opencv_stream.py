import pyrealsense2 as rs
import numpy as np
import cv2

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

	# print(color_image)

	mask = cv2.inRange(color_image, np.array([30, 0, 0]), np.array([255, 50, 30]))

	cv2.imshow('mask', mask)

	# depth_image_filtered = depth_image * mask

	## Finds minimum z distance in frame
	min_z = np.min(depth_image[np.nonzero(depth_image)]) / 10000
	print(min_z)

	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break

pipe.stop()
