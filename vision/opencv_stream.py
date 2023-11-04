## Connecting Intel Realsense with Python and OpenCV

import pyrealsense2 as rs
import numpy as np
import cv2

## make connection to webcam
pipe = rs.pipeline()
## make variable for initiation calls
cfg = rs.config()

## set up rgb streaming (size: 640x480, format: 8 bit, fps: 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
## set up depth streaming (16-bit format)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

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
	
	depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha = 0.5), cv2.COLORMAP_JET)

	## show images
	cv2.imshow('rgb', color_image)
#	cv2.imshow('depth', depth_image)
	cv2.imshow('depth', depth_cm)

	## Finds minimum z distance in frame
	min_z = np.min(depth_image[np.nonzero(depth_image)])
#	print(min_z)

	## break out of screen if press q
	if cv2.waitKey(1) == ord('q'):
		break

pipe.stop()
