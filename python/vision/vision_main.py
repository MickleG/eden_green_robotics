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

	mask = cv2.inRange(color_image, np.array([30, 0, 0]), np.array([255, 50, 30]))

	isolated_mask = np.nonzero(mask)

	kernel = np.ones((2, 2), np.uint8)
	mask = cv2.erode(mask, kernel, iterations=3)
	mask = cv2.dilate(mask, kernel, iterations=3)

	u_indices = isolated_mask[1]
	v_indices = isolated_mask[0]

	return(depth_image, color_image, mask, u_indices, v_indices)

def collect_vine_mask(color_image):
	vine_mask = cv2.inRange(color_image, np.array([0, 0, 0]), np.array([60, 60, 60]))

	isolated_mask = np.nonzero(vine_mask)

	kernel = np.ones((4, 4), np.uint8)
	vine_mask = cv2.erode(vine_mask, kernel, iterations=1)
	vine_mask = cv2.dilate(vine_mask, kernel, iterations=1)

	return(vine_mask)

def distance(pt1, pt2):
	return(math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

def scissor_distance()
	vid = cv2.VideoCapture(2, cv2.CAP_DSHOW)

	while(vid.isOpened()):
		ret, frame = vid.read()
		if(ret):
			brighter = cv2.convertScaleAbs(frame, alpha = 3, beta = 25)
			hsv = cv2.cvtColor(brighter, cv2.COLOR_BGR2HSV)
			# (h, s, v) = cv2.split(hsv)

			# s = s * 1
			# s = np.clip(s,0,255)

			# hsv = cv2.merge([h,s,v])

			pt1 = (0,0)
			pt2 = (0,0)

			height, width, _ = frame.shape
			roi_start = int(0.5*width)
			top = int(height * 4/ 9)
			bottom = int(height * 5 / 9)
			roi = hsv[top:bottom, roi_start:]

			lower_red = np.array([0, 60, 60])
			upper_red = np.array([10, 255, 255])

			mask = cv2.inRange(roi, lower_red, upper_red)

			kernel = np.ones((4,4), np.uint8)
			mask = cv2.erode(mask, kernel, iterations=1)
			mask = cv2.dilate(mask, kernel, iterations=1)

			v_indices_r,u_indices_r = np.nonzero(mask)

			if u_indices_r.size > 0:

				avg_ur = int(statistics.mean(u_indices_r)+ (0.5*width)) 
				avg_vr = int(statistics.mean(v_indices_r))

				red_ptr = cv2.circle(brighter, (avg_ur, avg_vr), radius=4, color = [0,0,255], thickness=-2)
				pt1 = (avg_ur, avg_vr)


			#left
			roi_l = hsv[top:bottom, int(width/10):roi_start,:]

			mask_l = cv2.inRange(roi_l, lower_red, upper_red)

			mask_l = cv2.erode(mask_l, kernel, iterations=1)
			mask_l = cv2.dilate(mask_l, kernel, iterations=1)

			v_indices_l,u_indices_l = np.nonzero(mask_l)

			if u_indices_l.size > 0:

				avg_ul = int(statistics.mean(u_indices_l)+ int(width/10)) 
				avg_vl = int(statistics.mean(v_indices_l))

				red_ptl = cv2.circle(brighter, (avg_ul, avg_vl), radius=4, color = [0,0,255], thickness=-2)

				pt2 = (avg_ul, avg_vl)


			red_dist = (distance(pt1, pt2))

			last10 = []

			i = 0
			if i < 100:
				last10.append(red_dist)
				i +=1
			else:
				last10.remove(0)
				last10.append(red_dist)

			red_dist_avg = statistics.median(last10)

			centeru = int(width * 0.5)
			centerv = int(height * 0.5)

		cv2.waitKey(5)

	vid.release()
	cv2.destroyAllWindows()
	red_dist_avg = int(red_dist_avg)

	return(red_dist_avg)






