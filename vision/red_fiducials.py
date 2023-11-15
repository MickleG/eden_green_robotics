import cv2
import math
import numpy as np
import statistics

threshold = 50

vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)

#insert code to calibrate start and end positions

def distance(pt1, pt2):
	return(math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

while(vid.isOpened()):
	ret, frame = vid.read()
	if(ret):

		height, width, _ = frame.shape
		roi_start = int(0.5*width)
		roi = frame[:, roi_start:,:]

		lower_red = np.array([0, 0, 90])
		upper_red = np.array([50, 50, 255])

		mask = cv2.inRange(roi, lower_red, upper_red)

		kernel = np.ones((4,4), np.uint8)
		mask = cv2.erode(mask, kernel, iterations=1)
		mask = cv2.dilate(mask, kernel, iterations=1)

		v_indices_r,u_indices_r = np.nonzero(mask)

		if u_indices_r.size > 0:

			avg_ur = int(statistics.mean(u_indices_r)+ (0.5*width)) 
			avg_vr = int(statistics.mean(v_indices_r))

			red_ptr = cv2.circle(frame, (avg_ur, avg_vr), radius=4, color = [0,0,255], thickness=-2)


		#left
		roi_end = int(0.5*width)
		roi_l = frame[:, :roi_end,:]

		mask_l = cv2.inRange(roi_l, lower_red, upper_red)

		kernel_l = np.ones((4,4), np.uint8)
		mask_l = cv2.erode(mask_l, kernel, iterations=1)
		mask_l = cv2.dilate(mask_l, kernel, iterations=1)

		v_indices_l,u_indices_l = np.nonzero(mask_l)

		if u_indices_l.size > 0:

			avg_ul = int(statistics.mean(u_indices_l)) 
			avg_vl = int(statistics.mean(v_indices_l))

			red_ptl = cv2.circle(frame, (avg_ul, avg_vl), radius=4, color = [0,0,255], thickness=-2)

		cv2.imshow('color', frame)
		cv2.imshow('Red location', mask)
		cv2.imshow('left location', mask_l)

	cv2.waitKey(5)

vid.release()
cv2.destroyAllWindows()
