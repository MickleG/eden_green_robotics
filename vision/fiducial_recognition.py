
import cv2
import math

threshold = 50

vid = cv2.VideoCapture(0)

#insert code to calibrate start and end positions

def distance(pt1, pt2):
	return(math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

while(vid.isOpened()):
	ret, frame = vid.read()
	if(ret):
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		lower_red = np.array([0, 100, 100])
		upper_red = np.array([10, 255, 255])

		mask = cv2.inRange(hsv, lower_red, upper_red)

		color_image = cv2.bitwise_and(vid, vid, mask=mask)

		cv2.imshow('Red location', frame)

	cv2.waitKey(5)

vid.release()
cv2.destroyAllWindows()

