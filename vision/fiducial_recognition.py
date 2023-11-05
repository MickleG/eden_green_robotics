import apriltag
import cv2
import math

threshold = 50

vid = cv2.VideoCapture(0)

options = apriltag.DetectorOptions(families='tag16h5', 
								 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=False,
                                 debug=False,
                                 quad_contours=True)
detector = apriltag.Detector(options)

#insert code to calibrate start and end positions

def distance(pt1, pt2):
	return(math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2))

while(vid.isOpened()):
	ret, frame = vid.read()
	if(ret):
		frameProcessing = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	results = detector.detect(frameProcessing)
	k = cv2.waitKey(5)

	filtered_tags = []

	for r in results:
		# extract the bounding box (x, y)-coordinates for the AprilTag
		# and convert each of the (x, y)-coordinate pairs to integers
		(ptA, ptB, ptC, ptD) = r.corners

		if(distance(ptA, ptB) < threshold or distance(ptB, ptC) < threshold or distance(ptC, ptD) < threshold or distance(ptD, ptA) < threshold):
			break

		ptB = (int(ptB[0]), int(ptB[1]))
		ptC = (int(ptC[0]), int(ptC[1]))
		ptD = (int(ptD[0]), int(ptD[1]))
		ptA = (int(ptA[0]), int(ptA[1]))

		# draw the bounding box of the AprilTag detection
		cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
		cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
		cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
		cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

		(cX, cY) = (int(r.center[0]), int(r.center[1]))

		filtered_tags.append([cX, cY])

		cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

	print(filtered_tags)

	cv2.imshow('frame', frame)

vid.release()
cv2.destroyAllWindows()

