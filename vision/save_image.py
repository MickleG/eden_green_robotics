import pyrealsense2 as rs
import numpy as np
import cv2
import statistics
import time
import os

resolution = [424, 240]

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
pipe.start(cfg)

## wait for frames to become available
frames = pipe.wait_for_frames()

color_frame = frames.get_color_frame()
color_image = np.asanyarray(color_frame.get_data())

directory = r'C:\Users\ekriz\eden_green_robotics\vision'
filename = 'colortest.png'
os.chdir(directory)
cv2.imwrite(filename, color_image)

depth_frame = frames.get_depth_frame()

pipe.stop()