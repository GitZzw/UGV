#!/usr/bin/python3.6
import pyrealsense2 as rs
import numpy as np
import cv2
from time import time
pipeline = rs.pipeline()
# 创建 config 对象：
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
get_img_flag = False
count = 0
# while not (get_img_flag and count > 10): # 多取几幅图片，前几张不清晰
# 	# Wait for a coherent pair of frames（一对连贯的帧）: depth and color
# 	frames = pipeline.wait_for_frames()
# 	print('wait for frames in the first loop')
# 	get_color_frame = frames.get_color_frame()

# 	if not get_color_frame: # 如果color没有得到图像，就continue继续
# 	    continue

# 	color_frame = np.asanyarray(get_color_frame.get_data())
# 	get_img_flag = True # 跳出循环
# 	count += 1
while True:
	next_frames = pipeline.wait_for_frames()
	get_next_color_frame = next_frames.get_color_frame()
	next_color_frame = np.asanyarray(get_next_color_frame.get_data())
	cv2.imshow("color", next_color_frame)
	cv2.waitKey(1)
