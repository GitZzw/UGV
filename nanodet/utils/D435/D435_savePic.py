import pyrealsense2 as rs
from time import time
import numpy as np
import cv2
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

cfg = pipeline.start(config)
for i in range(1,2000):
    for j in range(1,10000):
        k = i+j
profile = cfg.get_stream(rs.stream.depth)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr) 

print('*************************\n')

profile2 = cfg.get_stream(rs.stream.color)
intr2 = profile2.as_video_stream_profile().get_intrinsics()
print(intr2) 
count = 0
print('*************************\n')
while True:
    next_frames = pipeline.wait_for_frames()
    get_next_color_frame = next_frames.get_color_frame()
    frame = np.asanyarray(get_next_color_frame.get_data())
    cv2.imshow("pic",frame)
    cv2.waitKey(1)
    cv2.imwrite("./savepics2/" + str(count)+".jpg", frame)
    count = count + 1
    for i in range(1,500):
        for j in range(1,500):
            k = i+j
    

