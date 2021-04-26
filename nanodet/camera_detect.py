import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import rospy
import numpy as np
import os
import time
import torch
import argparse
from nanodet.util import cfg, load_config, Logger
from nanodet.model.arch import build_model
from nanodet.util import load_model_weight
from nanodet.data.transform import Pipeline

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped 
from std_msgs.msg import Float32
from nlink_parser.msg import LinktrackNodeframe3
import math


'''目标检测-摄像头'''
# python camera_detect.py webcam

global predictor
global pub,pub2
global dis,tempy

temy = 0.0

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('demo', default='webcam', help='demo type, eg. image, video and webcam')
    parser.add_argument('--config', default ='./config/nanodet_custom_xml_dataset.yml',help='model config file path')
    parser.add_argument('--model',default = './model/model_best.pth', help='model file path')
    parser.add_argument('--path', default='0', help='path to images or video')
    parser.add_argument('--camid', type=int, default=0, help='webcam demo camera id')
    args = parser.parse_args()
    return args


class Predictor(object):
    def __init__(self, cfg, model_path, logger, device='cuda:0'):
        self.cfg = cfg
        self.device = device
        model = build_model(cfg.model)
        ckpt = torch.load(model_path, map_location=lambda storage, loc: storage)
        load_model_weight(model, ckpt, logger)
        self.model = model.to(device).eval()
        self.pipeline = Pipeline(cfg.data.val.pipeline, cfg.data.val.keep_ratio)

    def inference(self, img):
        img_info = {}
        if isinstance(img, str):
            img_info['file_name'] = os.path.basename(img)
            img = cv2.imread(img)
        else:
            img_info['file_name'] = None

        height, width = img.shape[:2]
        img_info['height'] = height
        img_info['width'] = width
        meta = dict(img_info=img_info,
                    raw_img=img,
                    img=img)
        meta = self.pipeline(meta, self.cfg.data.val.input_size)
        meta['img'] = torch.from_numpy(meta['img'].transpose(2, 0, 1)).unsqueeze(0).to(self.device)
        with torch.no_grad():
            results = self.model.inference(meta)
        return meta, results

    def visualize(self, dets, meta, class_names, score_thres, wait=0):
        self.model.head.show_result(meta['raw_img'], dets, class_names, score_thres=score_thres, show=True)



def main():
    global pub,pub2
    global predictor
    global dis
    dis = 0
    args = parse_args()
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True
    load_config(cfg, args.config)
    logger = Logger(-1, use_tensorboard=False)
    predictor = Predictor(cfg, args.model, logger, device='cuda:0')
    logger.log('Press "Esc", "q" or "Q" to exit.')
    if args.demo == 'webcam':
        rospy.init_node('listener', anonymous=True)
        pub = rospy.Publisher('camera_detect', Vector3Stamped, queue_size=1)
        pub2 = rospy.Publisher('camera_detect2', Vector3Stamped, queue_size=1)
        rospy.Subscriber("/MVcam/image", Image, callback)   
        rospy.Subscriber("/nlink_linktrack_nodeframe3", LinktrackNodeframe3, callback2)
        rospy.spin()


def callback2(data):
    global dis
    
    if(data.nodes):
        dis = data.nodes[0].dis
    

def callback(data):
    global dis,tempy
    global pub,pub2
    global predictor
    t1 = time.time()
    distance = dis
    img = np.ndarray(shape=(data.height, data.width, 1),
	               dtype=np.uint8, buffer=data.data)
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # cv2.imshow("Nanodet",frame)    
    meta, res = predictor.inference(frame)
    predictor.visualize(res, meta, cfg.class_names, 0.30)
    all_box = []
    for label in res:
        for bbox in res[label]:           
            score = bbox[-1]
            if score>0.450:
                x0, y0, x1, y1 = [int(i) for i in bbox[:4]]
                all_box.append([label, x0, y0, x1, y1, score])
    all_box.sort(key=lambda v: v[5])

    send_msg = Vector3Stamped()
    send_msg2 = Vector3Stamped()
    if (len(all_box) == 0 or dis < 0.01):
        pass
        # send_msg.header.stamp = data.header.stamp
        # send_msg.vector.x = 0.0
        # send_msg.vector.y = 0.0
        # send_msg.vector.z = 0.0
        # pub.publish(send_msg)
    else:
        zzw = all_box[-1]
        label, x0, y0, x1, y1, score = zzw
        cx = 617.537944116307
        cy = 517.929086073535
        f = 1680.49567593274
        xmin = x0
        ymin = y0
        xmax = x1
        ymax = y1
        px = (xmin + xmax)/2
        deltax = px-cx
        py = (ymin + ymax)/2
        deltay = py-cy
        disz = distance/math.sqrt((abs(deltax)/f)*(abs(deltax)/f)+(abs(deltay)/f)*(abs(deltay)/f)+1)
        disx = disz*deltax/f
        disy = disz*deltay/f

        send_msg2.header.stamp = data.header.stamp
        send_msg.header.stamp = data.header.stamp
        # print(time.time()-t1)
        send_msg.vector.x = disz
        send_msg.vector.y = -disx
        send_msg.vector.z =  -disy

        tempy = disy
        if(abs(temy-disy)>1.0):
            disy = tempy
        else:    
            tempy = disy
            send_msg2.vector.x = disx
            send_msg2.vector.y = disy
            send_msg2.vector.z = disz

        # pub.publish(send_msg)
        pub2.publish(send_msg2)
    cv2.waitKey(1)
    


if __name__ == '__main__':
    main()
