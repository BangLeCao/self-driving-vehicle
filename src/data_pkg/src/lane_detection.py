#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import copy
from itertools import product

def auto_canny(image, sigma = 0.5):
    v = np.median(image)

    lower = int(max(0, (1.0 - sigma)*v))
    upper = int(min(0, (1.0 + sigma)*v))
    edged = cv2.Canny(image, lower, upper)

    return edged

class Lane_Detection():
    def __init__(self):
        self.left = np.array([])
        self.bridge = CvBridge()
        self.new_left = 0

    def subscribe_left(self, msg):
        self.left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.new_left = 1


node_name = 'My_Lane_Detection_Node'
rospy.init_node(node_name)
lane_detect = Lane_Detection()
rospy.Subscriber('/stereo_camera/left/image_rect', Image, lane_detect.subscribe_left)
pub = rospy.Publisher('LFLAG', Float64MultiArray, queue_size=10)
rate = rospy.Rate(20)

input_h = 480
input_w = 640

atan2 = math.atan2

frame_count = -1
publish_data = []

draw = 0

start = time.time()

kernel = np.ones((11, 11),np.uint8)

horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))

lane_flag = 0

while not rospy.is_shutdown():
    start = time.time()
    frame = None
    while frame is None and not rospy.is_shutdown():
        if lane_detect.new_left == 1:
            frame = lane_detect.left
            frame_count += 1
            lane_detect.new_left = 0

    if rospy.is_shutdown():
        continue

    frame = np.reshape(frame, (input_h, input_w, 1))
    clone_frame = frame.copy()
    clone_frame = clone_frame[240:480, 0:640]
    clone_frame = np.array(255.0*((clone_frame/255.0)**0.4), dtype='uint8')


    blurred = cv2.GaussianBlur(clone_frame,(9,9),0)
    blurred = cv2.medianBlur(blurred, 11)
    blurred = cv2.blur(blurred,(9,9),51)

    thresholded = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
                    cv2.THRESH_BINARY,11,2)
    thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, horizontal_kernel, iterations=1)
    edged = auto_canny(thresholded)

    mask = np.zeros((edged.shape[0], edged.shape[1]), dtype="uint8")


    for i, j in product(range(48), range(128)):
        lines = cv2.HoughLines(edged[5*i:5*(i+1), 5*j:5*(j+1)], 1, np.pi/180, 1)
        if lines is not None:
            #if len(lines) > 5:
            mask[5*i:5*(i+1), 5*j:5*(j+1)] = np.zeros((5,5), dtype='uint8')
        else:
            mask[5*i:5*(i+1), 5*j:5*(j+1)] = np.ones((5,5), dtype='uint8')

    
    mask = cv2.erode(mask,kernel,iterations = 9)
    mask = cv2.dilate(mask, kernel, iterations = 4)
    mask = cv2.medianBlur(mask, 31)
        
    mask = cv2.dilate(mask, kernel, iterations = 11)
    mask = cv2.erode(mask,kernel,iterations = 4)
    mask = np.reshape(mask, (240, 640, 1))
    #print(mask)

    result = mask*clone_frame
    #print(result)

    left_result = result[:, :320]
    right_result = result[:, 320:]
    idx_left = np.array(np.where(left_result == 0))
    idx_right = np.array(np.where(right_result == 0))
    #print(idx_left.shape[1], idx_right.shape[1])
    if idx_left.shape[1] > 30000 and idx_left.shape[1] - idx_right.shape[1] > 10000:
        lane_flag = 1
    elif idx_right.shape[1] > 30000 and idx_right.shape[1] - idx_left.shape[1] > 10000:
        lane_flag = 2
    else:
        lane_flag = 0
    print(lane_flag)
    msg = Float64MultiArray()
    msg.data = np.array([lane_flag]).flatten()
    pub.publish(msg)

    if draw == 1:
        cv2.imshow("Clone", clone_frame)
        cv2.waitKey(1)
    end = time.time()
    #print [node_name], 'Execution time: {}'.format(end - start)

#end = time.time()
# print [node_name], 'Number of frames: {}'.format(frame_count - 1)
# print [node_name], 'Execution time: {}'.format(end - start)
# print [node_name], 'FPS:', (frame_count - 1)/(end - start)

# out.release()
# vid.release()
cv2.destroyAllWindows
