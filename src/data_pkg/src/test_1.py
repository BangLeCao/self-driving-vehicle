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


class CropLayer(object):
    def __init__(self, params, blobs):
        self.xstart = 0
        self.xend = 0
        self.ystart = 0
        self.yend = 0

    # Our layer receives two inputs. We need to crop the first input blob
    # to match a shape of the second one (keeping batch size and number of channels)
    def getMemoryShapes(self, inputs):
        inputShape, targetShape = inputs[0], inputs[1]
        batchSize, numChannels = inputShape[0], inputShape[1]
        height, width = targetShape[2], targetShape[3]

        self.ystart = (inputShape[2] - targetShape[2]) // 2
        self.xstart = (inputShape[3] - targetShape[3]) // 2
        self.yend = self.ystart + height
        self.xend = self.xstart + width

        return [[batchSize, numChannels, height, width]]

    def forward(self, inputs):
        return [inputs[0][:,:,self.ystart:self.yend,self.xstart:self.xend]]
#! [CropLayer]

#! [Register]
cv2.dnn_registerLayer('Crop', CropLayer)
#! [Register]

# Load the model.
prototype_url_edge = '/home/nguyen/thesis_ws/weights/deploy.prototxt'
model_url_edge = '/home/nguyen/thesis_ws/weights/hed_pretrained_bsds.caffemodel'
net = cv2.dnn.readNetFromCaffe(prototype_url_edge, model_url_edge)


class Detection():
    def __init__(self):
        self.left = np.array([])
        self.right = np.array([])
        self.disparity = np.array([])
        self.pc = []
        self.bridge = CvBridge()
        self.f = 0
        self.T = 0
        self.new_left = 0
        self.new_right = 0
        self.new_disparity = 0
        self.VDATA = []
        self.new_VDATA = 0
    def subscribe_left(self, msg):
        self.left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.new_left = 1
    def subscribe_disparity(self, msg):
        self.disparity = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        self.f = msg.f # in pixels
        self.T = msg.T # in meters
        self.new_disparity = 1
    def subscribe_VDATA(self, msg):
        self.VDATA = list(msg.data)
        self.new_VDATA = 1
    # def subscribe_pc(self, msg):
    #     pc = ros_numpy.numpify(msg)
    #     self.pc = [pc['x'], pc['y'], pc['z']]

node_name = 'My_Lane_Detection_Node'
rospy.init_node(node_name)
detect = Detection()
rospy.Subscriber('/stereo_camera/left/image_rect', Image, detect.subscribe_left)
rospy.Subscriber('/stereo_camera/disparity', DisparityImage, detect.subscribe_disparity)
rospy.Subscriber('VDATA', Float64MultiArray, detect.subscribe_VDATA)
rate = rospy.Rate(20)

input_h = 480
input_w = 640

atan2 = math.atan2

frame_count = -1
publish_data = []

#start = time.time()

while not rospy.is_shutdown():
    
    frame = None
    while frame is None and not rospy.is_shutdown():
        #if detect.new_left == 1 and detect.new_disparity == 1 and detect.new_VDATA == 1:
        if detect.new_left == 1 and detect.new_disparity == 1:
            frame = detect.left
            #print(frame.shape)
            frame_count += 1
            detect.new_left = 0
            disparity = detect.disparity
            f = detect.f
            b = detect.T
            detect.new_disparity = 0
            #robot_pos = [detect.VDATA[0], detect.VDATA[1]]
            #robot_yaw = detect.VDATA[2] # rad
            robot_pos = [0, 0]
            robot_yaw = 0
            detect.new_VDATA = 0
    if rospy.is_shutdown():
        continue
    #print('Frame', frame_count)
    if frame_count % 10 == 0:
        start = time.time()
        frame = np.reshape(frame, (input_h, input_w, 1))
        frame3 = np.concatenate((frame, frame, frame), axis = 2)
    
        #print(frame3.shape)

        inp = cv2.dnn.blobFromImage(frame3, scalefactor=1.0, size=(input_w, input_h),
                                    mean=(104.00698793, 116.66876762, 122.67891434),
                                    swapRB=False, crop=False)
    
        net.setInput(inp)

        out = net.forward()
        out = out[0, 0]
        out = cv2.resize(out, (input_w, input_h))
        out_edge = out*255
        out_edge = np.uint8(out_edge)
        cv2.imshow("Edge", out_edge)
        end = time.time()
        print [node_name], 'Execution time: {}'.format(end - start)

#end = time.time()
# print [node_name], 'Number of frames: {}'.format(frame_count - 1)
# print [node_name], 'Execution time: {}'.format(end - start)
# print [node_name], 'FPS:', (frame_count - 1)/(end - start)

# out.release()
# vid.release()
cv2.destroyAllWindows
