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
import warnings
warnings.filterwarnings('ignore')
from collections import Counter


def get_object(net, image, conf_threshold, h, w):
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()
    boxes = []
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            idx = int(detections[0, 0, i, 1])
            if idx == 2 or 6 <= idx <= 7 or 14 <= idx <= 15:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                box = [startX, startY, endX - startX, endY - startY, idx]
                boxes.append(box)
    return boxes

def get_box_info(box):
    (x, y, w, h) = [int(v) for v in box]
    center_X = int((x + (x + w)) / 2.0)
    center_Y = int((y + (y + h)) / 2.0)
    return x, y, w, h, center_X, center_Y

def cal_iou(boxA, boxB):
	xA = max(boxA[0], boxB[0])
	yA = max(boxA[1], boxB[1])
	xB = min(boxA[0]+boxA[2], boxB[0]+boxB[2])
	yB = min(boxA[1]+boxA[3], boxB[1]+boxB[3])
	interArea = max(0, xB - xA) * max(0, yB - yA)
	boxAArea = (boxA[2]) * (boxA[3])
	boxBArea = (boxB[2]) * (boxB[3])
	iou = interArea / float(boxAArea + boxBArea - interArea)
	return iou

def angle_2_vector(vx_org, vy_org, vx_obj, vy_obj):
    try:
        cosin = (vx_org*vx_obj + vy_org*vy_obj)/(sqrt(vx_org**2 + vy_org**2)*sqrt(vx_obj**2 + vy_obj**2))
    except:
        cosin = 1

    return acos(cosin)*180/pi

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

node_name = 'My_Detection_Node'
rospy.init_node(node_name)
detect = Detection()
rospy.Subscriber('/stereo_camera/left/image_rect', Image, detect.subscribe_left)
rospy.Subscriber('/stereo_camera/disparity', DisparityImage, detect.subscribe_disparity)
rospy.Subscriber('VDATA', Float64MultiArray, detect.subscribe_VDATA)
pub = rospy.Publisher('OBSTT', Float64MultiArray, queue_size=10)
rate = rospy.Rate(20)

prototype_url = '/home/nguyen/thesis_ws/weights/MobileNetSSD_deploy.prototxt'
model_url = '/home/nguyen/thesis_ws/weights/MobileNetSSD_deploy.caffemodel'
net = cv2.dnn.readNetFromCaffe(prototype_url, model_url)


input_h = 480
input_w = 640
conf_threshold = 0.5

sin = math.sin
cos = math.cos
tan = math.tan
atan = math.atan
atan2 = math.atan2
asin = math.asin
acos = math.acos
sqrt = math.sqrt
pi = math.pi

veloc_weights = np.array([1, 2, 4, 8, 16, 32, 64, 128, 256])
direction_tracker = list()
draw = 0
frame_count = -1
id_count = 0
curr_trackers = []
publish_data = []
# output_file = open('/home/nguyen/thesis_ws/test.txt', 'w')
sin = math.sin
cos = math.cos
classes = ['Background', 'Aeroplane', 'Bicycle', 'Bird', 'Boat', 'Bottle', 'Bus', 
            'Car', 'Cat', 'Chair', 'Cow', 'Dining table', 'Dog', 'Horse', 'Motorbike', 
            'Person', 'Potted plant', 'Sheep', 'Sofa', 'Train', 'TV/monitor']

start = time.time()

while not rospy.is_shutdown():
    #start = time.time()
    frame = None
    while frame is None and not rospy.is_shutdown():
        if detect.new_left == 1 and detect.new_disparity == 1 and detect.new_VDATA == 1:
        #if detect.new_left == 1 and detect.new_disparity == 1:
            frame = detect.left
            #print(frame.shape)
            frame_count += 1
            detect.new_left = 0
            disparity = detect.disparity
            f = detect.f
            b = detect.T
            detect.new_disparity = 0
            robot_pos = [detect.VDATA[0], detect.VDATA[1]]
            robot_yaw = detect.VDATA[2] # rad
            #robot_pos = [0, 0]
            #robot_yaw = 0
            detect.new_VDATA = 0
    if rospy.is_shutdown():
        continue
    frame = np.reshape(frame, (input_h, input_w, 1))
    
    frame3 = np.concatenate((frame, frame, frame), axis = 2)

    result = np.copy(frame)

    old_trackers = curr_trackers
    curr_trackers = []
    publish_data = []

    # //////////////////OBJECT TRACKING//////////////////////
    for obj in old_trackers:
        tracker = obj['tracker']
        (ret, box) = tracker.update(frame)
        obj['tracker'] = tracker
        x, y, w, h, center_X, center_Y = get_box_info(box)
        obj['pre_pos'] = np.copy(obj['pos']).tolist()

        if ret == False:
            obj['tracking_failed'] += 1
            if obj['tracking_failed'] == 20:
                continue
            vx = obj['veloc'][obj['veloc'].shape[0] - 1, 0]
            vy = obj['veloc'][obj['veloc'].shape[0] - 1, 1]
            obj['pos'] = (np.array(obj['pre_pos']) + np.array([vx, vy, 0])).tolist()
        else:
            obj['tracking_failed'] = 0
            obj['box'] = np.array(box)

            d = np.sort(disparity[max(y, 0):min(y+h, input_h), max(x, 0):min(x+w, input_w)], axis = None)
            hist, bin_edges = np.histogram(d, bins = 6)
            index = np.where(hist == hist[1:].max())[0][-1]
            d_mean = np.mean(d[sum(hist[0:index]):sum(hist[0:index + 1])])

            if d_mean != 0:
                pos_z = f * b / d_mean
                pos_x = (center_X - input_w/2) * pos_z / f - b/2
                pos_y = (center_Y - input_h/2) * pos_z / f
                pos_w = w * pos_z / f
                pos_h = h * pos_z / f
                #print('KC', pos_z)

                if draw == 1:
                    cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(result, (center_X, center_Y), 4, (0, 255, 0), -1)
                    cv2.putText(result, classes[obj['class']], (x, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv2.putText(result, 'Distance: ' + '%.3f' % (sqrt(pos_z**2 + pos_x**2)) + ' m', (center_X - 30, center_Y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(result, 'ID: ' + str(obj['tracker_id']), (center_X - 30, center_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0,), 2)

                obj['pos'] = [pos_x, pos_z, pos_w/2]
                # print obj['pos']
                x_rob = pos_x*sin(robot_yaw) + pos_z*cos(robot_yaw) + robot_pos[0]
                y_rob = -pos_x*cos(robot_yaw) + pos_z*sin(robot_yaw) + robot_pos[1]
                obj['pos'] = [x_rob, y_rob, pos_w/2]

                Timer = time.time() - obj['timer']
                obj['timer'] = time.time()

                if obj['pre_pos'][0] == 0 and obj['pre_pos'][1] == 0:
                    vx = 0
                    vy = 0
                else:
                    vx = (obj['pos'][0] - obj['pre_pos'][0])/Timer
                    vy = (obj['pos'][1] - obj['pre_pos'][1])/Timer
            else:
                vx = obj['veloc'][obj['veloc'].shape[0] - 1, 0]
                vy = obj['veloc'][obj['veloc'].shape[0] - 1, 1]
                obj['pos'] = (np.array(obj['pre_pos']) + np.array([vx, vy, 0])).tolist()

        if obj['veloc'].shape[0] >= 20:
            obj['veloc'] = obj['veloc'][1:21,]

        obj['veloc'] = np.concatenate((obj['veloc'], np.array([[vx, vy]])), axis = 0)
        vx_mean = np.mean(obj['veloc'][1:,0])
        vy_mean = np.mean(obj['veloc'][1:,1])


        if obj['veloc'].shape[0] >= 9:
            vx_mean_copy = np.average(obj['veloc'][-9:,0], axis = 0, weights=veloc_weights)
            vy_mean_copy = np.average(obj['veloc'][-9:,1], axis = 0, weights=veloc_weights)
            vx_veloc = np.average(abs(obj['veloc'][-9:,0]))
            vy_veloc = np.average(abs(obj['veloc'][-9:,1]))
        else:
            vx_mean_copy = vy_mean_copy = 0
            vx_veloc = vy_veloc = 0


        if abs(vx_mean_copy) < 5e-3:  
            vx_mean_copy  = 0
        if abs(vy_mean_copy) < 5e-3:
            vy_mean_copy = 0


        """ angle = angle_2_vector(cos(robot_yaw), sin(robot_yaw), vx_mean, vy_mean)
        if 0 < angle < 60:
            #print('Cung chieu')
            direction_tracker.append(0)
        elif 60 < angle < 120 and angle != 90:
            if vy_mean > 0:
                #print('Sang phai')
                direction_tracker.append(1)
            else:
                #print('Sang trai')
                direction_tracker.append(2)
        elif 120 < angle < 180:
            #print('Nguoc chieu')
            direction_tracker.append(3)
        elif angle == 0 or angle == 90:
            #print('Dung yen')
            direction_tracker.append(4) """
        
        """ if (len(direction_tracker) >= 3 and direction_tracker[-1] == direction_tracker[-2] == direction_tracker[-3]):
            if direction_tracker[-1] == 0:
                print('Cung chieu')
            elif direction_tracker[-1] == 1:
                print('Sang phai')
            elif direction_tracker[-1] == 2:
                print('Sang trai')
            elif direction_tracker[-1] == 3:
                print('Nguoc chieu')
            elif direction_tracker[-1] == 4:
                print('Dung yen') """

        #print(direction_tracker)
        obj['v_mean'] = np.array([vx_mean, vy_mean])

        velocity = sqrt(vx_veloc**2 + vy_veloc**2)

        if draw == 1:
            cv2.putText(result, 'v: ' + '%.3f' % velocity + ' m/s', (center_X - 30, center_Y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        data = np.concatenate((obj['pos'], [vx_mean, vy_mean]), axis = 0)
        publish_data.append(data.tolist())
        curr_trackers.append(obj)


    # //////////////////OBJECT DETECTION//////////////////////
    if frame_count % 10 == 0:
        boxes_d = get_object(net, frame3, conf_threshold, h=input_h, w=input_w)

        for box in boxes_d:
            skip_flag = 0
            class_idx = box[4]
            box = box[0:4]
            xd, yd, wd, hd, center_Xd, center_Yd = get_box_info(box)
            
            iou_max = 0.5
            for obj in curr_trackers:
                xt, yt, wt, ht, center_Xt, center_Yt = get_box_info(obj['box'])
                iou = cal_iou(box, obj['box'])
                if iou > iou_max:
                    iou_max = iou
                    max_id = obj['tracker_id']
                    skip_flag = 1

            if skip_flag == 1:
                for obj in curr_trackers:
                    if obj['tracker_id'] == max_id:
                        obj['detected'] = 1
                        if iou_max <= 0.9:
                            tracker = cv2.TrackerKCF_create()
                            #tracker = cv2.TrackerMOSSE_create()
                            tracker.init(frame, tuple(box))
                            # tracker = dlib.correlation_tracker()
                            # rect = dlib.rectangle(xd, yd, xd + wd, yd + hd)
                            # tracker.start_track(frame, rect)
                            obj['tracker'] = tracker
                            obj['box'] = np.array(box)
                        # if draw == 1:
                            # cv2.rectangle(result, (xd, yd), ((xd + wd), (yd + hd)), (0, 255, 255), 2)
                            # cv2.putText(result, str(obj['tracker_id']), (center_Xd + 15, center_Yd), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                continue

            id_count += 1
            tracker = cv2.TrackerKCF_create()
            #tracker = cv2.TrackerMOSSE_create()
            tracker.init(frame, tuple(box))
            new_obj = dict()
            new_obj['tracker_id'] = id_count
            new_obj['tracker'] = tracker
            new_obj['box'] = np.array([xd, yd, wd, hd])
            new_obj['veloc'] = np.array([[0, 0]])
            new_obj['v_mean'] = [0, 0]
            new_obj['tracking_failed'] = 0
            new_obj['detected'] = 0
            new_obj['not_detected'] = -1
            new_obj['pos'] = [0, 0, 0] # x, y, w
            new_obj['pre_pos'] = [0, 0, 0]
            new_obj['class'] = class_idx
            new_obj['timer'] = time.time()
            curr_trackers.append(new_obj)

        for obj in curr_trackers:
            if obj['detected'] == 0:
                obj['not_detected'] += 1
                if obj['not_detected'] >= 2:
                    curr_trackers.remove(np.array(obj))
            else:
                obj['detected'] = 0

    msg = Float64MultiArray()
    msg.data = np.array(publish_data).flatten()
    pub.publish(msg)
    #print(msg)
    if draw == 1:
        cv2.imshow('Result', result)
        cv2.waitKey(1)
    # rate.sleep()
    end = time.time()
    #print('FPS:', (frame_count - 1)/(end - start))
    #print('FPS:', fps)
    #print ('Execution time: {}'.format(end - start))

#end = time.time()
# print [node_name], 'Number of frames: {}'.format(frame_count - 1)
# print [node_name], 'Execution time: {}'.format(end - start)
# print [node_name], 'FPS:', (frame_count - 1)/(end - start)

# out.release()
# vid.release()
cv2.destroyAllWindows
