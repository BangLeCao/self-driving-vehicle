import rospy
import ros_numpy
from sensor_msgs.msg import Image, PointCloud2
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np 
import matplotlib.pyplot as plt
import math
import dlib
import time
import YOLO

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
                box = [startX, startY, endX - startX, endY - startY]
                boxes.append(box)
    return boxes

def get_box_info(box):
    (x, y, w, h) = [int(v) for v in box]
    center_X = int((x + (x + w)) / 2.0)
    center_Y = int((y + (y + h)) / 2.0)
    return x, y, w, h, center_X, center_Y

def remove_bad_trackers(curr_trackers, centers):
    remove = []
    for tracker in curr_trackers:
        centers_cnt = 0
        (x, y, w, h) = [int(v) for v in tracker['box']]
        for temp_tracker in curr_trackers:
            if temp_tracker['tracker_id'] <= tracker['tracker_id']:
                continue
            if cal_iou(temp_tracker['box'], tracker['box']) > 0.5:
                print('Remove collided object:', tracker['tracker_id'], 'collide with', temp_tracker['tracker_id'])
                remove.append(tracker['tracker_id'])
        for center in centers:
            (xc, yc) = [int(v) for v in center]
            if x < xc < x + w and y < yc < y + h:
                centers_cnt += 1
                if centers_cnt > 2:
                    print('Remove object:', tracker['tracker_id'])
                    remove.append(tracker['tracker_id'])
                    break
    for index in remove:
        for tracker in curr_trackers:
            if tracker['tracker_id'] == index: 
                curr_trackers.remove(tracker)

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

class Detection():
    def __init__(self):
        self.left = np.array([])
        self.right = np.array([])
        self.disparity = np.array([])
        self.pc = []
        self.f = 0
        self.T = 0
    def subscribe_left(self, msg):
        self.left = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
    def subscribe_right(self, msg):
        self.right = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
    def subscribe_disparity(self, msg):
        self.disparity = CvBridge().imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        self.f = msg.f # in pixels
        self.T = msg.T # in meters
    # def subscribe_pointcloud(self, msg):
    #     pc = ros_numpy.numpify(msg)
    #     self.pc = [pc['x'], pc['y'], pc['z']]

rospy.init_node('My_Detection')
detect = Detection()
rospy.Subscriber('/stereo_camera/left/image_rect', Image, detect.subscribe_left)
rospy.Subscriber('/stereo_camera/right/image_rect', Image, detect.subscribe_right)
rospy.Subscriber('/stereo_camera/disparity', DisparityImage, detect.subscribe_disparity)
# rospy.Subscriber('/stereo_camera/points2', PointCloud2, detect.subscribe_pointcloud)
pub = rospy.Publisher('My_Obstacles', Float32MultiArray, queue_size=10)
rate = rospy.Rate(20)

prototype_url = 'MobileNetSSD_deploy.prototxt'
model_url = 'MobileNetSSD_deploy.caffemodel'
net = cv2.dnn.readNetFromCaffe(prototype_url, model_url)

input_h = 480
input_w = 640
conf_threshold = 0.5

frame_count = 0
id_count = 0
curr_trackers = []
centers = []
start = time.time()

while not rospy.core.is_shutdown():    
    frame = detect.left
    frame_right = detect.right
    if frame.shape[0] == 0 or frame.shape[1] == 0 or frame_right.shape[0] == 0 or frame_right.shape[1] == 0:
        continue
    # print 'Frame', frame_count
    # print 'Frame.shape', frame.shape   
     
    frame = np.reshape(frame, (input_h, input_w, 1))
    frame3 = np.concatenate((frame, frame, frame), axis = 2)
    frame_gray = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY)
    result = np.copy(frame_gray)

    # remove_bad_trackers(curr_trackers, centers)
    old_trackers = curr_trackers
    curr_trackers = []
    centers = []
    publish_data = []

    for obj in old_trackers:
        tracker = obj['tracker']
        tracker.update(frame_gray)
        pos = tracker.get_position()
        x = int(max(pos.left(), 0))
        y = int(max(pos.top(), 0))
        w = int(pos.right() - x)
        h = int(pos.bottom() - y)
        center_X = int((x + (x + w)) / 2.0)
        center_Y = int((y + (y + h)) / 2.0)
        box = [x, y, w, h]
        obj['tracker'] = tracker       
        obj['box'] = np.array(box) 
        centers.append(np.array([center_X, center_Y]))   

        if detect.disparity.shape[0] > 0:
            obj['pre_pos'] = np.copy(obj['pos']).tolist()            
            # d = detect.disparity[center_Y, center_X]
            # pos_z = float(detect.f * detect.T / d)
            # pos_x = float((center_X - input_w/2) * pos_z / detect.f) - detect.T/2
            # pos_y = float((center_Y - input_h/2) * pos_z / detect.f) 
            # if np.array(detect.pc).shape[0] > 0:
            #     print 'pointcloud', np.array(detect.pc).shape, np.array(detect.pc)[:, center_Y, center_X] 
            # print [pos_x, pos_y, pos_z], center_X, center_Y, detect.T
            d = np.sort(detect.disparity[y:y+h, x:x+w], axis = None)[::-1]
            d_mean = np.mean(d[1000:min(3000, d.shape[0])])       
            pos_z = float(detect.f * detect.T / d_mean)
            pos_x = float((center_X - input_w/2) * pos_z / detect.f) - detect.T/2
            pos_y = float((center_Y - input_h/2) * pos_z / detect.f) 
            pos_w = float(w * pos_z / detect.f)
            pos_h = float(h * pos_z / detect.f)            
            obj['pos'] = [pos_x, pos_y, pos_z, pos_w, pos_h]
            vx = float(obj['pos'][1] - obj['pre_pos'][1])
            vy = float(obj['pos'][2] - obj['pre_pos'][2])     
        else:
            vx = obj['veloc'][-1][0]
            vy = obj['veloc'][-1][1]

        obj['veloc'].append([vx, vy])
        if len(obj['veloc']) >= 21:
            obj['veloc'] = obj['veloc'][1:21]
        vx_mean = np.mean(np.array(obj['veloc'])[:,0])
        vy_mean = np.mean(np.array(obj['veloc'])[:,1])
        # print vx_mean, vy_mean

        if x + w/2 < 0 or x + w/2 > input_w or y + h/2 < 0 or y + h/2 > input_h:
            # print 'Out of range: ', obj['tracker_id']
            continue
        publish_data.append(np.concatenate((obj['pos'][1:4], [vx_mean, vy_mean]), axis = 0).tolist())
        curr_trackers.append(obj)

        # cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # cv2.circle(result, (center_X, center_Y), 4, (0, 255, 0), -1)
        # cv2.putText(result, 'ID: ' + str(obj['tracker_id']), (center_X, center_Y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        # cv2.putText(result, 'z: ' + str(obj['pos'][2]), (center_X, center_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    if frame_count % 50 == 0:
        # boxes_d = get_object(net, frame3, conf_threshold, h=input_h, w=input_w)
        boxes_d = YOLO.yolo_detect(frame3)
        for box in boxes_d:
            skip_flag = 0
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
                        tracker = dlib.correlation_tracker()
                        tracker.start_track(frame_gray, dlib.rectangle(xd, yd, xd + wd, yd + hd))
                        obj['tracker'] = tracker
                        obj['box'] = np.array(box)
                        obj['detected'] = 1
                continue 
            
            id_count += 1
            tracker = dlib.correlation_tracker()
            tracker.start_track(frame_gray, dlib.rectangle(xd, yd, xd + wd, yd + hd))
            # cv2.rectangle(result, (xd, yd), ((xd + wd), (yd + hd)), (0, 255, 255), 2)
            # cv2.putText(result, str(id_count), (center_Xd + 15, center_Yd), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            new_obj = dict()
            new_obj['tracker_id'] = id_count
            new_obj['tracker'] = tracker
            new_obj['box'] = np.array([xd, yd, wd, hd])
            new_obj['veloc'] = []
            new_obj['veloc'].append([0, 0])
            new_obj['detected'] = 0
            new_obj['not_detected'] = -1
            new_obj['pos'] = [0, 0, 0, 0, 0] # x, y, z, w, h
            new_obj['pre_pos'] = [0, 0, 0, 0, 0] # x, y, z, w, h
            curr_trackers.append(new_obj)

        for obj in curr_trackers:
            if obj['detected'] == 0:
                obj['not_detected'] += 1
                if obj['not_detected'] >= 2:
                    curr_trackers.remove(np.array(obj))
            else:
                obj['detected'] = 0
                obj['not_detected'] = 0
    
    # print 'publish_data', publish_data
    msg = Float32MultiArray()
    msg.data = np.array(publish_data).flatten()
    pub.publish(msg)
    # cv2.imshow('Result', result)
    # cv2.waitKey(1)    
    frame_count += 1
    # rate.sleep()

end = time.time()
#print 'Number of frames:', frame_count
#print 'Execution time:', end - start
#print 'FPS:', frame_count/(end - start)
cv2.destroyAllWindows
    

