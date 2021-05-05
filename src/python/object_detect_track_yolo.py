import cv2
import math
import numpy as np
import time
import os
from numpy import dot
from scipy.linalg import inv, block_diag
import matplotlib.pyplot as plt

import YOLO

# Ham detect car và bus tu anh input
# def get_object(net, image, conf_threshold=0.5, h=480, w=854):
#     blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
#     net.setInput(blob)
#     detections = net.forward()
#     boxes = []

#     for i in range(0, detections.shape[2]):
#         confidence = detections[0, 0, i, 2]
#         if confidence > conf_threshold:
#             idx = int(detections[0, 0, i, 1])
#             # if 6 <= idx <= 7:
#             if idx == 2:
#                 box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
#                 (startX, startY, endX, endY) = box.astype("int")
#                 box = [startX, startY, endX - startX, endY - startY]
#                 boxes.append(box)

#     return boxes

def get_box_info(box):
    (x, y, w, h) = [int(v) for v in box]
    center_X = int((x + (x + w)) / 2.0)
    center_Y = int((y + (y + h)) / 2.0)
    return x, y, w, h, center_X, center_Y

def remove_bad_trackers(curr_trackers, centers):
  for tracker in curr_trackers:
    centers_cnt = 0
    (x, y, w, h) = [int(v) for v in tracker['box']]
    for center in centers:
      (xc, yc) = [int(v) for v in center]
      if x < xc < x + w and y < yc < y + h:
        centers_cnt += 1
        if centers_cnt > 2:
          # print('Frame: ', frame_count, 'Remove box: ', tracker['tracker_id'])
          curr_trackers.remove(tracker)
          break

# Define cac tham so

prototype_url = 'models/MobileNetSSD_deploy.prototxt'
model_url = 'models/MobileNetSSD_deploy.caffemodel'
video_path = 'test.mp4'

max_distance = 50
input_h = 480
input_w = 854
# laser_line = input_h - 50

# net = cv2.dnn.readNetFromCaffe(prototype_url, model_url)
vid = cv2.VideoCapture(video_path)

out = cv2.VideoWriter('/content/MiAI_Object_Detect_Tracking/output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 20.0, (854,480))

# Khoi tao tham so
frame_count = 0
# car_number = 0
obj_cnt = 0
curr_trackers = []
centers = []

start = time.time()

while vid.isOpened():

    remove_bad_trackers(curr_trackers, centers)    

    # Doc anh tu video
    _, frame = vid.read()
    if frame is None:
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    gray1 = np.reshape(gray, (480, 854, 1))
    gray = np.concatenate((gray1, gray1, gray1), axis = 2)

    result = gray
    # print(frame.shape)    

    # Resize nho lai
    # frame = cv2.resize(frame, (input_w, input_h))

    # Duyet qua cac doi tuong trong tracker
    old_trackers = curr_trackers
    curr_trackers = []
    centers = []

    for obj in old_trackers:

        # Update tracker
        tracker = obj['tracker']
        (ret, box) = tracker.update(gray1)
        # boxes.append(box)

        # Tinh toan tam doi tuong
        x, y, w, h, center_X, center_Y = get_box_info(box)
        centers.append(np.array([center_X, center_Y]))

        if ret == False:
          obj['tracking_failed'] += 1
          # print('Frame:', frame_count, obj['tracker_id'], 'tracking_failed', obj['tracking_failed'])
          if obj['tracking_failed'] == 20: 
            # if obj['tracker_id'] == 9:
            #   print('tracking_failed')
            continue
        else: 
          obj['tracking_failed'] = 0
          # z = np.array([x, y])
          # obj['state'].kalman_filter(z) 

        if x + w/2 < 0 or x + w/2 > input_w or y + h/2 < 0 or y + h/2 > input_h:
          # print('Frame:', frame_count,obj['tracker_id'], 'out of range')
          continue                  
                 
        new_obj = dict()
        new_obj['tracker_id'] = obj['tracker_id']
        new_obj['tracker'] = tracker
        # new_obj['state'] = obj['state']
        if ret == False:
          new_obj['box'] = obj['box']
        else:
          new_obj['box'] = np.array([x, y, w, h])
        new_obj['tracking_failed'] = obj['tracking_failed']   

        # if new_obj['tracker_id'] == 5:
        #   print('Frame: ', frame_count)
        #   print(z)
        #   print(new_obj['state'].x_state.T)
        #   print(new_obj['state'].K)
        #   print(new_obj['state'].y)

        # Ve hinh chu nhat quanh doi tuong
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Ve hinh tron tai tam doi tuong
        cv2.circle(result, (center_X, center_Y), 4, (0, 255, 0), -1)
        cv2.putText(result, str(obj['tracker_id']), (center_X - 30, center_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2) 
        # cv2.putText(result, str(obj['state'].x_state[3]), (center_X - 15, center_Y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)       

        curr_trackers.append(new_obj)
    
    tmp_trackers = curr_trackers
    # Thuc hien object detection moi 5 frame
    if frame_count % 10 == 0:
        # Detect doi tuong
        # boxes_d = get_object(net, gray)
        boxes_d = YOLO.yolo_detect(gray)

        for box in boxes_d:
            skip_flag = 0
            xd, yd, wd, hd, center_Xd, center_Yd = get_box_info(box)
            tmp_min = max_distance
            for obj in tmp_trackers:
              xt, yt, wt, ht, center_Xt, center_Yt = get_box_info(obj['box'])
              # center_Xt, center_Yt = int((xt + (xt + wt)) / 2.0), int((yt + (yt + ht)) / 2.0)
              distance = math.sqrt((center_Xt - center_Xd) ** 2 + (center_Yt - center_Yd) ** 2)
              # Duyet qua cac box, neu sai lech giua doi tuong detect voi doi tuong da track ko qua max_distance thi coi nhu 1 doi tuong
              if distance < tmp_min:
                tmp_min = distance
                skip_flag = 1
                if wd - wt > 30 or hd - ht > 30:
                  break
                cv2.rectangle(result, (xd, yd), ((xd + wd), (yd + hd)), (0, 255, 255), 2)
                cv2.putText(result, str(obj['tracker_id']), (center_Xd + 15, center_Yd), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                for tmp_tracker in curr_trackers:
                  if tmp_tracker['tracker_id'] == obj['tracker_id']:
                    tracker = cv2.TrackerMOSSE_create()
                    tracker.init(gray1, tuple(box))
                    tmp_tracker['tracker'] = tracker
                    tmp_tracker['box'] = box      
            if skip_flag == 1: continue
            
            cv2.rectangle(result, (xd, yd), ((xd + wd), (yd + hd)), (0, 255, 255), 2)
            
            tracker = cv2.TrackerMOSSE_create()
            tracker.init(gray1, tuple(box))
            # state = Kalman()
            # state.x_state = np.array([xd, yd, 1, 1]).T
            obj_cnt += 1
            cv2.putText(result, str(obj_cnt), (center_Xd + 15, center_Yd), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            new_obj = dict()
            new_obj['tracker_id'] = obj_cnt
            new_obj['tracker'] = tracker
            new_obj['box'] = np.array([xd, yd, wd, hd])
            # new_obj['state'] = state
            new_obj['tracking_failed'] = 0

            curr_trackers.append(new_obj)

    # Tang frame
    frame_count += 1
    # file_name = '/content/MiAI_Object_Detect_Tracking/results/' + str(frame_count) + '.png'
    # cv2.imwrite(file_name, result)
    # if frame_count == 176:
    #   print(curr_trackers)

    # out.write(result)
    # cv2.imshow("Image", frame)
    # key = cv2.waitKey(1) & 0xFF
    # if key == 27:
    #     break

end = time.time()
print('Number of frames: {}'.format(frame_count))
print('Execution time: {}'.format(end - start))
out.release()
vid.release()
cv2.destroyAllWindows
