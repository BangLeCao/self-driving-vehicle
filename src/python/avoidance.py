import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
import matplotlib.pyplot as plt
import json
import math
import time
import os
from datetime import datetime

def velocity_distance(obstacles, r_rob, pos, yaw, start, stop, phi_max, safety):
    start = math.floor(start)
    stop = round(stop)
    # print('start', start, 'stop', stop)
    intervals = []
    append = intervals.append
    for obstacle in obstacles:
        r = obstacle[2] + r_rob + safety
        for t in range(start, stop + 1):
            # print('t', t)
            if obstacle[3] <= 0.03 and obstacle[4] <= 0.03 and t > start:
                break
            x = (obstacle[0] + obstacle[3]*t - pos[0])*sin(yaw) - (obstacle[1] + obstacle[4]*t - pos[1])*cos(yaw)
            y = (obstacle[0] + obstacle[3]*t - pos[0])*cos(yaw) + (obstacle[1] + obstacle[4]*t - pos[1])*sin(yaw)
            # print('obstacle', obstacle, pos, yaw)
            # print('x_new, y_new =', x, ',', y)
            if y < - (obstacle[2] + r_rob):
                # print('Out of sight!')
                continue
            # if x**2 + y**2 - r**2 < 0:
            #     if t == start:
            #         print('Collided with obstacle!')
            #         r = sqrt(x**2 + y**2 - 0.01)
            #     else:
            #         break
            if t == start:
                if x**2 + y**2 - (obstacle[2] + r_rob + safety)**2 > 0 and x**2 + y**2 - (obstacle[2] + r_rob + 3*safety/2)**2 <= 0:
                    r = obstacle[2] + r_rob + safety/2
                elif x**2 + y**2 - (obstacle[2] + r_rob)**2 > 0 and x**2 + y**2 - (obstacle[2] + r_rob + safety)**2 <= 0:
                    r = obstacle[2] + r_rob
                elif x**2 + y**2 - (obstacle[2] + r_rob)**2 <= 0:
                    # print('Collided with obstacle!')
                    r = sqrt(x**2 + y**2 - 0.01)
            elif x**2 + y**2 - r**2 < 0:
                break
            b = sqrt(x**2 + y**2)  # hypot() also works here
            phi = atan2(y, x) - pi/2
            if phi > pi:
                phi -= 2*pi
            elif phi < - pi:
                phi += 2*pi
            # print('r', r, 'b', b, 'asin(r/b)', asin(r/b))
            # print('phi', phi)
            phi1 = phi + asin(r/b)  # direction angle of point T1 from C
            phi2 = phi - asin(r/b)  # direction angle of point T2 from C
            # print('phi1', phi1, 'phi2', phi2)
            if (phi1 > phi_max and phi2 > phi_max) or (phi1 < -phi_max and phi2 < -phi_max):
                continue
            phi1 = phi_max if phi1 > phi_max else phi1
            phi2 = - phi_max if phi2 < - phi_max else phi2
            if x == 0:
                p1 = [0, y - r]
                p2 = [0, y + r]
            elif y == 0:
                p1 = [x + r, 0]
                p2 = [x - r, 0]
            else:
                a = (y - 0)/(x - 0)
                b = (-(y - 0)*x + (x - 0)*y)/(x - 0)
                p1, p2 = intersection_solve([x, y], r, a, b)
            d1 = sqrt(p1[0]**2+p1[1]**2)
            d2 = sqrt(p2[0]**2+p2[1]**2)
            # print('d1', d1, 'd2', d2)
            append([phi2, phi1, min(d1, d2, L)])
    intervals.sort(key=lambda x: x[0])
    # print(intervals)
    return intervals
def intersection_solve(center, r, a, b):
    # print(center, r, a, b)
    aa = 1 + a**2
    bb = -2*center[0] + 2*a*(b - center[1])
    cc = center[0]**2 + (b - center[1])**2 - r**2
    delta = bb**2 - 4*aa*cc
    # print(aa, bb, cc, delta)
    x1 = (- bb + sqrt(delta))/(2*aa)
    y1 = a*x1 + b
    # print('x1, y1', [x1, y1])
    x2 = (- bb - sqrt(delta))/(2*aa)
    y2 = a*x2 + b
    # print('x2, y2', [x2, y2])
    return [x1, y1], [x2, y2]
def min_union(old_intervals, phi_max, L):
    intervals = [[- phi_max, phi_max, L]]
    remove = intervals.remove
    append = intervals.append
    for new in old_intervals:
        c_min, c_max, d = new[0:3]
        temp = np.copy(intervals).tolist()
        for interval in temp:
            c1, c2, d12 = interval[0:3]
            if c_min <= c1 and c2 <= c_max:
                remove(interval)
                d12 = min(d, d12)
                append([c1, c2, d12])
            elif c1 <= c_min and c_max <= c2 and d < d12:
                remove(interval)
                if c1 < c_min:
                    append([c1, c_min, d12])
                append([c_min, c_max, d])
                if c_max < c2:
                    append([c_max, c2, d12])
            elif c1 <= c_min < c2 <= c_max and d < d12:
                remove(interval)
                if c1 < c_min:
                    append([c1, c_min, d12])
                if c_min < c2:
                    append([c_min, c2, d])
            elif c_min <= c1 < c_max <= c2 and d < d12:
                remove(interval)
                if c1 < c_max:
                    append([c1, c_max, d])
                if c_max < c2:
                    append([c_max, c2, d12])
    intervals.sort(key=lambda x: x[0])
    len_intervals = len(intervals)
    # print(intervals)
    i = 0
    new = []
    append = new.append
    while i < len_intervals:
        if i == len_intervals - 1:
            append(intervals[-1])
            break
        else:
            j = i + 1
            while j < len_intervals:
                # print(j)
                if intervals[j - 1][2] == intervals[j][2] and intervals[j - 1][1] == intervals[j][0]:
                    if j == len_intervals - 1:
                        interval = [intervals[i][0], intervals[j - 1][1], intervals[i][2]]
                        i = j + 1
                        break
                    j = j + 1
                else:
                    interval = [intervals[i][0], intervals[j - 1][1], intervals[i][2]]
                    append(interval)
                    i = j
                    break
    return new
def objective_function(pos, yaw, curr_index, intervals, path, phi_goal, v_max, T, old_phi):
    ft = []
    len_intervals = len(intervals)
    borders = [phi_goal/Tc]
    borders.extend(np.array(intervals)[:,0].tolist())
    borders.append(intervals[len_intervals - 1][1])
    # print('borders', borders)
    lanes = lane(pos, yaw, curr_index, path, borders, lane_width)
    append = ft.append
    for j in range(len_intervals):
        p_min, p_max, d = intervals[j]
        if p_min <= 0 and p_max >= 0:
            d0 = min(d, lane(pos, yaw, curr_index, path, [0], lane_width)[0])
        points = [[0,phi_goal/Tc], [v_max,phi_goal/Tc], [0,p_max], [v_max,p_max], [0,p_min], [v_max,p_min]]
        len_points = len(points)
        d_lane = [lanes[0], lanes[0], lanes[j+2], lanes[j+2], lanes[j+1], lanes[j+1]]
        for i in range(len_points):
            v, phi = points[i]
            dd = d_lane[i]
            if intervals[j][2] == L and 4<=i<=5 and j > 0 and intervals[j-1][2] < L:
                phi += 0.2
                dd = lane(pos, yaw, curr_index, path, [phi], lane_width)[0]
            if intervals[j][2] == L and 2<=i<=3 and j < len_intervals-1 and intervals[j+1][2] < L:
                phi -= 0.2
                dd = lane(pos, yaw, curr_index, path, [phi], lane_width)[0]
            # print(dd)
            d = min(dd, intervals[j][2])
            # print('d', d, 'd_lane', d_lane[i])
            v = min(v, d/T)
            if (v < 0 or v > v_max or phi < p_min or phi > p_max):
                # print('phi_goal:', phi_goal, p_min, p_max)
                # plt.show()
                continue
            f = alpha1*v/v_max + alpha2*d/L + alpha3*(1+alpha4*(phi_goal/pi)**2)*(1-abs(phi_goal-phi*Tc)/pi)
            # print('f', f, 'v', v, 'phi', phi, 'i', i, 'j', j, 'd', d)
            append([f, v, phi, j])
        # f = abs(phi_goal*np.ones(points.shape[1]) + Tc*points[1,:])/pi
        # f = - alpha3*(1+alpha4*(phi_goal/pi)**2)*f + alpha1/v_max*points[0,:]
        # f = f + (alpha2*d/L+alpha3*(1+alpha4*(phi_goal/pi)**2))*np.ones(points.shape[1])
        # f = np.concatenate((f.reshape((9,1)), points.T), axis = 1)
        # ft.extend(f.tolist())
    # print('ft', ft)
    ft.sort(key=lambda x: x[0], reverse = True)
    len_ft = len(ft)
    # print('ft', ft)
    if ft[0][0] < 0.7:
        tv = d0/T_imp
        rv = 0
        return tv, rv
    v_ftmax, phi_ftmax, j_ftmax = ft[0][1:4]
    # print('v_ftmax', v_ftmax, 'phi_ftmax', phi_ftmax)
    i_final = 0
    for i in range(i_final, len_ft):
        v, phi = ft[i][1:3]
        a = sqrt((pos[1]+v*np.sin(yaw+phi)-final_goal[1])**2+(pos[0]+v*np.cos(yaw+phi)-final_goal[0])**2)
        b = sqrt((pos[1]-final_goal[1])**2+(pos[0]-final_goal[0])**2)
        # print('a', a, 'b', b)
        if a < b:
            v_ftmax, phi_ftmax, j_ftmax = ft[i][1:4]
            break
    i_final = i
    # print('v_ftmax', v_ftmax, 'phi_ftmax', phi_ftmax)
    # if abs(phi_ftmax - old_phi) >= pi/2:
    # # if old_phi != 0:
    #     for i in range(i_final + 1, len_ft):
    #         if abs(phi_ftmax - old_phi) < pi/2 and abs(ft[0][0] - ft[i][0]) < 0.1:
    #             v_ftmax, phi_ftmax, j_ftmax = ft[i][1:4]
    #             break
    # print('v_ftmax', v_ftmax, 'phi_ftmax', phi_ftmax)
    return v_ftmax, phi_ftmax
def lane(pos, yaw, curr_index, path, pts, width):
    i1 = curr_index if curr_index >= len(path) - 1 else curr_index+1
    i2 = curr_index-1 if curr_index >= len(path) - 1 else curr_index
    alpha = atan2((path[i1]['x']-path[i2]['x']), -(path[i1]['y']-path[i2]['y']))
    x1 = path[i1]['x']-width*cos(alpha) # right lane
    y1 = path[i1]['y']-width*sin(alpha)
    x2 = path[i2]['x']-width*cos(alpha)
    y2 = path[i2]['y']-width*sin(alpha)
    x3 = path[i1]['x']*2 - x1 # left lane
    y3 = path[i1]['y']*2 - y1
    x4 = path[i2]['x']*2 - x2
    y4 = path[i2]['y']*2 - y2
    if draw == 1:
        x = np.linspace(x2, x1, 10)
        ax.plot(x, (x-x2)/(x1-x2)*(y1-y2)+y2)
        x = np.linspace(x4, x3, 10)
        ax.plot(x, (x-x4)/(x3-x4)*(y3-y4)+y4)
    new = robot_coordinate(pos, yaw, [[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
    [x1, y1], [x2, y2], [x3, y3], [x4, y4] = new
    # print(x1, y1, x2, y2, x3, y3, x4, y4)
    d_lane = []
    append = d_lane.append
    for point in pts:
        phi = point
        # print('phi', phi)
        phi = phi + pi/2
        if phi > pi:
            phi -= 2*pi
        elif phi < -pi:
            phi += 2*pi
        line1 = -x1 if x1==x2 else ((-x2)/(x1-x2)*(y1-y2)+y2)
        line2 = -x3 if x3==x4 else ((-x4)/(x3-x4)*(y3-y4)+y4)
        if line1*line2 > 0:
            if sqrt(x1**2+y1**2) <= sqrt(x3**2+y3**2):
                x_right, y_right = 0, 0
                x_left = x3 if x3==x4 else (x4*(y3-y4)/(x3-x4)-y4)/((y3-y4)/(x3-x4)-tan(phi))
                y_left = tan(phi)*x_left
                d = sqrt(x_left**2+y_left**2)
            else:
                x_left, y_left = 0, 0
                x_right = x1 if x1==x2 else (x2*(y1-y2)/(x1-x2)-y2)/((y1-y2)/(x1-x2)-tan(phi))
                y_right = tan(phi)*x_right
                d = sqrt(x_right**2+y_right**2)
        else:
            x_right = x1 if x1==x2 else (x2*(y1-y2)/(x1-x2)-y2)/((y1-y2)/(x1-x2)-tan(phi))
            y_right = tan(phi)*x_right
            x_left = x3 if x3==x4 else (x4*(y3-y4)/(x3-x4)-y4)/((y3-y4)/(x3-x4)-tan(phi))
            y_left = tan(phi)*x_left
            if phi-0.01<=atan2(y_right, x_right)<=phi+0.01:
                d = sqrt(x_right**2+y_right**2)
            elif phi-0.01<=atan2(y_left, x_left)<=phi+0.01:
                d = sqrt(x_left**2+y_left**2)
        # print('x_right', x_right, 'y_right', y_right,'x_left', x_left, 'y_left', y_left, )
        # print(atan2(y_right, x_right), atan2(y_left, x_left), phi)
        # print('phi', phi, 'd', d)
        append(d)
    return d_lane
def robot_coordinate(pos, yaw, points):
    new = []
    append = new.append
    for point in points:
        x, y = point
        x_new = (x - pos[0])*sin(yaw) - (y - pos[1])*cos(yaw)
        y_new = (x - pos[0])*cos(yaw) + (y - pos[1])*sin(yaw)
        append([x_new, y_new])
    return new

class Avoidance():
    def __init__(self):
        self.obj = []
        self.VDATA = []
        self.new_obj = 0
        self.new_VDATA = 0
    def subscribe_obj(self, msg):
        self.obj = np.array(msg.data)
        self.obj.reshape((5, int(self.pos.shape[0]/5)))
        self.obj.tolist()
        self.new_obj = 1
    def subscribe_VDATA(self, msg):
        self.VDATA = msg.data
        self.VDATA.tolist()
        self.new_VDATA = 1

rospy.init_node('My_Avoidance')
avoid = Avoidance()
rospy.Subscriber('OBSTT', Float64MultiArray, avoid.subscribe_obj)
rospy.Subscriber('VDATA', Float64MultiArray, avoid.subscribe_VDATA)
pub = rospy.Publisher('PCDAT', Float64MultiArray, queue_size=10)
rate = rospy.Rate(20)
# //////////////////Parameters/////////////////////
draw = 0
dist_goal = 1 # (m) distance between robot_pos and final_goal where the robot will stop
r_rob = 0.2 # (m) radius of the robot
v_max = 0.3 # (m/s) maximum translational velocity
phi_max = v_max*2/0.388 # (rad/s) maximum rotaional velocity, 0.388(m): distance between two wheels
T_imp = 2/v_max # (s) robot will be able to travel at least T_imp seconds before hitting an obstacle
alpha1 = 0.1 # weights in the objective function
alpha2 = 0.3 # weights in the objective function
alpha3 = 0.6 # weights in the objective function
alpha4 = 0 # weights in the objective function
safety_margin = 0.1 # (m) Increase size of obstacles by a safety margin
path_cut = 10 # only consider path in range [i_curr, i_curr + path_cut)
lane_width = 1 # (m) the width of the lane
temp_goal = 2  # the offset added to the max index of the path
                # that an obstacle collides with to determine a temporary goal
temp_nextgoal = 1 # if the robot is closer to path[j] than temporary goal,
                  # the new temporary goal is path[j + temp_nextgoal]
# //////////////////Init///////////////////////////
L = v_max * T_imp
old_phifinal = 0
detected_flag = 0
i_goal = None
i_prevgoal = None
input_data = []
total_time = []
p_rob = []
p_obs = []
detected = []
msg = Float64MultiArray()
sin = math.sin
cos = math.cos
tan = math.tan
atan = math.atan
atan2 = math.atan2
asin = math.asin
sqrt = math.sqrt
pi = math.pi
# /////////////////////////////////////////////////
f = open('/home/nguyen/thesis_ws/data.txt', 'w')
path = '/home/nguyen/thesis_ws/map_out'
len_path = 0
count = 0
# /////////////////////////////////////////////////
start = time.time()
while not rospy.is_shutdown():
    # //////////////////Read map///////////////////////
    if len_path == 0: print 'Waiting to read map...'
    while len_path == 0:    
        if os.path.isfile(path):
            json_input = open(path)
            input_path = json.load(json_input)
            print 'Successfully read the map!'
            os.remove(path)
            len_path = len(input_path)
            print 'Length of the map:', len_path
            final_goal = [input_path[-1]['x'], input_path[-1]['y']]
            i_finalgoal = len_path - 1   
    # print('Count:', count)   
    # //////////////////Update/////////////////////////
    pre_obs = np.copy(detected)
    # pre_detectedflag = detected_flag
    robot_pos = []
    while len(robot_pos) == 0:
        if avoid.new_VDATA == 1 and avoid.new_obj == 1:
            robot_pos = [avoid.VDATA[0], avoid.VDATA[1]]
            i_curr = avoid.VDATA[3]
            robot_yaw = avoid.VDATA[2] 
            avoid.new_VDATA = 0
            new = avoid.obj
            avoid.new_obj = 0
        elif avoid.new_VDATA == 0:
            print 'Waiting to read VDATA...'
        elif avoid.new_obj == 0:
            print 'Waiting to read OBSTT...'
    append = detected.append
    for obj in new:
        x = obj[0]*sin(robot_yaw)+obj[1]*cos(robot_yaw)+robot_pos[0]
        y = -obj[0]*cos(robot_yaw)+obj[1]*sin(robot_yaw)+robot_pos[1]
        w = obj[2]
        vx = obj[3]*sin(robot_yaw)+obj[4]*cos(robot_yaw)
        vy = -obj[3]*cos(robot_yaw)+obj[4]*sin(robot_yaw)
        append([x, y, w, vx, vy])
    len_detected = len(detected)
    # print('detected', detected)      
    step = 1 if i_curr<=i_finalgoal else -1
    # //////////////////Draw///////////////////////////
    if draw == 1:
        if p_rob == [] and p_obs == []:
            fig, ax = plt.subplots(1)
            plt.grid(True)
            plt.axis("equal")
            plt.plot(input_data['x'], input_data['y'], "-y")
            p_goal = plt.plot(final_goal[0], final_goal[1], "ob")
            append = p_obs.append
            for obstacle in detected:
                append(plt.plot(obstacle[0], obstacle[1], "og"))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='g', alpha=0.5, edgecolor='None'))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='y', alpha=0.5, edgecolor='None'))
        else:
            # p_rob[0].remove()
            for i in range(len(p_obs)):
                p_obs[i][0].remove()
            p_obs = []
            for obstacle in pre_obs:
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='w', alpha=1, edgecolor='w'))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='w', alpha=1, edgecolor='w'))
            append = p_obs.append
            for obstacle in detected:
                append(plt.plot(obstacle[0], obstacle[1], "og"))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, facecolor='g', alpha=0.5, edgecolor='None'))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, facecolor='y', alpha=0.5, edgecolor='None'))
        p_rob = plt.plot(robot_pos[0], robot_pos[1], "or")
        # x = np.linspace(robot_pos[0], robot_pos[0] + v_final*np.cos(robot_yaw), 100)
        # ax.plot(x, tan(robot_yaw)*(x - robot_pos[0]) + robot_pos[1])
    # /////////////////////////////////////////////////
    if sqrt((robot_pos[0] - final_goal[0])**2 + (robot_pos[1] - final_goal[1])**2) > dist_goal:
        # start_1 = time.time()
        # print('robot_pos', robot_pos, 'robot_yaw', robot_yaw)
        # print('i_curr', i_curr, 'i_goal', i_goal, 'i_finalgoal', i_finalgoal)
        temp = 0
        if len_detected > 0:            
            for j in range(i_curr, min(i_curr+path_cut, i_finalgoal) if step==1 else max(i_curr-path_cut, i_finalgoal), step):
                if j == i_curr:
                    s = 0
                else:
                    s = s + sqrt((input_path[j]['x'] - input_path[j - step]['x']) ** 2 + (input_path[j]['y'] - input_path[j - step]['y']) ** 2)
                t = s/v_max
                for obstacle in detected:
                    distance = sqrt((input_path[j]['x'] - (obstacle[0] + obstacle[3]*t)) ** 2 + (input_path[j]['y'] - (obstacle[1] + obstacle[4]*t)) ** 2)
                    if distance <= obstacle[2] + r_rob + safety_margin:
                        temp = 1
                        i_start = i_curr
                        t_start = t
                        break
                if temp == 1:
                    detected_flag = 1
                    break
        # print('detected_flag', detected_flag)
        if detected_flag == 1:
            count += 1
            if temp == 1:
                collision = []
                append = collision.append
                for obstacle in detected:
                    for j in range(i_curr, i_finalgoal, step):
                        dist1 = sqrt((input_path[j]['x'] - (obstacle[0] + obstacle[3]*t))**2 + (input_path[j]['y'] - (obstacle[1] + obstacle[4]*t))**2)
                        dist2 = sqrt((input_path[j + step]['x'] - (obstacle[0] + obstacle[3]*t_start))**2 + (input_path[j + step]['y'] - (obstacle[1] + obstacle[4]*t_start))**2)
                        if dist1 <= obstacle[2] + r_rob + safety_margin and dist2 >= obstacle[2] + r_rob + safety_margin:
                            append(j + step)
                            break
                # print('collision', collision)
                if len(collision) == 0:
                    i_goal = i_finalgoal
                else:
                    i_goal = min(max(collision)+temp_goal, i_finalgoal) if step==1 else max(min(collision)-temp_goal, i_finalgoal)
            for j in range(i_goal + step, i_finalgoal, step):
                dist1 = sqrt((robot_pos[0] - input_path[i_goal]['x'])**2 + (robot_pos[1] - input_path[i_goal]['y'])**2)
                dist2 = sqrt((robot_pos[0] - input_path[j]['x'])**2 + (robot_pos[1] - input_path[j]['y'])**2)
                dist3 = sqrt((robot_pos[0] - input_path[j + step]['x'])**2 + (robot_pos[1] - input_path[j + step]['y'])**2)
                if dist2 <= dist1 and dist2 <= dist3:
                    i_goal = min(j+temp_nextgoal, i_finalgoal) if step==1 else max(j-temp_nextgoal, i_finalgoal)
                    break
            goal = [input_path[i_goal]['x'], input_path[i_goal]['y']]
            # print('i_goal', i_goal)
            s = sqrt((input_path[i_curr]['x'] - goal[0]) ** 2 + (input_path[i_curr]['y'] - goal[1]) ** 2)
            t_stop = s/v_max
            if draw == 1:
                p_goal[0].remove()
                p_goal = plt.plot(goal[0], goal[1], "ob")
            if sqrt((robot_pos[0] - goal[0])**2 + (robot_pos[1] - goal[1])**2) > dist_goal:
                obstacle_intervals = velocity_distance(detected, r_rob, robot_pos, robot_yaw, t_start, t_stop, phi_max, safety_margin)
                # print('obstacle_intervals', obstacle_intervals)
                new_intervals = min_union(obstacle_intervals, phi_max, L)
                # print('new_intervals', new_intervals)
                phi_goal = atan2(goal[1] - robot_pos[1], goal[0] - robot_pos[0]) - robot_yaw
                if phi_goal < -pi:
                    phi_goal += 2*pi
                elif phi_goal > pi:
                    phi_goal -= 2*pi
                # print('phi_goal', phi_goal)
                v_final, phi_final = objective_function(robot_pos, robot_yaw, i_curr, new_intervals, input_path, phi_goal, v_max, T_imp, old_phifinal)
                # print('v_final', v_final, 'phi_final', phi_final)
                phi_final = - phi_final
                old_phifinal = phi_final
                msg.data = np.array([v_final, phi_final])
                pub.publish(msg)
                # count = count + 1
            else:
                detected_flag = 0
                msg.data = np.array([detected_flag])
                pub.publish(msg)
                count = 0
                if draw == 1:
                    p_goal[0].remove()
                    p_goal = plt.plot(final_goal[0], final_goal[1], "ob")
        # end_1 = time.time()
        # total_time.append(end_1 - start_1)      
    else:
        print 'Robot reached its destination.'
        len_path = 0

    if detected_flag == 0:
        f.write(str(datetime.now()) + ' ' + str(avoid.VDATA) + ' ' + str(avoid.obj) + '\n')
    else:
        f.write(str(datetime.now()) + ' ' + str(avoid.VDATA) + ' ' + str(avoid.obj) + ' ' + str(v_final) + ' ' + str(phi_final) + '\n')
    f.flush()  
    if draw == 1:
        plt.pause(0.1)

f.close()
# if len(total_time) > 0:
#     max = max(total_time)
    # print('Max execution time:', max)
if draw == 1:
    plt.show()

