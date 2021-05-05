import numpy as np
import cv2
import matplotlib.pyplot as plt
import json
import math
import time

def velocity_distance(obstacle, r_rob, pos, yaw):
    intervals = []
    r = obstacle[2] + r_rob + safety_margin
    for t in range(0, 6):
        # r = obstacle[2] + r_rob
        if obstacle[3] <= 0.03 and obstacle[4] <= 0.03 and t > 0:
            break
        x = obstacle[0] + obstacle[3]*t
        y = obstacle[1] + obstacle[4]*t
        # print('x_new, y_new =', x, ',', y)
        if y < - (obstacle[2] + r_rob):
            continue
        # r = r + safety_margin
        if x**2 + y**2 - r**2 < 0:
            if t > 0:
                continue
            print('Collided with obstacle!')
            r = math.sqrt(x**2 + y**2 - 1)
        c_min = 2*(x - r)/(x**2 + y**2 - r**2)
        c_max = 2*(x + r)/(x**2 + y**2 - r**2)
        # print('c_min =', c_min)
        # print('c_max =', c_max)
        x_min = (x - r)/(1 - c_min*r)
        y_min = y/(1 - c_min*r)
        x_max = (x + r)/(1 + c_max*r)
        y_max = y/(1 + c_max*r)
        if y_min < 0 or y_max < 0:
            continue
        # print('x_min, y_min =', x_min, ',', y_min)
        # print('x_max, y_max =', x_max, ',', y_max)
        if c_min == 0:
            dc_cmin = y_min
        else:
            dc_cmin = abs(1/c_min*math.atan(y_min/(x_min - 1/c_min)))
        if c_max == 0:
            dc_cmax = y_max
        else:
            dc_cmax = abs(1/c_max*math.atan(y_max/(x_max - 1/c_max)))
        p, c = devide_into_quadrants(x, y, r, x_min, y_min, x_max, y_max)
        if p is not None:
            if c == 0:
                dc_c = p[1]
            else:
                dc_c = abs(1/c*math.atan(p[1]/(p[0] - 1/c)))
            if dc_c == min(dc_cmax, dc_cmin, dc_c):
                intervals.append([c_min, c_max, min(dc_cmin, dc_c)])
                # return [[c_min, c_max, min(dc_cmin, dc_c)]]
            else:
                intervals.append([c_min, c, min(dc_cmin, dc_c)])
                intervals.append([c, c_max, min(dc_cmax, dc_c)])
                # return [c_min, c, min(dc_cmin, dc_c)], [c, c_max, min(dc_cmax, dc_c)]
        else:
            intervals.append([c_min, c_max, min(dc_cmin, dc_cmax)])
            # return [[c_min, c_max, min(dc_cmin, dc_cmax)]]
    intervals.sort(key=takeFirst)
    return intervals

def intersection_solve(center, r, a, b):
    aa = 1 + a**2
    bb = -2*center[0] + 2*a*(b - center[1])
    cc = center[0]**2 + (b - center[1])**2 - r**2
    delta = bb**2 - 4*aa*cc
    x1 = (- bb + math.sqrt(delta))/(2*aa)
    y1 = a*x1 + b
    # print('x1, y1', [x1, y1])
    x2 = (- bb - math.sqrt(delta))/(2*aa)
    y2 = a*x2 + b
    # print('x2, y2', [x2, y2])
    return [x1, y1], [x2, y2]

def devide_into_quadrants(x, y, r, x_min, y_min, x_max, y_max):
    if x == 0:
        p1 = [0, y - r]
        p2 = [0, y + r]
        p3 = [r, y]
        p4 = [- r, y]
    elif y == 0:
        p1 = [x + r, 0]
        p2 = [x - r, 0]
        p3 = [x, r]
        p4 = [x, - r]
    else:
        a = (y - 0)/(x - 0)
        b = (-(y - 0)*x + (x - 0)*y)/(x - 0)
        p1, p2 = intersection_solve([x, y], r, a, b)
        a = - (x - 0)/(y - 0)
        b = ((x - 0)*x + (y - 0)*y)/(y - 0)
        p3, p4 = intersection_solve([x, y], r, a, b)
    points = [p1, p2, p3, p4]
    # print('points', points)
    for point in points:
        angle1 = math.atan2(y_min - y, x_min - x)
        angle2 = math.atan2(point[1] - y, point[0] - x)
        angle3 = math.atan2(y_max - y, x_max - x)
        if angle1 < 0:
            angle1 += 2*math.pi
        if angle2 < 0:
            angle2 += 2*math.pi
        if angle3 < 0:
            angle3 += 2*math.pi
        angle_max = max(angle1, angle3)
        angle_min = min(angle1, angle3)
        if 0 < angle_max - angle_min < math.pi and angle_min < angle2 < angle_max:
            c = 2*point[0]/(point[0]**2 + point[1]**2)
            return point, c
        elif math.pi < angle_max - angle_min and angle_max < angle2 < angle_min + 2*math.pi:
            c = 2*point[0]/(point[0]**2 + point[1]**2)
            return point, c
    return None, None

def min_union(old_intervals, L):
    intervals = [[- math.inf, math.inf, L]]
    for new in old_intervals:
        c_min, c_max, d = new[0:3]
        # print('new', new)
        temp = np.copy(intervals).tolist()
        # print('temp', temp)
        for interval in temp:
            # print('interval', interval)
            c1, c2, d12 = interval[0:3]
            # print('c1', c1, 'c2', c2, 'c_min', c_min, 'c_max', c_max)
            if c_min <= c1 and c2 <= c_max:
                # print('1')
                intervals.remove(interval)
                d12 = min(d, d12)
                intervals.append([c1, c2, d12])
            elif c1 <= c_min and c_max <= c2 and d < d12:
                # print('2')
                intervals.remove(interval)
                if c1 < c_min:
                    intervals.append([c1, c_min, d12])
                intervals.append([c_min, c_max, d])
                if c_max < c2:
                    intervals.append([c_max, c2, d12])
            elif c1 <= c_min < c2 <= c_max and d < d12:
                # print('3')
                intervals.remove(interval)
                if c1 < c_min:
                    intervals.append([c1, c_min, d12])
                if c_min < c2:
                    intervals.append([c_min, c2, d])
            elif c_min <= c1 < c_max <= c2 and d < d12:
                # print('4')
                intervals.remove(interval)
                if c1 < c_max:
                    intervals.append([c1, c_max, d])
                if c_max < c2:
                    intervals.append([c_max, c2, d12])
            # print('intervals', intervals)
    intervals.sort(key=takeFirst)
    i = 0
    new = []
    while i < len(intervals):
        if i == len(intervals) - 1:
            new.append(intervals[-1])
            break
        else:
            j = i + 1
            while j < len(intervals):
                if intervals[j - 1][2] == intervals[j][2] and intervals[j - 1][1] == intervals[j][0]:
                    j = j + 1
                else:
                    interval = [intervals[i][0], intervals[j - 1][1], intervals[i][2]]
                    new.append(interval)
                    i = j
                    break
    return new

def objective_function(pos, yaw, i_start, i_goal, intervals, old_rv):
    ft = []
    for i in range(rv_points * 2 + 1):
        tv = tv_max
        rv = rv_max*(i - rv_points)/rv_points
        c = rv/tv
        for interval in intervals:
            if interval[0] < c < interval[1]:
                d = interval[2]
        d_lane = lane(pos, yaw, i_start, i_goal, x_path, y_path, c)
        # print('d', d, 'd_lane', d_lane)
        d = min(d, d_lane)
        # if d_lane < d:
        #     print('Exceed lane width!')
        phi_goal = math.atan2(y_path[i_goal] - pos[1], x_path[i_goal] - pos[0]) - yaw
        # print('phi_goal', phi_goal)
        if tv > d/T_imp:
            tv = d/T_imp
            rv = c*tv
        # print('tv', tv, 'rv', rv, 'd', d, 'c', c)
        # print(alpha1*tv/tv_max, alpha2*d/L, alpha3*(1 - abs(phi_goal + rv)/math.pi))
        f = alpha1*tv/tv_max + alpha2*d/L + alpha3*(1 - abs(phi_goal + rv)/math.pi)
        # print('f', f)
        ft.append([f, tv, rv])
    ft.sort(key=takeFirst, reverse = True)
    # print('ft', ft)
    if ft[0][0] < 0.8:
        return 0, 0
    tv_ftmax, rv_ftmax = ft[0][1], ft[0][2]
    if rv_ftmax * old_rv < 0:
        for i in range(1, len(ft)):
            if ft[i][2] * old_rv > 0 and abs(ft[0][0] - ft[i][0]) < 0.01:
                tv_ftmax, rv_ftmax = ft[i][1], ft[i][2]
    return tv_ftmax, rv_ftmax

def dv_lane(c, x1, y1, x2, y2):
    if c == 0 and x2 == x1:
        # print('1')
        dv = math.inf
    elif x2 == x1:
        R = 1/c
        if R**2 - (x1 - R)**2 < 0:
            # print('2')
            dv = math.inf
        else:
            # print('3')
            root_x = x1
            root_y = math.sqrt(R**2 - (x1 - R)**2)
            dv = abs(1/c*math.atan(root_y/(root_x - 1/c)))
    elif c == 0:
        a = (y2 - y1)/(x2 - x1)
        b = - a*x1 + y1
        if b < 0:
            # print('4')
            dv = math.inf
        else:
            # print('5')
            dv = b
    else:
        a = (y2 - y1)/(x2 - x1)
        b = - (y2 - y1)/(x2 - x1)*x1 + y1
        R = 1/c
        delta = R**2 - 2*a*b*R - b**2
        if delta < 0:
            # print('6')
            dv = math.inf
        elif delta >= 0:
            # print('7')
            root_x1 = (R-a*b+math.sqrt(delta))/(1+a**2)
            root_x2 = (R-a*b-math.sqrt(delta))/(1+a**2)
            root_y1 = a*root_x1 + b
            root_y2 = a*root_x2 + b
            if root_y1 < 0:
                dv1 = math.inf
            else:
                dv1 = abs(1/c*math.atan(root_y1/(root_x1 - 1/c)))
            if root_y2 < 0:
                dv2 = math.inf
            else:
                dv2 = abs(1/c*math.atan(root_y2/(root_x2 - 1/c)))
            dv = min(dv1, dv2)
    return dv

def lane(pos, yaw, i1, i2, x_path, y_path, c):
    dist = []
    # print('i1', i1, 'i2', i2)
    for j in range(i1, i2 + 1):
        dist.append([math.sqrt((pos[0] - x_path[j])**2 + (pos[1] - y_path[j])**2), j])
    dist = np.array(dist)
    i_min = int(dist[np.where(dist == dist.min(axis = 0))[0][1], 1])
    # print('i_min', i_min)
    if i_min == len(x_path) - 1:
        alpha = math.atan2((x_path[i_min]-x_path[i_min-1]), -(y_path[i_min]-y_path[i_min-1]))
    else:
        alpha = math.atan2((x_path[i_min+1]-x_path[i_min]), -(y_path[i_min+1]-y_path[i_min]))

    x1 = x_path[i_min + 1]-lane_width*math.cos(alpha)
    y1 = y_path[i_min + 1]-lane_width*math.sin(alpha)
    x2 = x_path[i_min]-lane_width*math.cos(alpha)
    y2 = y_path[i_min]-lane_width*math.sin(alpha)
    # x = np.linspace(x2, x1, 10)
    # ax.plot(x, (x-x2)/(x1-x2)*(y1-y2)+y2);
    x3 = (x1 - pos[0])*math.sin(yaw) - (y1 - pos[1])*math.cos(yaw)
    y3 = (x1 - pos[0])*math.cos(yaw) + (y1 - pos[1])*math.sin(yaw)
    x4 = (x2 - pos[0])*math.sin(yaw) - (y2 - pos[1])*math.cos(yaw)
    y4 = (x2 - pos[0])*math.cos(yaw) + (y2 - pos[1])*math.sin(yaw)
    d1 = dv_lane(c, x3, y3, x4, y4)

    x1 = x_path[i_min + 1]+lane_width*math.cos(alpha)
    y1 = y_path[i_min + 1]+lane_width*math.sin(alpha)
    x2 = x_path[i_min]+lane_width*math.cos(alpha)
    y2 = y_path[i_min]+lane_width*math.sin(alpha)
    # x = np.linspace(x2, x1, 10)
    # ax.plot(x, (x-x2)/(x1-x2)*(y1-y2)+y2);
    x3 = (x1 - pos[0])*math.sin(yaw) - (y1 - pos[1])*math.cos(yaw)
    y3 = (x1 - pos[0])*math.cos(yaw) + (y1 - pos[1])*math.sin(yaw)
    x4 = (x2 - pos[0])*math.sin(yaw) - (y2 - pos[1])*math.cos(yaw)
    y4 = (x2 - pos[0])*math.cos(yaw) + (y2 - pos[1])*math.sin(yaw)
    d2 = dv_lane(c, x3, y3, x4, y4)
    return min(d1, d2)

def takeFirst(elem):
    return elem[0]

draw = 0
dist_goal = 1 # (m) distance between robot_pos and final_goal where the robot will stop
r_rob = 0.2 # (m) radius of the robot
tv_max = 0.3 # (m/s) maximum translational velocity
rv_max = tv_max*2/0.388 # (rad/s) maximum rotaional velocity, 0.388(m): distance between two wheels
T_imp = 2/tv_max # (s) robot will be able to travel at least T_imp seconds before hitting an obstacle
rv_points = 30 # consider rv_points rotational velocity values in range [-rv_max, rv_max]
alpha1 = 0.1 # weights in the objective function
alpha2 = 0.3 # weights in the objective function
alpha3 = 0.6 # weights in the objective function
safety_margin = 0.2 # (m) Increase size of obstacles by a safety margin
path_cut = 10 # only consider path in range [i_curr, i_curr + path_cut)
lane_width = 1 # (m) the width of the lane
temp_goal = 1  # the offset added to the max index of the path
                # that an obstacle collides with to determine a temporary goal
temp_nextgoal = 1 # if the robot is closer to path[j] than temporary goal,
                  # the new temporary goal is path[j + temp_nextgoal]

json_input = open('./map_out.txt')
input_data = json.load(json_input)
x_path = input_data['x']
y_path = input_data['y']

L = tv_max * T_imp
old_rvfinal = 0
detected_flag = 0
i_goal = None
total_time = []
p_rob = []
p_obs = []

while not rospy.is_shutdown():
    # //////////////////Update/////////////////////////
    detected = avoid.pos
    # print 'detected', detected
    robot_pos = [x_path[0], y_path[0]]
    i_curr = 0
    robot_yaw = math.atan2((y_path[1] - y_path[0]), (x_path[1] - x_path[0]))
    final_goal = [x_path[-1], y_path[-1]]
    if final_goal == []:
        final_goal = robot_pos
        i_finalgoal = i_curr
    else:
        i_finalgoal =
        # path_dist = []
        # for j in range(len(x_path)):
        #     dist = math.sqrt((x_path[j] - final_goal[0])**2 + (y_path[j] - final_goal[1])**2)
        #     path_dist.append([dist, j])
        # path_dist = np.array(path_dist)
        # i_finalgoal = int(path_dist[np.where(path_dist == path_dist.min(axis = 0))[0][1], 1])
    step = 1 if i_curr<=i_finalgoal else -1
    # //////////////////Draw///////////////////////////
    if draw == 1:
        if p_rob == [] and p_obs == []:
            fig, ax = plt.subplots(1)
            plt.grid(True)
            plt.axis("equal")
            plt.plot(input_data['x'], input_data['y'], "-y")
            p_goal = plt.plot(final_goal[0], final_goal[1], "ob")
            for obstacle in detected:
                p_obs.append(plt.plot(obstacle[0], obstacle[1], "og"))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, color='g', alpha=0.5, edgecolor='None'))
                # ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob + safety_margin, color='y', alpha=0.5, edgecolor='None'))
        else:
            p_rob[0].remove()
            for i in range(len(p_obs)):
                p_obs[i][0].remove()
            p_obs = []
            for obstacle in detected:
                p_obs.append(plt.plot(obstacle[0], obstacle[1], "og"))
                ax.add_patch(plt.Circle((pre_obs[0], pre_obs[1]), pre_obs[2] + r_rob, color='w', alpha=1, edgecolor='None'))
                ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), obstacle[2] + r_rob, color='g', alpha=0.5, edgecolor='None'))
        p_rob = plt.plot(robot_pos[0], robot_pos[1], "or")
        # x = np.linspace(robot_pos[0], robot_pos[0] + tv_final*np.cos(robot_yaw), 100)
        # ax.plot(x, math.tan(robot_yaw)*(x - robot_pos[0]) + robot_pos[1])
    # /////////////////////////////////////////////////
    if np.sqrt((robot_pos[0] - final_goal[0])**2 + (robot_pos[1] - final_goal[1])**2) > dist_goal:
        # start = time.time()
        print 'i_curr', i_curr
        print 'i_finalgoal', i_finalgoal
        if len(detected) > 0 and detected_flag == 0:
            for j in range(i_curr, min(i_curr+path_cut, i_finalgoal) if step==1 else max(i_curr-path_cut, i_finalgoal), step):
                if j == i_curr:
                    s = 0
                else:
                    s = s + math.sqrt((x_path[j] - x_path[j - step]) ** 2 + (y_path[j] - y_path[j - step]) ** 2)
                t = s/tv_max
                for obstacle in detected:
                    distance = math.sqrt((x_path[j] - (obstacle[0] + obstacle[3]*t)) ** 2
                                    + (y_path[j] - (obstacle[1] + obstacle[4]*t)) ** 2)
                    if distance <= obstacle[2] + r_rob + safety_margin:
                        detected_flag = 1
                        i_start = i_curr
                        # count = 1
                        break
                if detected_flag == 1:
                    break
        print('detected_flag', detected_flag)
        if detected_flag == 1:
            collision = []
            for obstacle in detected:
                for j in range(i_curr, i_finalgoal, step):
                    dist1 = math.sqrt((x_path[j] - (obstacle[0] + obstacle[3]*t))**2 + (y_path[j] - (obstacle[1] + obstacle[4]*t))**2)
                    dist2 = math.sqrt((x_path[j + step] - (obstacle[0] + obstacle[3]*t))**2 + (y_path[j + step] - (obstacle[1] + obstacle[4]*t))**2)
                    if dist1 <= obstacle[2] + r_rob + safety_margin and dist2 >= obstacle[2] + r_rob + safety_margin:
                        collision.append(j + step)
                        break
            if len(collision) > 0:
                i_goal = min(max(collision)+temp_goal, i_finalgoal) if step==1 else max(min(collision)-temp_goal, i_finalgoal)
            for j in range(i_goal + step, i_finalgoal, step):
                dist1 = math.sqrt((robot_pos[0] - x_path[i_goal])**2 + (robot_pos[1] - y_path[i_goal])**2)
                dist2 = math.sqrt((robot_pos[0] - x_path[j])**2 + (robot_pos[1] - y_path[j])**2)
                dist3 = math.sqrt((robot_pos[0] - x_path[j + step])**2 + (robot_pos[1] - y_path[j + step])**2)
                if dist2 <= dist1 and dist2 <= dist3:
                    i_goal = min(j+temp_nextgoal, i_finalgoal) if step==1 else max(j-temp_nextgoal, i_finalgoal)
                    break
            goal = [x_path[i_goal], y_path[i_goal]]
            if draw == 1:
                p_goal[0].remove()
                p_goal = plt.plot(goal[0], goal[1], "ob")
            if np.sqrt((robot_pos[0] - goal[0])**2 + (robot_pos[1] - goal[1])**2) > dist_goal:
                # print('count:', count)
                # print('robot_pos =', robot_pos)
                # print('robot_yaw =', robot_yaw)
                # print('detected =', detected)
                obstacle_intervals = []
                obstacles_robot = []
                for obstacle in detected:
                    dv = velocity_distance(obstacle, r_rob, robot_pos, robot_yaw)
                    if dv is not None:
                        for i in range(len(dv)):
                            obstacle_intervals.append(dv[i])
                # print('obstacle_intervals', obstacle_intervals)
                new_intervals = min_union(obstacle_intervals, L)
                # print('new_intervals', new_intervals)
                tv_final, rv_final = objective_function(robot_pos,
                        robot_yaw, i_start, i_goal, new_intervals, old_rvfinal)
                print('tv_final', tv_final, 'rv_final', rv_final)
                old_rvfinal = rv_final
                # count = count + 1
            else:
                detected_flag = 0
                if draw == 1:
                    p_goal[0].remove()
                    p_goal = plt.plot(final_goal[0], final_goal[1], "ob")

        # end = time.time()
        # print('Execution time:', end - start)
        # if detected_flag == 1:
        #     total_time.append(end - start)
        if draw == 1:
            plt.pause(0.5)

# print('The end!')
# if len(total_time) > 0:
#     print('Max execution time:', max(total_time))
if draw == 1:
    plt.show()
