import numpy as np
import json
import ast
import math
import sys
import os
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import Alphabot
import util.measure as measure
from util.utilityFunctions import *

import ast
import numpy as np
import json
import matplotlib.pyplot as plt
def printCompare(aligned,unaligned):
    ax = plt.gca()
    ax.scatter(aligned[0][:], aligned[1][:], marker='o', color='C0', s=100)
    ax.scatter(unaligned[0][:], unaligned[1][:], marker='x', color='C1', s=100)
    for i in range(1,11):
        ax.text(aligned[0][i-1]+0.05, aligned[1][i-1]+0.05, i, color='C0', size=12)
        ax.text(unaligned[0][i-1]+0.05, unaligned[1][i-1]+0.05, i, color='C1', size=12)
    plt.title('Arena')
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_xticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    ax.set_yticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    plt.legend(['aligned','unaligned'])
    plt.grid()
    plt.show()

def convert_shape(points):
    final = [[],[]]
    for i in range(len(points)):
        x_cord = points[i][0][0]
        y_cord = points[i][1][0]
        final[0].append(x_cord)
        final[1].append(y_cord)
    return final

def calculate_translation(true_pose, robot_pose):
    translation = np.zeros(2)
    translation[0] = true_pose[0][0] - robot_pose[0][0]
    translation[1] = true_pose[1][0] - robot_pose[1][0]
    return translation

def translate_coordinates(points, translation):
    # Iterate through the list of coordinates and apply the translation
    translated = [[],[]]
    for i in range(len(points[0])):
        translated_x = points[0][i] + translation[0]
        translated_y = points[1][i] + translation[1]
        translated[0].append(translated_x)
        translated[1].append(translated_y)
    return translated

def dist_between_points(point1,point2):
    return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def convertArrayToMap(array):
    map_dict = {}
    for i in range(len(array[0])):
        map_dict[f'aruco{i+1}_0'] = {'x': array[0][i],'y': array[1][i]}
    return map_dict


def addFruitToDict(fruit_dict,coords,label):
    true_dict = {
            'red apple': 1,
            'green apple': 1,
            'mango': 1,
            'orange': 1,
            'capsicum': 1
    }
    for est in fruit_dict:
        if est.__contains__("red apple"):
            true_dict['red apple'] += 1
        elif est.__contains__("green apple"):
            true_dict['green apple'] += 1
        elif est.__contains__("mango"):
            true_dict['mango'] += 1
        elif est.__contains__("capsicum"):
            true_dict['capsicum'] += 1
        elif est.__contains__("orange"):
            true_dict['orange'] += 1
    
    fruit_dict[f'{label}_{true_dict[label]-1}'] = {'x': coords[0],'y': coords[1]}
    return fruit_dict




def clamp_angle(rad_angle=0, min_value=-np.pi, max_value=np.pi):
        """
        Restrict angle to the range [min, max]
        :param rad_angle: angle in radians
        :param min_value: min angle value
        :param max_value: max angle value
        """

        if min_value > 0:
            min_value *= -1

        angle = (rad_angle + max_value) % (2 * np.pi) + min_value

        return angle

def get_angle_robot_to_goal(robot_state=np.zeros(3), goal=np.zeros(2)):
        """
        Obtain from exercise/practical
        Compute angle to the goal relative to the heading of the robot.
        Angle is restricted to the [-pi, pi] interval
        :param robot_state: 3D vector (x, y, theta) representing the current state of the robot
        :param goal: 3D Cartesian coordinates of goal location
        """
        if goal.shape[0] < 3:
            goal = np.hstack((goal, np.array([0])))

        x_goal, y_goal,_ = goal
        x, y, theta = robot_state
        x_diff = x_goal - x
        y_diff = y_goal - y

        alpha = clamp_angle(np.arctan2(y_diff, x_diff) - theta)

        return alpha
def turn_to_point(waypoint, robot_pose, wheel_vel):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    wheel_vel = wheel_vel # tick to move the robot
    angle_diff = get_angle_robot_to_goal(np.asarray(robot_pose), np.asarray(waypoint))
    if np.abs(angle_diff)<np.pi/8:
        cur_wheel_speed=wheel_vel*2
    else:
        cur_wheel_speed=wheel_vel
    turn_time = (angle_diff)/(2*wheel_vel*scale/baseline) # replace with your calculation
    #print("Turning for {:.2f} seconds".format(turn_time))

    return turn_time,cur_wheel_speed

def get_robot_pose(operate):
    operate.ekf.robot.state[2] = clamp_angle(operate.ekf.robot.state[2])
    return operate.ekf.robot.state


def addAruco(operate,aruco_true_pos):
    if operate.ekf_on is False:
        operate.ekf_on = True
    # operate.ekf.taglist = [i for i in range(1, 11)]
    # operate.ekf.markers = aruco_true_pos.T
    # init_cov = 1e-3
    # operate.ekf.P = np.zeros((3, 3))
    # for i in range(len(operate.ekf.taglist)):
    #     operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((2, operate.ekf.P.shape[1]))), axis=0)
    #     operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((operate.ekf.P.shape[0], 2))), axis=1)
    #     operate.ekf.P[-2, -2] = np.power(init_cov,2)
    #     operate.ekf.P[-1, -1] = np.power(init_cov,2)
    aruco_measurement = []
    idi = 1
    # importing the map into the slam
    for lm in aruco_true_pos:
        lm_bff2d = np.array([np.array([lm[0]]), np.array([lm[1]])])
        new_marker = measure.Marker(lm_bff2d, idi)
        aruco_measurement.append(new_marker)
        idi = idi + 1
    #operate.ekf.taglist = [i for i in range(1, 11)]
    operate.ekf.add_landmarks(aruco_measurement)
    operate.ekf.update(aruco_measurement)
    
    
    return True

def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 3 types of target fruit to search

    @param fname: filename of the map
    @return:
        1) list of target fruits, e.g. ['redapple', 'greenapple', 'orange']
        2) locations of the target fruits, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as f:
        try:
            gt_dict = json.load(f)                   
        except ValueError as e:
            with open(fname, 'r') as f:
                gt_dict = ast.literal_eval(f.readline())   
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5])
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('search_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos,output = False):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    waypoints = []
    n_fruit = 1
    for fruit in search_list:
        for i in range(len(fruit_list)):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
                if output is True: waypoints.append([fruit_true_pos[i][0],fruit_true_pos[i][1]])

        n_fruit += 1
    if output is True:
        return waypoints