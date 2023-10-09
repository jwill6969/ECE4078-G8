# M4 - Autonomous fruit searching

# basic python packages
from cmath import sqrt
import sys, os
import cv2
import numpy as np
import json
import ast
import argparse
import time
#from Practical03_Support.Obstacle import *
#from Practical03_Support.path_animation import *
#import meshcat.geometry as g
#import meshcat.transformations as tf
#from a_star import AStarPlanner 


import numpy as np
import random
import os
#from rrtc import RRTC
# import SLAM components
# sys.path.insert(0, "{}/slam".format(os.getcwd()))
# from slam.ekf import EKF
# from slam.robot import Robot
# import slam.aruco_detector as aruco
# basic python packages
import sys, os
import cv2
import numpy as np
import json
import ast
import argparse
import time

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco
from slam.RRTC import RRTC
from slam.Obstacle import Circle
from slam.A_star_search import AStarPlanner
from operate import Operate
from TargetPoseEst import merge_estimations,filtering,estimate_pose
# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import Alphabot
import util.measure as measure
from util.utilityFunctions import *


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


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(3):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1


# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# additional improvements:
# you may use different motion model parameters for robot driving on its own or driving while pushing a fruit
# try changing to a fully automatic delivery approach: develop a path-finding algorithm that produces the waypoints
def drive_to_point(waypoint, robot_pose):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    wheel_vel = 10 # tick to move the robot
    x_dist = np.abs(waypoint[0]-robot_pose[0])
    y_dist = np.abs(waypoint[1]-robot_pose[1])
    desired_angle = np.arctan2(y_dist/x_dist)
    turning_angle = desired_angle - robot_pose[2]
    
    if(turning_angle <0):
        turning_angle = np.abs(turning_angle)+180

    # turn towards the waypoint
    turn_time = (turning_angle/360) * np.pi*wheel_vel*scale# replace with your calculation
    print("Turning for {:.2f} seconds".format(turn_time))
    ppi.set_velocity([0, 1], turning_tick=wheel_vel, time=turn_time)
    
    # after turning, drive straight to the waypoint
    distance = np.sqrt((x_dist**2)+(y_dist**2))
    drive_time = distance*wheel_vel*scale # replace with your calculation
    print("Driving for {:.2f} seconds".format(drive_time))
    ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))


def get_robot_pose(robot_pose):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here

    # update the robot pose [x,y,theta]
    robot_pose = [0.0,0.0,0.0] # replace with your calculation
    ####################################################

    return robot_pose

# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints in command line, you can get coordinates by clicking on a map (GUI input), see camera_calibration.py
        # goal = np.array([0.8, 0.4])
        # start = np.array([0, 0])
        # red_apple = Circle(0.8, 0.4, 0.1)
        # green_apple = Circle(1.2,-0.4,0.1)
        # orange = Circle(0, -0.8,0.1)
        # mango = Circle(-0.4, 0.4,0.1)
        # capsicum = Circle(-0.8, 0.8,0.1)
        # fruits = [red_apple,green_apple, orange, mango, capsicum]


        # all_obstacles = [Circle(0.4, 0.4, 0.1), Circle(0.8, 0, 0.1),
        #                 Circle(1.2, 0.4, 0.1), Circle(0.8, 0.8, 0.1), Circle(0, -0.4, 0.1),
        #                 Circle(0, 1.2, 0.1), Circle(0, -0.4, 0.1),Circle(-0.4, -1.2, 0.1),Circle(-1.2, -0.4, 0.1)]

        # for fruit in fruits:
        #     obstacle_fruits =[]
        #     goal = np.array([fruit.center[0]-0.1, fruit.center[1]-0.1])
        #     start = np.array([0, 0])
        #     all_obstacles = [Circle(0.4, 0.4, 0.1), Circle(0.8, 0, 0.1),
        #                     Circle(1.2, 0.4, 0.1), Circle(0.8, 0.8, 0.1), Circle(0, -0.4, 0.1),
        #                     Circle(0, 1.2, 0.1), Circle(0, -0.4, 0.1),Circle(-0.4, -1.2, 0.1),Circle(-1.2, -0.4, 0.1)]
        #     for fruit_obstacle in fruits:
        #         if fruit_obstacle != fruit:
        #             all_obstacles.append(fruit_obstacle)
            
        #     rrtc = RRTC(start=start, goal=goal, width=3.2, height=3.2, obstacle_list=all_obstacles,
        #         expand_dis=0.5, path_resolution=0.2)
        #     rrtc = RRTC(start=start, goal=goal, width=3.2, height=3.2, obstacle_list=all_obstacles,
        #         expand_dis=0.5, path_resolution=0.2)
        #     path = rrtc.planning()
        #     for waypoint in path:
        #         drive_to_point(waypoint,robot_pose)
                
        #     print(path)   
        sx = 0# [m]
        sy = 0  # [m]
        gx = 0.8 # [m]
        gy = 0.4 # [m]
        grid_size = 0.2# [m]
        robot_radius = 0.2 # [m]
    
        # set obstacle positions
        ox, oy = [0.4,0.8,1.2,0.8,0,0,0,-0.4,-1.2], [0.4,0,0.4,0.8,-0.4,1.2,-0.4,-1.2,-0.4]
        for i in range(-12, 12,1):
            ox.append(-1.2)
            oy.append(i/10)
            ox.append(1.2)
            oy.append(i/10)
            ox.append(i/10)
            oy.append(-1.2)
            ox.append(i/10)
            oy.append(1.2)
        
        

        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)
        rx = np.flip(rx)
        ry = np.flip(ry)
        for i in range(len(rx)):
            waypoint = [rx,ry]
            drive_to_point(waypoint,robot_pose)
            robot_pose = get_robot_pose()

        x = input("X coordinate of the waypoint: ")
        try:
            x = float(x)
        except ValueError:
            print("Please enter a number.")
            continue
        y = input("Y coordinate of the waypoint: ")
        try:
            y = float(y)
        except ValueError:
            print("Please enter a number.")
            continue

        # estimate the robot's pose
        robot_pose = get_robot_pose()

        # robot drives to the waypoint
        waypoint = [x,y]
        drive_to_point(waypoint,robot_pose)
        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break