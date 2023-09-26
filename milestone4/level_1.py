# M4 - Autonomous fruit searching

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
from operate import Operate
# import utility functions
sys.path.insert(0, "util")
from pibot import Alphabot
import measure as measure


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
    
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    wheel_vel = 10 # tick to move the robot
    initial = robot_pose #x,y,theta
    x_init = initial[0]
    y_init = initial[1]

    goal = waypoint
    x_final = goal[0]
    y_final = goal[1]
    

    dist = np.sqrt((x_final-x_init)**2 + (y_final-y_init)**2)
    
    acos = np.arccos(x_final-x_init/dist) #in radian
    asin = np.arcsin(y_final-y_init/dist)
    
    #find angle from quadrant
    if(acos>=0 and asin>=0):
        angle = acos
    elif(acos<0 and asin>=0):
        angle = acos
    elif(acos<=0 and asin<=0):
        angle = -acos
    elif(acos>=0 and asin<=0):
        angle = asin
    
        
    theta =np.abs(angle) - initial[2]

    if(theta < 0):
        #C.W
        dir = -1
    else:
        #C.C.W
        dir = 1
    if(np.abs(theta)>np.pi):
        theta = np.abs(theta) - np.pi
    else:
        theta = theta

    velocity = wheel_vel * scale
    
    rot_dist = baseline/2 * theta
    # turn towards the waypoint
    turn_time = rot_dist/velocity # replace with your calculation
    print("Turning for {:.2f} seconds".format(turn_time))
    ppi.set_velocity([0, dir], turning_tick=wheel_vel, time=turn_time)
    
    # after turning, drive straight to the waypoint
    drive_time = dist/velocity # replace with your calculation
    print("Driving for {:.2f} seconds".format(drive_time))
    ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
def slam_estimation(drive_meas=None):
    return
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
def get_robot_pose(operate, raw_meas = None):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here
    operate.take_pic()
    
    measurements, img_marked = operate.aruco_det.detect_marker_positions(img=operate.img)
    if(raw_meas!=None):
        if(raw_meas.dt != None):
            operate.ekf.predict = EKF.predict(raw_meas)
    operate.ekf.update(measurements)
    # update the robot pose [x,y,theta]
    
    ####################################################

    return operate.ekf.robot.state
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
def PATH(waypoint):
    # PATH should go to all the waypoints in the list
    fruit_list, fruit_true_pos, aruco_true_pos = read_true_map("M4_true_map.txt")
    start = get_robot_pose(operate)[0:2]
    goal = np.array(waypoint)
    radius = 0.075
    tolerance = 0.3
    aruco_obstacles = []
    # Adding all arucos to obstacle list
    for i in range(len(aruco_true_pos)):
                x = aruco_true_pos[i][0]
                y = aruco_true_pos[i][1]
                aruco_obstacles.append(Circle(x,y,radius))
    obstacles = aruco_obstacles
    for i in range(len(fruit_list)):
        if (goal[0] != fruit_true_pos[i][0] and goal[1] != fruit_true_pos[i][1]):
                x = fruit_true_pos[i][0]
                y = fruit_true_pos[i][1]
                obstacles.append(Circle(x,y,radius))
                
        print(obstacles)    

    # iterating through all the waypoints
    # for curr_waypoint in range(len(fruit_list)):
    #     start = get_robot_pose()
    #     goal = fruit_true_pos[curr_waypoint]
    #     obstacles = aruco_obstacles

    #     # Adding non waypoint fruits to obstacle list
    #     for i in range(len(fruit_list)): 
    #         if (i != curr_waypoint):
    #             x = fruit_true_pos[i][0]
    #             y = fruit_true_pos[i][1]
    #             obstacles.append(Circle(x,y,radius))
        
        
        rrtc = RRTC(start=start, goal=goal, width=3.2, height=3.2, obstacle_list=obstacles, expand_dis=0.1, path_resolution=0.01)
        
        path = rrtc.planning()
        path = path[::,-1]
        path = path[1:-1]

        # Navigate through the path
        for pt in path: drive_to_point(pt,get_robot_pose(operate))

        dist = np.sqrt((goal[0]-get_robot_pose(operate)[0])^2 + (goal[1]-get_robot_pose(operate)[1])^2)
        while (dist > tolerance):
            drive_to_point(pt,get_robot_pose(operate))
            dist = np.sqrt((goal[0]-get_robot_pose(operate)[0])^2 + (goal[1]-get_robot_pose(operate)[1])^2)
        #time.sleep(3)
        
    return True
    
def addAruco(operate,aruco_true_pos):
    operate.ekf.taglist = [i for i in range(1, 11)]
    operate.ekf.markers = aruco_true_pos.T


    init_cov = 1e-3
    operate.ekf.P = np.zeros((3, 3))
    for i in range(len(operate.ekf.taglist)):
        operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((2, operate.ekf.P.shape[1]))), axis=0)
        operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((operate.ekf.P.shape[0], 2))), axis=1)
        operate.ekf.P[-2, -2] = np.power(init_cov,2)
        operate.ekf.P[-1, -1] = np.power(init_cov,2)

    for i in range(5):
        operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((2, operate.ekf.P.shape[1]))), axis=0)
        operate.ekf.P = np.concatenate((operate.ekf.P, np.zeros((operate.ekf.P.shape[0], 2))), axis=1)
        operate.ekf.P[-2, -2] = np.power(init_cov,2)
        operate.ekf.P[-1, -1] = np.power(init_cov,2)
    return True
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='network/models/yolov8_model.pt')
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)
    operate = Operate(args)
    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    addAruco(operate,aruco_true_pos)
    search_list = read_search_list()
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]
    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints in command line, you can get coordinates by clicking on a map (GUI input), see camera_calibration.py
        x,y = 0.8,0.4
        # x = input("X coordinate of the waypoint: ")
        # try:
        #     x = float(x)
        # except ValueError:
        #     print("Please enter a number.")
        #     continue
        # y = input("Y coordinate of the waypoint: ")
        # try:
        #     y = float(y)
        # except ValueError:
        #     print("Please enter a number.")
        #     continue

        # estimate the robot's pose
        

        # robot drives to the waypoint
        waypoint = np.array([x,y])
        checker = PATH(waypoint)
        if (checker):
            print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint,robot_pose))
        else:
            print("shit doesnt work")

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break