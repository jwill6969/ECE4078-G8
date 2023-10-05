# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import cv2
import numpy as np
import json
import ast
import argparse
import time
from asyncio.windows_events import NULL
from cmath import nan
from pathlib import Path
# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco
from slam.RRTC import RRTC
from slam.Obstacle import Circle
from slam.A_star_search import AStarPlanner
from operate import Operate
from TargetPoseEst import *
# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import Alphabot
import util.measure as measure
from util.utilityFunctions import *

def drive_to_point(waypoint):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')

    tick = 20
    linear_vel = 20
    angular_vel = 20 
    robot_pose = get_robot_pose(operate)
    print("robot_pose",robot_pose)   
    x_diff = waypoint[0] - robot_pose[0]
    
    y_diff = waypoint[1] - robot_pose[1]
    pos_diff = np.hypot(x_diff, y_diff)

    turn_time,curr_wheel_vel = turn_to_point(waypoint, robot_pose, angular_vel)

    if np.abs(turn_time) >= 0.01:
        if turn_time < 0:
            command = [0, -0.8]
        if turn_time > 0:
            command = [0, 0.8]
    else:
        command = [0, 0]
    print("turn time",turn_time)

    lv, rv = operate.pibot.set_velocity(command,tick=tick, turning_tick=curr_wheel_vel, time=np.abs(turn_time[0]))
    # lv -= command[0] * linear_tick
    # rv -= command[0] * linear_tick
    turn_drive_meas = measure.Drive(lv, rv, np.abs(turn_time))
    time.sleep(0.3)
    slam_estimation(turn_drive_meas)

    drive_time = pos_diff/(scale*linear_vel)
    if np.abs(drive_time) > 0.01:
        command = [1, 0]
    else:
        command = [0, 0]

    print("drive time",drive_time)
    lv, rv = operate.pibot.set_velocity(command, tick=tick, time=drive_time[0])
    drive_meas = measure.Drive(lv, rv, np.abs(drive_time))
    time.sleep(0.3)
    slam_estimation(drive_meas)
    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))

def slam_estimation(drive_meas=None):
    if not operate or not ppi:
        return
    elif drive_meas == None:
        lv,rv = ppi.set_velocity([0,0],tick=20, turning_tick=10, time=0)
        drive_meas = measure.Drive(lv, rv, 0)
        operate.take_pic()
        operate.update_slam(drive_meas)
        operate.ekf.robot.state[2] = clamp_angle(operate.ekf.robot.state[2])
        # operate.draw(canvas)
        # pygame.display.update()
    elif drive_meas != None:
        operate.take_pic()
        # landmarks,_ = operate.aruco_det.detect_marker_positions(operate.img)
        operate.update_slam(drive_meas)
        operate.ekf.robot.state[2] = clamp_angle(operate.ekf.robot.state[2])
        # operate.draw(canvas)
        # pygame.display.update()
    else:
        return
    

def localise():
    angular_wheel_vel = 20
    turning_tick = 5
    tick = 10
    #slam_estimation()
    for pos in aruco_true_pos:
        robot_pose = get_robot_pose(operate)
        turn_time,curr_wheel_vel = turn_to_point(pos[:2], robot_pose, angular_wheel_vel)
        
        if np.abs(turn_time) >= 0.01:
            if turn_time < 0:
                command = [0, -0.75]
            if turn_time > 0:
                command = [0, 0.75]
            # if np.abs(turn_time) > 1:
            #     command[1] = command[1]*1.15
            # else:
            #     command[1] = command[1]*1.5
        else:
            command = [0, 0]
        
        lv, rv = operate.pibot.set_velocity(command,tick=tick, turning_tick=curr_wheel_vel, time=np.abs(turn_time[0]))
        
        drive_meas = measure.Drive(lv, rv, np.abs(turn_time))
        time.sleep(0.3)
        slam_estimation(drive_meas)
    # turn_time = turn_to_point([0,0], robot_pose, angular_wheel_vel)
    # if np.abs(turn_time) >= 0.01:
    #     if turn_time < 0:
    #         command = [0, -0.75]
    #     if turn_time > 0:
    #         command = [0, 0.75]
    #     else:
    #         command = [0, 0]
    # lv, rv = operate.pibot.set_velocity(command, 
    #                             turning_tick = angular_wheel_vel, 
    #                             time=np.abs(turn_time))
    # drive_meas = measure.Drive(lv, rv, np.abs(turn_time))
    print(get_robot_pose(operate))
    return


def PATH(waypoint):
    # PATH should go to all the waypoints in the list
    fruit_list, fruit_true_pos, aruco_true_pos = read_true_map("M4_true_map.txt")
    
    start = [get_robot_pose(operate)[0][0],get_robot_pose(operate)[1][0]]
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
    # rrtc = RRTC(start=start, goal=goal, width=3.2, height=3.2, obstacle_list=obstacles, expand_dis=0.2, path_resolution=0.1)  
    # path = rrtc.planning()
    # if path is None:
    #     print("no path")
    # else:
    #     path = path[1:-1]
    #     print(path)
    resolution = 0.2
    robot_radius = 0.09
    planner = AStarPlanner(aruco_true_pos, fruit_true_pos, resolution, robot_radius)    
    rx,ry = planner.planning(start[0],start[1],goal[0],goal[1])
    path = []
    for i in range(len(rx)):
        path.append([rx[i],ry[i]])
    path = path[::-1]
    path = path[1:-1]
    print("path",path)
        # Navigate through the path
    for i in range((len(path))):
        print("waypoint",path[i]) 
        drive_to_point(path[i])
        # if (np.abs(path[i][0])<0.2 and np.abs(path[i][1])<0.2):
        #     localise()
        print(get_robot_pose(operate))
    localise()
    print("FOUND FRUIT")
    time.sleep(3)
    #drive_to_point(waypoint)
        
    return True
    
# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='192.168.137.27')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='network/models/yolov8_model.pt')
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)
    operate = Operate(args)
    operate.ekf = operate.init_ekf(args.calib_dir, args.ip)
    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map("M4_true_map.txt")
    search_list = read_search_list()
    waypoints = print_target_fruits_pos(search_list, fruits_list, fruits_true_pos,output=True)

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]
    addAruco(operate,aruco_true_pos)
    print("waypoints",waypoints)
    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints in command line, you can get coordinates by clicking on a map (GUI input), see camera_calibration.py
        
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
        # checker = drive_to_point([x,y])
        # localise()
        # estimate the robot's pose
        

        # robot drives to the waypoint
        for i in range(len(waypoints)):
            arrive = False
            PATH(waypoints[i])
            localise()
            while arrive is False:
                if (np.sqrt((get_robot_pose(operate)[0]-waypoints[i][0])**2+(get_robot_pose(operate)[1]-waypoints[i][1])**2)>0.5):
                    PATH([waypoints[i][0]-0.1,waypoints[i][1]-0.1])
                    localise()
                else:
                    arrive = True
                    print("arrived")
                

            
        #drive_to_point(waypoint)
    
        # exit
        ppi.set_velocity([0, 0])
        # uInput = input("Add a new waypoint? [Y/N]")
        # if uInput == 'N':
        break