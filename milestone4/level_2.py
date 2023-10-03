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
from slam.A_star_search import AStarPlanner
from operate import Operate
# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import Alphabot
import util.measure as measure
from util.utilityFunctions import *

# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# additional improvements:
# you may use different motion model parameters for robot driving on its own or driving while pushing a fruit
# try changing to a fully automatic delivery approach: develop a path-finding algorithm that produces the waypoints
def drive_to_point(waypoint):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point
    linear_tick = 10
    angular_tick = 5
    linear_vel = 20
    angular_vel = 10 # tick to move the robot
    robot_pose = get_robot_pose(operate)
    print("robot_pose",robot_pose)   
    x_diff = waypoint[0] - robot_pose[0]
    
    y_diff = waypoint[1] - robot_pose[1]
    pos_diff = np.hypot(x_diff, y_diff)
    # wrap robot orientation to (-pi, pi]
    robot_orient = (robot_pose[2]) % (2*np.pi)
    if robot_orient > np.pi:
        robot_orient -= 2*np.pi
    
    # compute min turning angle to waypoint
    turn_diff = np.arctan2(y_diff, x_diff) - robot_orient
    if turn_diff > np.pi:
        turn_diff -= 2*np.pi
    elif turn_diff < -np.pi:
        turn_diff += 2*np.pi
    
    # turn towards the waypoint
    print("turn_diff",turn_diff)
    turn_time = turn_diff*baseline/(2.0*scale*angular_vel) # replace with your calculation
    # print("Turning for {:.2f} seconds".format(turn_time))
    if np.abs(turn_time) >= 0.01:
        if turn_time < 0:
            command = [0, -0.55]
        if turn_time > 0:
            command = [0, 0.55]
    else:
        command = [0, 0]
    print("turn time",turn_time)

    # if np.abs(turn_time) >= 0.01:
    #     if turn_diff > 0: # turn left
    #         lv, rv = operate.pibot.set_velocity([-0.1, 1], turning_tick=angular_vel, time=turn_time)
    #     elif turn_diff < 0: # turn right
    #         lv, rv = operate.pibot.set_velocity([0.7, -1], turning_tick=angular_vel, time=turn_time)
    # else:
    lv, rv = operate.pibot.set_velocity(command, turning_tick=angular_vel, time=np.abs(turn_time))
    turn_drive_meas = measure.Drive(lv, rv, turn_time)
    time.sleep(0.3)
    slam_estimation(turn_drive_meas)
    # print(operate.ekf.robot.state)

    # print("Driving for {:.2f} seconds".format(drive_time))
    
    
    # # after turning, drive straight to the waypoint
    drive_time = pos_diff/(scale*linear_vel)
    if np.abs(drive_time) > 0.01:
        command = [0.8, 0]
    else:
        command = [0, 0]
    # lv += command[1] * angular_tick
    # rv -= command[1] * angular_tick

    print("drive time",drive_time)
    # #print("Driving for {:.2f} seconds".format(drive_time))
    lv, rv = ppi.set_velocity(command, tick=linear_vel, time=drive_time[0])
    drive_meas = measure.Drive(lv, rv, np.abs(drive_time))
    time.sleep(0.3)
    slam_estimation(drive_meas)
    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))

def drive(correction,wheel_vel=10):
    scale = np.loadtxt("calibration/param/scale.txt", delimiter=',')
    drive_time = correction/(scale*wheel_vel)
    
    print("drive time corr",drive_time)
    if (correction > 0):
        lv, rv = ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    else:
        lv, rv = ppi.set_velocity([-1, 0], tick=wheel_vel, time=drive_time)
    drive_meas = measure.Drive(lv, rv, np.abs(drive_time))
    slam_estimation(drive_meas)
    return


def slam_estimation(drive_meas=None):
    if not operate or not ppi:
        return
    elif drive_meas == None:
        lv,rv = ppi.set_velocity([0,0],tick=10, turning_tick=5, time=0)
        drive_meas = measure.Drive(lv, rv, 0)
        operate.take_pic()
        operate.update_slam(drive_meas)
        # operate.draw(canvas)
        # pygame.display.update()
    elif drive_meas != None:
        operate.take_pic()
        landmarks,_ = operate.aruco_det.detect_marker_positions(operate.img)
        operate.ekf.predict(drive_meas)
        operate.ekf.update(landmarks)
        # operate.draw(canvas)
        # pygame.display.update()
    else:
        return

def get_robot_pose(operate):
    #print(operate.ekf.robot.state)
    return operate.ekf.robot.state

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
    print("path",path)
        # Navigate through the path
    for i in range((len(path))):
        print("waypoint",path[i]) 
        drive_to_point(path[i])
        print(get_robot_pose(operate))     
    time.sleep(3)
    #drive_to_point(waypoint)
        
    return True
    
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
    operate.ekf = operate.init_ekf(args.calib_dir, args.ip)
    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map("M4_true_map.txt")
    search_list = read_search_list()
    waypoints = print_target_fruits_pos(search_list, fruits_list, fruits_true_pos,outputs=True)

    
    robot_pose = [0.0,0.0,0.0]
    addAruco(operate,aruco_true_pos)
    
    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints in command line, you can get coordinates by clicking on a map (GUI input), see camera_calibration.py
        

        # estimate the robot's pose
        

        # robot drives to the waypoint
        
        slam_estimation()
        for i in range(len(waypoints)):
            checker = PATH(waypoints[i])
            if (checker):
                print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoints[i],get_robot_pose(operate)))
            else:
                print("shit doesnt work")

        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break