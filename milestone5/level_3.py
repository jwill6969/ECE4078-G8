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
    turn_time = turn_to_point(robot_pose=robot_pose,waypoint=waypoint,wheel_vel=angular_vel) # replace with your calculation
    # print("Turning for {:.2f} seconds".format(turn_time))
    if np.abs(turn_time) >= 0.01:
        if turn_time < 0:
            command = [0, -0.55]
        if turn_time > 0:
            command = [0, 0.55]
        if np.abs(turn_time) > 0.3:
            command[1] = command[1]*0.8
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

#############################
#############################
#############################
def fruitFinder():
    fruits_truelist = {
        "redapple",
        "greenapple",
        "orange",
        "capsicum",
        "mango"
    }
    fruits_list, _, _ = read_true_map("M4_true_map.txt")
    fruits_tofind = []
    for fruit in fruits_truelist:
        if fruit not in fruits_list:
            fruits_tofind.append(fruit)
        
    target_pose_dict = []
    detected_type_list = []
    while fruits_tofind is not None:
        # bboxes = [label,[x,y,width,height]]
        bboxes, _ = operate.detect_target()
        if bboxes is not None:
            robot_pose = get_robot_pose(operate)
            for detection in bboxes:
            # count the occurrence of each target type
                if (detection[0] in fruits_tofind):
                    occurrence = detected_type_list.count(detection[0])
                    target_pose_dict[f'{detection[0]}_{occurrence}'] = estimate_pose(camera_matrix, detection, robot_pose)
                    if (detection[0] in fruits_tofind and occurrence == 3):
                        fruits_tofind.remove(detection[0])
    merged = merge_estimations(target_pose_dict)                




#############################
#############################
#############################
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
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)

    # camera matrix
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fileK = f'{script_dir}/calibration/param/intrinsic.txt'
    camera_matrix = np.loadtxt(fileK, delimiter=',')

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]
    addAruco(operate,aruco_true_pos)
    
    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        slam_estimation()
        # exit
        ppi.set_velocity([0, 0])
        uInput = input("Add a new waypoint? [Y/N]")
        if uInput == 'N':
            break