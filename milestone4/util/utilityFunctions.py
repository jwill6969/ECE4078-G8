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
    print(operate.ekf.taglist)
    
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

    print("Search order:")
    waypoints = []
    n_fruit = 1
    for fruit in search_list:
        for i in range(3):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
                if output is True: waypoints.append(fruit_true_pos[i][0],fruit_true_pos[i][1])

        n_fruit += 1
    if output is True:
        return waypoints