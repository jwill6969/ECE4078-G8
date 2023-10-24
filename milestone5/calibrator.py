
# basic python packages
from asyncio.windows_events import NULL
from cmath import nan
from ftplib import all_errors
import numpy as np
import argparse
import time
from operate import Operate
import util.measure as measure


def auto_control(speed,tick,t,turning_tick=NULL):
    if t>0:
        if turning_tick!=NULL:
            lv, rv = ppi.set_velocity(speed,tick=tick,turning_tick=turning_tick, time = t)            
        else:
            lv, rv = ppi.set_velocity(speed,tick=tick, time = t)    
    else:
        lv=0   
        rv=0
        t=0
        
    drive_meas = measure.Drive(lv, rv, t)
    operate.control_clock = time.time()
    #ppi.set_velocity([0, 0]) 
    return drive_meas

def check_scale():
    drive_time = 1/(scale*20)
    print("Driving for {:.2f} seconds".format(drive_time))
    auto_control([1, 0], t=drive_time, tick=20)

def check_baseline():
    turn_time = baseline*(np.pi*2)/(2*scale*20) # replace with your calculation
    auto_control([0, 0.925], tick=20, t=turn_time,turning_tick=20)

def check_short_baseline():
    for i in range(12):
        turn_time = short_baseline*(np.pi/6)/(2*scale*40) # replace with your calculation
        auto_control([0, 0.6], tick=20, t=turn_time,turning_tick=40)
        time.sleep(0.5)
if __name__ == "__main__":
    
    ## global file import
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    short_baseline=baseline*1.43
    #########################
    
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='192.168.137.27')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='network/models/yolov8_model.pt')
    args, _ = parser.parse_known_args()


    operate=Operate(args)
    ppi = operate.pibot

    mode=1
    if(mode==0):
        """
            open scale and pibot
            1) check if the robot continuos to go straight, adjust line 22 and 23 accordingly
            2) chekc if the robot starts straight, adjust 39-45 accordingly
            3) check if robot reaches 1m, change scale
        """
        check_scale()
    elif(mode==1):
        """
            open baseline
            1) check if the robot spins 360, adjust baseline accordingly
        """
        check_baseline()

    elif(mode==2):
        """
            1) check if the robot spins 360, adjust short_baseline accordingly
        """
        check_short_baseline()
