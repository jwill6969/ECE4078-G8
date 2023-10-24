# teleoperate the robot, perform SLAM and object detection

# basic python packages
import numpy as np
import cv2 
import os, sys
import time

# import utility functions
sys.path.insert(0, "{}/utility".format(os.getcwd()))
from util.pibot import Alphabot # access the robot
import util.DatasetHandler as dh # save/load functions
import util.measure as measure # measurements
import pygame # python package for GUI
import shutil # python package for file operations
from util.utilityFunctions import *
from util.TargetPoseEst import merge_estimations,filtering,estimate_pose

# import SLAM components you developed in M2
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import CV components
sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector

from final_eval import parse_and_sort,evaluate_map,evaluate_map_00

class Operate:
    def __init__(self, args):
        self.folder = 'pibot_dataset/'
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
        else:
            shutil.rmtree(self.folder)
            os.makedirs(self.folder)
        
        # initialise data parameters
        if args.play_data:
            self.pibot = dh.DatasetPlayer("record")
        else:
            self.pibot = Alphabot(args.ip, args.port)

        # initialise SLAM parameters
        self.ekf = self.init_ekf(args.calib_dir, args.ip)
        self.aruco_det = aruco.aruco_detector(
            self.ekf.robot, marker_length = 0.06) # size of the ARUCO markers

        if args.save_data:
            self.data = dh.DatasetWriter('record')
        else:
            self.data = None
        self.output = dh.OutputWriter('lab_output')
        self.command = {'motion':[0, 0], 
                        'inference': False,
                        'output': False,
                        'save_inference': False,
                        'save_image': False}
        self.quit = False
        self.pred_fname = ''
        self.request_recover_robot = False
        self.file_output = None
        self.ekf_on = False
        self.double_reset_comfirm = 0
        self.image_id = 0
        self.notification = 'Press ENTER to start SLAM'
        # a 5min timer
        self.count_down = 300
        self.endpos = []
        self.start_time = time.time()
        self.control_clock = time.time()
        # initialise images
        self.img = np.zeros([240,320,3], dtype=np.uint8)
        self.aruco_img = np.zeros([240,320,3], dtype=np.uint8)
        self.detector_output = np.zeros([240,320], dtype=np.uint8)
        
        # adding map saving and camera matrix
        self.stopUpdate = True
        self.scale = 0
        self.baseline = 0
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        fileK = f'{self.script_dir}/calibration/param/intrinsic.txt'
        fileS = f"{self.script_dir}/calibration/param/scale.txt"
        self.scale = np.loadtxt(fileS, delimiter=',')
        fileB = f"{self.script_dir}/calibration/param/baseline.txt"  
        self.baseline = np.loadtxt(fileB, delimiter=',')
        self.camera_matrix = np.loadtxt(fileK, delimiter=',')
        self.tag_ground_truth = {}
        self.map_dict = {}
        self.fruit_dict= {}
        self.map_dict_unaligned = {}
        if args.ckpt == "":
            self.detector = None
            self.network_vis = cv2.imread('pics/8bit/detector_splash.png')
        else:
            self.detector = Detector(args.ckpt)
            self.network_vis = np.ones((240, 320,3))* 100
        self.bg = pygame.image.load('pics/gui_mask.jpg')

    def set_scale_baseline(self,datadir):
        fileS = "{}scale.txt".format(datadir)
        self.scale = np.loadtxt(fileS, delimiter=',')
        fileB = "{}baseline.txt".format(datadir)  
        self.baseline = np.loadtxt(fileB, delimiter=',')

    # wheel control
    def control(self):       
        if args.play_data:
            lv, rv = self.pibot.set_velocity()            
        else:
            lv, rv = self.pibot.set_velocity(
                self.command['motion'])
        if not self.data is None:
            self.data.write_keyboard(lv, rv)
        dt = time.time() - self.control_clock
        drive_meas = measure.Drive(lv, rv, dt)
        self.control_clock = time.time()
        return drive_meas
    # camera control
    def take_pic(self):
        self.img = self.pibot.get_image()
        if not self.data is None:
            self.data.write_image(self.img)

    # SLAM with ARUCO markers       
    def update_slam(self, drive_meas):
        lms, self.aruco_img = self.aruco_det.detect_marker_positions(self.img)
        if self.request_recover_robot:
            is_success = self.ekf.recover_from_pause(lms)
            if is_success:
                self.notification = 'Robot pose is successfuly recovered'
                self.ekf_on = True
            else:
                self.notification = 'Recover failed, need >2 landmarks!'
                self.ekf_on = False
            self.request_recover_robot = False
        elif self.ekf_on: # and not self.debug_flag:
            self.ekf.predict(drive_meas)
            self.ekf.add_landmarks(lms)
            self.ekf.update(lms)

    # using computer vision to detect targets
    def detect_target(self):
        if self.command['inference'] and self.detector is not None:
            
            # need to convert the colour before passing to YOLO
            yolo_input_img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)

            self.detector_output, self.network_vis = self.detector.detect_single_image(yolo_input_img)

            # covert the colour back for display purpose
            self.network_vis = cv2.cvtColor(self.network_vis, cv2.COLOR_RGB2BGR)

            self.command['inference'] = False     # uncomment this if you do not want to continuously predict
            self.file_output = (yolo_input_img, self.ekf)

            # self.notification = f'{len(self.detector_output)} target type(s) detected'

    # save raw images taken by the camera
    def save_image(self):
        f_ = os.path.join(self.folder, f'img_{self.image_id}.png')
        if self.command['save_image']:
            image = self.pibot.get_image()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(f_, image)
            self.image_id += 1
            self.command['save_image'] = False
            self.notification = f'{f_} is saved'

    # wheel and camera calibration for SLAM
    def init_ekf(self, datadir, ip):
        fileK = "{}intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        fileS = "{}scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=',')
        if ip == 'localhost':
            scale /= 2
        fileB = "{}baseline.txt".format(datadir)  
        baseline = np.loadtxt(fileB, delimiter=',')
        robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
        return EKF(robot)

    # save SLAM map
    def record_data(self):
        if self.command['output']:
            self.output.write_map(self.ekf)
            self.notification = 'Map is saved'
            self.command['output'] = False
        # save inference with the matching robot pose and detector labels
        if self.command['save_inference']:
            if self.file_output is not None:
                #image = cv2.cvtColor(self.file_output[0], cv2.COLOR_RGB2BGR)
                self.pred_fname = self.output.write_image(self.file_output[0],
                                                        self.file_output[1])
                self.notification = f'Prediction is saved to {operate.pred_fname}'
            else:
                self.notification = f'No prediction in buffer, save ignored'
            self.command['save_inference'] = False

    # paint the GUI            
    def draw(self, canvas):
        canvas.blit(self.bg, (0, 0))
        text_colour = (220, 220, 220)
        v_pad = 40
        h_pad = 20

        # paint SLAM outputs
        ekf_view = self.ekf.draw_slam_state(res=(320, 480+v_pad),
            not_pause = self.ekf_on)
        canvas.blit(ekf_view, (2*h_pad+320, v_pad))
        robot_view = cv2.resize(self.aruco_img, (320, 240))
        self.draw_pygame_window(canvas, robot_view, 
                                position=(h_pad, v_pad)
                                )

        # for target detector (M3)
        detector_view = cv2.resize(self.network_vis,
                                (320, 240), cv2.INTER_NEAREST)
        self.draw_pygame_window(canvas, detector_view, 
                                position=(h_pad, 240+2*v_pad)
                                )

        # canvas.blit(self.gui_mask, (0, 0))
        self.put_caption(canvas, caption='SLAM', position=(2*h_pad+320, v_pad))
        self.put_caption(canvas, caption='Detector',
                        position=(h_pad, 240+2*v_pad))
        self.put_caption(canvas, caption='PiBot Cam', position=(h_pad, v_pad))

        notifiation = TEXT_FONT.render(self.notification,
                                        False, text_colour)
        canvas.blit(notifiation, (h_pad+10, 596))

        time_remain = self.count_down - time.time() + self.start_time
        if time_remain > 0:
            time_remain = f'Count Down: {time_remain:03.0f}s'
        elif int(time_remain)%2 == 0:
            time_remain = "Time Is Up !!!"
        else:
            time_remain = ""
        count_down_surface = TEXT_FONT.render(time_remain, False, (50, 50, 50))
        canvas.blit(count_down_surface, (2*h_pad+320+5, 530))
        return canvas

    @staticmethod
    def draw_pygame_window(canvas, cv2_img, position):
        cv2_img = np.rot90(cv2_img)
        view = pygame.surfarray.make_surface(cv2_img)
        view = pygame.transform.flip(view, True, False)
        canvas.blit(view, position)
    
    @staticmethod
    def put_caption(canvas, caption, position, text_colour=(200, 200, 200)):
        caption_surface = TITLE_FONT.render(caption,
                                        False, text_colour)
        canvas.blit(caption_surface, (position[0], position[1]-25))

    # keyboard teleoperation        
    def update_keyboard(self):
        for event in pygame.event.get():
            # drive forward

            if event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                self.command['motion'] = [0.9,0]
            # drive backward
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                self.command['motion'] = [-0.875,0]
            # turn left
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                turn_time = self.baseline*(np.pi/6)/(2*self.scale*40)
                
                lv, rv = operate.pibot.set_velocity([0, 0.95],tick=20, turning_tick=40, time=np.abs(turn_time))
                drive_meas = measure.Drive(lv, rv, 1.3*np.abs(turn_time)) ## THE factor for
                
                self.update_slam(drive_meas)
                # print(get_robot_pose(self)[2]*180/np.pi)
            # drive right
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                turn_time = self.baseline*(np.pi/6)/(2*self.scale*40)
                lv, rv = operate.pibot.set_velocity([0, -0.95],tick=20, turning_tick=40, time=np.abs(turn_time))
                
                drive_meas = measure.Drive(lv, rv, 1.45*np.abs(turn_time))
                self.update_slam(drive_meas)
                # print(get_robot_pose(self)[2]*180/np.pi)
            # stop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.command['motion'] = [0, 0]
            # save image
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_i:
                self.command['save_image'] = True
            # save SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                    self.command['output'] = True
                    self.saved_map = True
                    _, unaligned_points_arranged = parse_and_sort()
                    arranged = convert_shape(unaligned_points_arranged)
                    self.map_dict_unaligned = convertArrayToMap(arranged)
                    points = evaluate_map(self.tag_ground_truth)
                    endpos_true = [[0],[0]]
                    self.endpos = get_robot_pose(self)[:-1]
                    if dist_between_points(endpos_true,self.endpos) > 0.1:
                        print("ERROR BETWEEN 0,0",dist_between_points(endpos_true,self.endpos))
                        self.translation = calculate_translation(true_pose=endpos_true,robot_pose=self.endpos)
                        points = translate_coordinates(points = points,translation=self.translation)
                    # points = evaluate_map_00(self.tag_ground_truth,self.endpos)
                    self.map_dict = convertArrayToMap(points)
                    printCompare(aligned=points,unaligned=arranged)
                    if (self.stopUpdate is True):
                        chooser = input("1 for unaligned, 0 for aligned")
                        theta = get_robot_pose(self)[2] # save theta value add to slam after reset
                        
                        if (chooser == 1):
                            self.ekf.reset()
                            arranged = revert_shape(arranged)
                            addAruco(self,arranged)
                        elif chooser == 0:
                            self.ekf.reset()
                            points = revert_shape(points)
                            addAruco(self,points)
                        else:
                            continue
                        #self.ekf.robot.state[2] = theta
                        print("hi")
                        
                    
            #CHECKER: take pic and calculate xy poses to check 
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_o:
                robot_pose = get_robot_pose(self)
                self.take_pic()
                yolo_input_img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
                bboxlist, self.network_vis = self.detector.detect_single_image(yolo_input_img)
                for bbox  in bboxlist:
                    # print("estimated fruit position",estimate_pose(self.camera_matrix, bbox, robot_pose))
                    point = estimate_pose(self.camera_matrix, bbox, robot_pose)
                    coords = [point['x'],point['y']]
                    ans = dist_between_points(coords,robot_pose[:-1])
                    if ans < 0.8:
                        print("fruit added:",bbox[0],point)
                        self.fruit_dict = addFruitToDict(self.fruit_dict,coords,bbox[0])
                    else:
                        print("FILTERED",bbox[0])

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_c:
                    robot_pose = get_robot_pose(self)
                    print("Current Robot Pose:",robot_pose)
                    yolo_input_img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
                    bboxlist, self.network_vis = self.detector.detect_single_image(yolo_input_img)
                    for bbox  in bboxlist:
                        print("estimated fruit position",estimate_pose(self.camera_matrix, bbox, robot_pose))
            # reset SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                if self.double_reset_comfirm == 0:
                    self.notification = 'Press again to confirm CLEAR MAP'
                    self.double_reset_comfirm +=1
                elif self.double_reset_comfirm == 1:
                    self.notification = 'SLAM Map is cleared'
                    self.double_reset_comfirm = 0
                    self.ekf.reset()
            # run SLAM
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                n_observed_markers = len(self.ekf.taglist)
                if n_observed_markers == 0:
                    if not self.ekf_on:
                        self.notification = 'SLAM is running'
                        self.ekf_on = True
                        self.take_pic()
                        measurements,_ = self.aruco_det.detect_marker_positions(self.img)
                        if len(measurements) > 0:
                            self.tag_ground_truth[measurements[0].getTag()] = measurements[0].getPos()
                            # print("SAVE THIS POSITION OF THE ARUCO",self.tag_ground_truth)
                    else:
                        self.notification = '> 2 landmarks is required for pausing'
                elif n_observed_markers < 3:
                    self.notification = '> 2 landmarks is required for pausing'
                else:
                    if not self.ekf_on:
                        self.request_recover_robot = True
                    self.ekf_on = not self.ekf_on
                    if self.ekf_on:
                        self.notification = 'SLAM is running'
                    else:
                        self.notification = 'SLAM is paused'
            # run object detector
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                self.command['inference'] = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                coords = get_robot_pose(self)
                print("x:",coords[0][0],"  y:",coords[1][0],"  t:",coords[2][0])
            elif event.type == pygame.QUIT:
                self.quit = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                if self.map_dict != {}:
                    print("fruit_dict",self.fruit_dict)
                    print("map_dict",self.map_dict)
                    self.fruit_dict = merge_estimations(self.fruit_dict)
                    with open(f'{self.script_dir}/targets.txt', 'w') as fo:
                        json.dump(self.fruit_dict, fo, indent=4)
                    final = {**self.map_dict, **self.fruit_dict}
                    final_unaligned = {**self.map_dict_unaligned, **self.fruit_dict}
                    with open(f'{self.script_dir}/final_map.txt', 'w') as fo:
                        json.dump(final, fo, indent=4)
                    with open(f'{self.script_dir}/final_map_unaligned.txt', 'w') as fo:
                        json.dump(final_unaligned, fo, indent=4)
                 # convert map_dict to file
                self.quit = True
            else:
                self.command['motion'] = [0,0]
        if self.quit:
            pygame.quit()
            sys.exit()

        
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='192.168.137.27')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--ckpt", default='network/models/yolov8_model.pt')
    #parser.add_argument("estimate", type=str, help="The estimate file name.")
    args, _ = parser.parse_known_args()
    
    pygame.font.init() 
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)
    
    width, height = 700, 660
    canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption('ECE4078 2021 Lab')
    pygame.display.set_icon(pygame.image.load('pics/8bit/pibot5.png'))
    canvas.fill((0, 0, 0))
    splash = pygame.image.load('pics/loading.png')
    pibot_animate = [pygame.image.load('pics/8bit/pibot1.png'),
                     pygame.image.load('pics/8bit/pibot2.png'),
                     pygame.image.load('pics/8bit/pibot3.png'),
                    pygame.image.load('pics/8bit/pibot4.png'),
                     pygame.image.load('pics/8bit/pibot5.png')]
    pygame.display.update()

    start = False

    counter = 40
    while not start:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                start = True
        canvas.blit(splash, (0, 0))
        x_ = min(counter, 600)
        if x_ < 600:
            canvas.blit(pibot_animate[counter%10//2], (x_, 565))
            pygame.display.update()
            counter += 2

    operate = Operate(args)
    # get camera calibration files


    while start:
        operate.update_keyboard()
        operate.take_pic()
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        pygame.display.update()



