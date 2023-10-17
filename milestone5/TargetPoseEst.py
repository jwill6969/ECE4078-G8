# estimate the pose of target objects detected
import numpy as np
import json
import os
import ast
import cv2
import sys
from ultralytics import YOLO    
from network.scripts.detector import Detector
from scipy.signal import medfilt
from machinevisiontoolbox import Image
import PIL
# list of target fruits and vegs types
# Make sure the names are the same as the ones used in your YOLO model
TARGET_TYPES = ['orange','red apple', 'capsicum','green apple','mango']


def estimate_pose(camera_matrix, obj_info, robot_pose):
    """
    function:
        estimate the pose of a target based on size and location of its bounding box and the corresponding robot pose
    input:
        camera_matrix: list, the intrinsic matrix computed from camera calibration (read from 'param/intrinsic.txt')
            |f_x, s,   c_x|
            |0,   f_y, c_y|
            |0,   0,   1  |
            (f_x, f_y): focal length in pixels
            (c_x, c_y): optical centre in pixels
            s: skew coefficient (should be 0 for PenguinPi)
        obj_info: list, an individual bounding box in an image (generated by get_bounding_box, [label,[x,y,width,height]])
        robot_pose: list, pose of robot corresponding to the image (read from 'lab_output/images.txt', [x,y,theta])
    output:
        target_pose: dict, prediction of target pose
    """
    # read in camera matrix (from camera calibration results)
    focal_length = camera_matrix[0][0]

    # there are 8 possible types of fruits and vegs
    ######### Replace with your codes #########
    # TODO: measure actual sizes of targets [width, depth, height] and update the dictionary of true target dimensions
    target_dimensions_dict = {'orange': [0.075, 0.075, 0.072], 'red apple':  [0.074, 0.074, 0.087], 
                              'mango':  [0.113, 0.067, 0.058], 'green apple': [0.081, 0.081, 0.067], 
                              'capsicum': [0.073, 0.073, 0.088]}
    #########

    # estimate target pose using bounding box and robot pose
    target_class = obj_info[0]     # get predicted target label of the box
    target_box = obj_info[1]       # get bounding box measures: [x,y,width,height]
    true_height = target_dimensions_dict[target_class][2]   # look up true height of by class label

    # compute pose of the target based on bounding box info, true object height, and robot's pose
    pixel_height = target_box[3]
    pixel_center = target_box[0]
    distance = true_height/pixel_height * focal_length  # estimated distance between the robot and the centre of the image plane based on height
    # training image size 320x240p
    image_width = 640 # change this if your training image is in a different size (check details of pred_0.png taken by your robot)
    x_shift = image_width/2 - pixel_center              # x distance between bounding box centre and centreline in camera view
    theta = (np.arctan(x_shift/focal_length))     # angle of object relative to the robot
    ang = theta + robot_pose[2]     # angle of object in the world frame with 2% error mitigation
    
   # relative object location
    distance_obj = distance/np.cos(theta) # relative distance between robot and object
    x_relative = distance_obj * np.cos(theta) # relative x pose
    y_relative = distance_obj * np.sin(theta) # relative y pose
    relative_pose = {'x': x_relative, 'y': y_relative}
    #print(f'relative_pose: {relative_pose}')

    # location of object in the world frame using rotation matrix
    delta_x_world = (x_relative * np.cos(ang) - y_relative * np.sin(ang)) # 1.17159
    delta_y_world = (x_relative * np.sin(ang) + y_relative * np.cos(ang)) # 1.17159
    # add robot pose with delta target pose
    target_pose = {'y': (robot_pose[1]+delta_y_world)[0],
                   'x': (robot_pose[0]+delta_x_world)[0]}
    #print(f'delta_x_world: {delta_x_world}, delta_y_world: {delta_y_world}')
    #print(f'target_pose: {target_pose}')

    return target_pose
def filtering(estimation_array):
    # estimation array =[[x,y],[x,y]]
    filtered = []
    array = []
    
    if len(estimation_array)== 1:
        filtered = estimation_array
    else:
        for i in range(len(estimation_array)):
            array.append(np.sqrt(estimation_array[i][0]**2+estimation_array[i][1]**2))
        for i in range(len(estimation_array)):
            if ((array[i]>np.quantile(array,0.25)) and (array[i]<np.quantile(array,0.75))):
                filtered.append(estimation_array[i])

    if len(filtered) == 0:
        filtered = estimation_array
    
    return filtered

def merge_estimations(target_pose_dict):
    num_per_target = 1
    """
    function:
        merge estimations of the same target
    input:
        target_pose_dict: dict, generated by estimate_pose
    output:
        target_est: dict, target pose estimations after merging
    """
    target_est = {}

    ######### Replace with your codes #########
    # TODO: replace it with a solution to merge the multiple occurrences of the same class type (e.g., by a distance threshold)
    

    redapple_est = []
    greenapple_est = []
    orange_est = []
    mango_est = []
    capsicum_est = []
    
    for est in target_pose_dict:
        if est.__contains__("red apple"):
            redapple_est.append([target_pose_dict[est]["x"], target_pose_dict[est]["y"]])
        elif est.__contains__("green apple"):
            greenapple_est.append([target_pose_dict[est]["x"], target_pose_dict[est]["y"]])
        elif est.__contains__("mango"):
            mango_est.append([target_pose_dict[est]["x"], target_pose_dict[est]["y"]])
        elif est.__contains__("capsicum"):
            capsicum_est.append([target_pose_dict[est]["x"], target_pose_dict[est]["y"]])
        else:
            orange_est.append([target_pose_dict[est]["x"], target_pose_dict[est]["y"]])

    
    print(greenapple_est)
    print(orange_est)
    print(mango_est)
    print(capsicum_est)
    redapple_est = [np.average(filtering(redapple_est),axis=0)]     
    greenapple_est = [np.average(filtering(greenapple_est),axis=0)] 
    orange_est = [np.average(filtering(orange_est),axis=0)]
    mango_est = [np.average(filtering(mango_est),axis=0)]
    capsicum_est = [np.average(filtering(capsicum_est),axis=0)]

    


    if redapple_est != [] and np.isnan(np.all(redapple_est)) == False:
        target_est["redapple_0"] = {
            "x":redapple_est[0][0],
            "y":redapple_est[0][1]
        }
    if greenapple_est != [] and np.isnan(np.all(greenapple_est)) == False:
        target_est["greenapple_0"] = {
            "x":greenapple_est[0][0],
            "y":greenapple_est[0][1]
        }
    if orange_est != [] and np.isnan(np.all(orange_est)) == False:
        target_est["orange_0"] = {
            "x":orange_est[0][0],
            "y":orange_est[0][1]
        }
    if mango_est != [] and np.isnan(np.all(mango_est)) == False:
        target_est["mango_0"] = {
            "x":mango_est[0][0],
            "y":mango_est[0][1]
        }
    if capsicum_est != [] and np.isnan(np.all(capsicum_est)) == False:
        target_est["capsicum_0"] = {
            "x":capsicum_est[0][0],
            "y":capsicum_est[0][1]
        }
    return target_est

def get_image_info(base_dir, file_path, image_poses):
    # there are at most five types of targets in each image
    target_lst_box = [[], [], [], [], []]
    target_lst_pose = [[], [], [], [], []]
    completed_img_dict = {}

    # add the bounding box info of each target in each image
    # target labels: 1 = redapple, 2 = greenapple, 3 = orange, 4 = mango, 5=capsicum, 0 = not_a_target
    img_vals = set(Image(base_dir / file_path, grey=True).image.reshape(-1))
    #print(img_vals)
    for target_num in img_vals:
        if target_num > 0:
            try:
                box =   (target_num, base_dir/file_path) # [x,y,width,height]
                pose = image_poses[file_path] # [x, y, theta]
                target_lst_box[target_num-1].append(box) # bouncing box of target
                target_lst_pose[target_num-1].append(np.array(pose).reshape(3,)) # robot pose
            except ZeroDivisionError:
                pass

    # if there are more than one objects of the same type, combine them
    for i in range(5):
        if len(target_lst_box[i])>0:
            box = np.stack(target_lst_box[i], axis=1)
            pose = np.stack(target_lst_pose[i], axis=1)
            completed_img_dict[i+1] = {'target': box, 'robot': pose}
        
    return completed_img_dict

def get_bounding_box(target_number, image_path):
    image = PIL.Image.open(image_path).resize((640,480), PIL.Image.Resampling.NEAREST)
    target = Image(image)==target_number
    blobs = target.blobs()
    [[u1,u2],[v1,v2]] = blobs[0].bbox # bounding box
    width = abs(u1-u2)
    height = abs(v1-v2)
    center = np.array(blobs[0].centroid).reshape(2,)
    box = [center[0], center[1], int(width), int(height)] # box=[x,y,width,height]
    # plt.imshow(fruit.image)
    # plt.annotate(str(fruit_number), np.array(blobs[0].centroid).reshape(2,))
    # plt.show()
    # assert len(blobs) == 1, "An image should contain only one object of each target type"
    return box

# main loop
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))     # get current script directory (TargetPoseEst.py)

    # read in camera matrix
    fileK = f'{script_dir}/calibration/param/intrinsic.txt'
    camera_matrix = np.loadtxt(fileK, delimiter=',')

    # init YOLO model
    model_path = f'{script_dir}/network/scripts/model/yolov8_model.pt'
    yolo = Detector(model_path)

    # create a dictionary of all the saved images with their corresponding robot pose
    image_poses = {}
    with open(f'{script_dir}/lab_output/images.txt') as fp:
        for line in fp.readlines():
            pose_dict = ast.literal_eval(line)
            image_poses[pose_dict['imgfname']] = pose_dict['pose']

    # estimate pose of targets in each image
    target_pose_dict = {}
    detected_type_list = []
    for image_path in image_poses.keys():
        input_image = cv2.imread(image_path)
        bounding_boxes, bbox_img = yolo.detect_single_image(input_image)
        # cv2.imshow('bbox', bbox_img)
        # cv2.waitKey(0)
        robot_pose = image_poses[image_path]

        for detection in bounding_boxes:
            # count the occurrence of each target type
            occurrence = detected_type_list.count(detection[0])
            target_pose_dict[f'{detection[0]}_{occurrence}'] = estimate_pose(camera_matrix, detection, robot_pose)

            detected_type_list.append(detection[0])

    # merge the estimations of the targets so that there are at most 3 estimations of each target type
    target_est = {}
    target_est = merge_estimations(target_pose_dict)
    #print(target_est)
    # save target pose estimations
    with open(f'{script_dir}/lab_output/targets.txt', 'w') as fo:
        json.dump(target_est, fo, indent=4)

    print('Estimations saved!')