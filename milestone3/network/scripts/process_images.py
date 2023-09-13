import numpy as np
import os
import cv2
import imutils

def random_transform(fruit_dir):

    fruit = cv2.imread("images_fruits/" + fruit_dir)
    fruit = cv2.copyMakeBorder(fruit,23,24,32,31,cv2.BORDER_CONSTANT,value= [0, 0, 0])
    fruit = fruit[:, :, 0:3]
    height = np.shape(fruit)[0]
    width = np.shape(fruit)[1]

    # random zooming
    cy, cx = [ i/2 for i in fruit.shape[:-1] ]
    rot_mat = cv2.getRotationMatrix2D((cx,cy), 0, 0.7*np.random.rand() + 0.3)
    fruit = cv2.warpAffine(fruit, rot_mat, fruit.shape[1::-1], flags=cv2.INTER_LINEAR)
        
    # random translation
    tx = np.random.randint(low=-np.floor(width/3), high=np.ceil(width/3))
    ty = np.random.randint(low=-np.floor(height/3), high=np.floor(height/3))
    T = np.float32([[1, 0, tx], [0, 1, ty]])
    fruit = cv2.warpAffine(fruit, T, (width, height))
    
    # random rotation
    fruit = imutils.rotate(fruit, np.random.randint(low=0, high=359))

    return fruit

def superimpose_and_label(fruit, background, fruit_label, label = [0]):

    if np.shape(label)[0] == 1:
        label = np.zeros_like(fruit[:, :, 0])

    background[np.sum(fruit, 2) > 0] = 0
    superimposed_image = fruit + background

    label[np.sum(fruit, 2) > 0] = fruit_label

    return [superimposed_image, label]

def process(fruits_dir, background_dir, num_images):

    fileList_fruit = os.listdir(fruits_dir)
    fileList_background = os.listdir(background_dir)
    redapple_arr = []
    capsicum_arr = []
    greenapple_arr = []
    orange_arr = []
    mango_arr = []
    bg_arr = []
    for fruit_dir in fileList_fruit:
        if (os.path.basename(fruit_dir).find("capsicum") >= 0):
            capsicum_arr.append(fruit_dir)

        elif (os.path.basename(fruit_dir).find("greenapple") >= 0):
            greenapple_arr.append(fruit_dir)

        elif (os.path.basename(fruit_dir).find("mango") >= 0):
            mango_arr.append(fruit_dir)

        elif (os.path.basename(fruit_dir).find("orange") >= 0):
            orange_arr.append(fruit_dir)

        elif (os.path.basename(fruit_dir).find("redapple") >= 0):
            redapple_arr.append(fruit_dir)

    sorted_arr = [redapple_arr, greenapple_arr, orange_arr, mango_arr, capsicum_arr]

    for bg_dir in fileList_background:
        bg_arr.append(bg_dir)
    
    k = 0
    for j in range(num_images):
        label_arr = [1, 2, 3, 4, 5]

        cur_label = np.random.choice(label_arr)
        label_arr = np.delete(label_arr, np.where(label_arr == cur_label))

        cur_fruit = np.random.choice(sorted_arr[cur_label-1])
        transformed_fruit = random_transform(cur_fruit)

        index = np.random.randint(0, len(bg_arr)-1)
        bg = cv2.imread("images_arena/" + bg_arr[index])
        [bg, new_label] = superimpose_and_label(transformed_fruit, bg, cur_label)
        savename_img = "images/image_" + str(k) + ".png"
        cv2.imwrite(savename_img, bg)

        savename_label = "labels/image_" + str(k) + "_label.png"
        cv2.imwrite(savename_label, new_label)

        k = k + 1

        for i in range(4):
            cur_label = np.random.choice(label_arr)
            label_arr = np.delete(label_arr, np.where(label_arr == cur_label))

            cur_fruit = np.random.choice(sorted_arr[cur_label-1])
            transformed_fruit = random_transform(cur_fruit)

            [bg, new_label] = superimpose_and_label(transformed_fruit, bg, cur_label, new_label)

            savename_img = "images/image_" + str(k) + ".png"
            cv2.imwrite(savename_img, bg)

            savename_label = "labels/image_" + str(k) + "_label.png"
            cv2.imwrite(savename_label, new_label)

            k = k + 1
        
path_fruits = "images_fruits"
path_arena = "images_arena"

process(path_fruits, path_arena, 1000)








