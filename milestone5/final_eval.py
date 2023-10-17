# evaluate the map generated by SLAM against the true map
import ast
import numpy as np
import json
import matplotlib.pyplot as plt

def parse_groundtruth(fname : str) -> dict:
    with open(fname, 'r') as f:
        try:
            gt_dict = json.load(f)                   
        except ValueError as e:
            with open(fname, 'r') as f:
                gt_dict = ast.literal_eval(f.readline()) 
        
        aruco_dict = {}
        for key in gt_dict:
            if key.startswith("aruco"):
                aruco_num = int(key.strip('aruco')[:-2])
                aruco_dict[aruco_num] = np.reshape([gt_dict[key]["x"], gt_dict[key]["y"]], (2,1))
    return aruco_dict

def parse_user_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        try:
            usr_dict = json.load(f)                   
        except ValueError as e:
            with open(fname, 'r') as f:
                usr_dict = ast.literal_eval(f.readline()) 
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[tag] = np.reshape([usr_dict["map"][0][i],usr_dict["map"][1][i]], (2,1))
    return aruco_dict

def match_aruco_points(aruco0 : dict, aruco1 : dict):
    points0 = []
    points1 = []
    keys = []
    for key in aruco0:
        if not key in aruco1:
            continue
        
        points0.append(aruco0[key])
        points1.append(aruco1[key])
        keys.append(key)
    return keys, np.hstack(points0), np.hstack(points1)

def sort_aruco_list(aruco_dict):

    true_dict = [1,2,3,4,5,6,7,8,9,10]
    for key in aruco_dict:
        if not key in true_dict:
            continue
    
            
    
def solve_umeyama2d(points1, points2):
    # Solve the optimal transform such that
    # R(theta) * p1_i + t = p2_i

    assert(points1.shape[0] == 2)
    assert(points1.shape[0] == points2.shape[0])
    assert(points1.shape[1] == points2.shape[1])


    # Compute relevant variables
    num_points = points1.shape[1]
    mu1 = 1/num_points * np.reshape(np.sum(points1, axis=1),(2,-1))
    mu2 = 1/num_points * np.reshape(np.sum(points2, axis=1),(2,-1))
    sig1sq = 1/num_points * np.sum((points1 - mu1)**2.0)
    sig2sq = 1/num_points * np.sum((points2 - mu2)**2.0)
    Sig12 = 1/num_points * (points2-mu2) @ (points1-mu1).T

    # Use the SVD for the rotation
    U, d, Vh = np.linalg.svd(Sig12)
    S = np.eye(2)
    if np.linalg.det(Sig12) < 0:
        S[-1,-1] = -1
    
    # Return the result as an angle and a 2x1 vector
    R = U @ S @ Vh
    theta = np.arctan2(R[1,0],R[0,0])
    x = mu2 - R @ mu1

    return theta, x

def apply_transform(theta, x, points):
    # Apply an SE(2) transform to a set of 2D points
    assert(points.shape[0] == 2)
    
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    
    points_transformed =  R @ points + x
    return points_transformed


def compute_rmse(points1, points2):
    # Compute the RMSE between two matched sets of 2D points.
    assert(points1.shape[0] == 2)
    assert(points1.shape[0] == points2.shape[0])
    assert(points1.shape[1] == points2.shape[1])
    num_points = points1.shape[1]
    residual = (points1-points2).ravel()
    MSE = 1.0/num_points * np.sum(residual**2)

    return np.sqrt(MSE)

def sorter(aruco_dict =  None):
    sorted_array = []
    keys = []
    if aruco_dict is not None:
        sorted_dict = dict(sorted(aruco_dict.items()))
        for key in sorted_dict:
            if key > 10 or key < 0:
                continue
            else:
                keys.append(key)
                sorted_array.append(sorted_dict[key])
    return sorted_array
def parse_and_sort():
    
    parsed_map = parse_user_map('lab_output/slam.txt')
    aruco_coords = sorter(parsed_map)
    return parsed_map, aruco_coords

def evaluate_map(tag_ground_truth):
    us_aruco, aruco_coords =  parse_and_sort()
    gt_aruco = tag_ground_truth
    gt_aruco[0] = np.array([[    0],[   0]])
    us_aruco[0] = np.array([[    0],[   0]])
    taglist, us_vec, gt_vec = match_aruco_points(us_aruco, gt_aruco)

    idx = np.argsort(taglist)
    taglist = np.array(taglist)[idx]
    us_vec = us_vec[:,idx]
    gt_vec = gt_vec[:, idx]
    
    theta, x = solve_umeyama2d(us_vec, gt_vec)
    pretransform = [[],[]]
    for i in range(len(aruco_coords)):
        x_cord = aruco_coords[i][0][0]
        y_cord = aruco_coords[i][1][0]
        pretransform[0].append(x_cord)
        pretransform[1].append(y_cord)
    
    us_vec_aligned = apply_transform(theta, x, np.array(pretransform))

    return us_vec_aligned
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser("Matching the estimated map and the true map")
    parser.add_argument("groundtruth", type=str, help="The ground truth file name.")
    parser.add_argument("estimate", type=str, help="The estimate file name.")
    args = parser.parse_args()

    gt_aruco = parse_groundtruth(args.groundtruth)
    tag_ground_truth = {}
    tag_ground_truth[4] = np.array([[   1.1559],[   0.0081216]])
    alligned = evaluate_map(tag_ground_truth)
    us_aruco = parse_user_map(args.estimate)
    taglist, us_vec, gt_vec = match_aruco_points(us_aruco, gt_aruco)
    
    idx = np.argsort(taglist)
    taglist = np.array(taglist)[idx]
    us_vec = us_vec[:,idx]
    gt_vec = gt_vec[:, idx] 
    print("gt_vec",gt_vec)
    rmse_aligned = compute_rmse(alligned, gt_vec)
    rmse = compute_rmse(us_vec, gt_vec)
    print(rmse_aligned)
    print(rmse)
    # theta, x = solve_umeyama2d(us_vec, gt_vec)
    # us_vec_aligned = apply_transform(theta, x, us_vec)
    
    diff = gt_vec - alligned
    # rmse = compute_rmse(us_vec, gt_vec)
    # rmse_aligned = compute_rmse(us_vec_aligned, gt_vec)
    
    # print()
    # print("The following parameters optimally transform the estimated points to the ground truth.")
    # print("Rotation Angle: {}".format(theta))
    # print("Translation Vector: ({}, {})".format(x[0,0], x[1,0]))
    
    # print()
    # print("Number of found markers: {}".format(len(taglist)))
    # print("RMSE before alignment: {}".format(rmse))
    # print("RMSE after alignment:  {}".format(rmse_aligned))
    
    print()
    print('%s %7s %9s %7s %11s %9s %7s' % ('Marker', 'Real x', 'Pred x', 'Δx', 'Real y', 'Pred y', 'Δy'))
    print('-----------------------------------------------------------------')
    for i in range(len(taglist)):
        print('%3d %9.2f %9.2f %9.2f %9.2f %9.2f %9.2f\n' % (taglist[i], gt_vec[0][i], alligned[0][i], diff[0][i], gt_vec[1][i], alligned[1][i], diff[1][i]))
    
    ax = plt.gca()
    ax.scatter(gt_vec[0,:], gt_vec[1,:], marker='o', color='C0', s=100)
    ax.scatter(alligned[0,:], alligned[1,:], marker='x', color='C1', s=100)
    for i in range(len(taglist)):
        ax.text(gt_vec[0,i]+0.05, gt_vec[1,i]+0.05, taglist[i], color='C0', size=12)
        ax.text(alligned[0,i]+0.05, alligned[1,i]+0.05, taglist[i], color='C1', size=12)
    plt.title('Arena')
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_xticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    ax.set_yticks([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    plt.legend(['Real','Pred'])
    plt.grid()
    plt.show()