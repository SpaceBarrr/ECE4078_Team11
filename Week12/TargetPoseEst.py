# estimate the pose of target objects detected
import numpy as np
import json
import os
import ast
import cv2
from scipy.spatial.distance import pdist
import numpy as np
from YOLO.detector import Detector
from sklearn.cluster import KMeans, DBSCAN

# list of target fruits and vegs types
# Make sure the names are the same as the ones used in your YOLO model
TARGET_TYPES = ['Orange', 'Lemon', 'Lime', 'Tomato', 'Capsicum', 'potato', 'Pumpkin', 'Garlic']


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
    target_dimensions_dict = {'Orange': [0.077,0.078,0.074], 'Lemon': [0.070,0.051,0.053], # SWAP BACK FOR FAKE FRUIT 'Orange': [0.077,0.078,0.074], 'Lemon': [0.070,0.051,0.053], REAL Fruit : 'Orange': [0.0963,0.0892,0.0871], 'Lemon': [0.0651,0.091,0.0642]
                              'Lime': [0.073,0.053,0.051], 'Tomato': [0.072,0.073,0.062], 
                              'Capsicum': [0.079,0.076,0.097], 'potato': [0.095,0.060,0.067], 
                              'Pumpkin': [0.087,0.085,0.075],  'Garlic': [0.064,0.061,0.073]} # SWAP BACK FOR FAKE FRUIT 'Garlic': [0.064,0.061,0.073], real fruit : 'Garlic': [0.057,0.056,0.052]
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
    image_width = 320 # change this if your training image is in a different size (check details of pred_0.png taken by your robot)
    x_shift = image_width/2 - pixel_center              # x distance between bounding box centre and centreline in camera view
    theta = np.arctan(x_shift/focal_length)     # angle of object relative to the robot
    ang = theta + robot_pose[2]     # angle of object in the world frame
    
   # relative object location
    distance_obj = distance/np.cos(theta) # relative distance between robot and object
    x_relative = distance_obj * np.cos(theta) # relative x pose
    y_relative = distance_obj * np.sin(theta) # relative y pose
    relative_pose = {'x': x_relative, 'y': y_relative}
    #print(f'relative_pose: {relative_pose}')

    # location of object in the world frame using rotation matrix
    delta_x_world = x_relative * np.cos(robot_pose[2]) - y_relative * np.sin(robot_pose[2])
    delta_y_world = x_relative * np.sin(robot_pose[2]) + y_relative * np.cos(robot_pose[2])
    # add robot pose with delta target pose
    target_pose = {'y': (robot_pose[1]+delta_y_world)[0],
                   'x': (robot_pose[0]+delta_x_world)[0]}
    #print(f'delta_x_world: {delta_x_world}, delta_y_world: {delta_y_world}')
    #print(f'target_pose: {target_pose}')

    return target_pose

def merge_estimations(target_pose_dict: dict, threshold: float) -> dict:
    """
    function:
        merge estimations of the same target
    input:
        target_pose_dict: dict, generated by estimate_pose
        threshold: passed as a command line argument, default 0.25, distance at which 2 estimations are considered different fruits
    output:
        target_est: dict, target pose estimations after merging
    """
    target_est = {}

    ######### Replace with your codes #########
    # TODO: replace it with a solution to merge the multiple occurrences of the same class type (e.g., by a distance threshold)
    fruits_temp = {}

    for fruit in TARGET_TYPES:
        # create a new dict containing the points of each fruit, and their centroid
        total = 0
        sum_x = 0
        sum_y = 0
        fruits_temp[fruit] = {}
        fruits_temp[fruit]["points_x"] = []
        fruits_temp[fruit]["points_y"] = []
        for fruit_from_dict in target_pose_dict:
            if fruit_from_dict.split("_")[0].lower() == fruit.lower():
                sum_x += target_pose_dict[fruit_from_dict]['x']
                sum_y += target_pose_dict[fruit_from_dict]['y']
                total += 1
                fruits_temp[fruit]["points_x"].append(target_pose_dict[fruit_from_dict]['x'])
                fruits_temp[fruit]["points_y"].append(target_pose_dict[fruit_from_dict]['y'])
        fruits_temp[fruit]["centroid_x"] = sum_x / total
        fruits_temp[fruit]["centroid_y"] = sum_y / total
        
        # outlier rejection
        # clustering_x = DBSCAN(eps=0.3, min_samples=3).fit(fruits_temp[fruit]["points_x"])
        # clustering_y = DBSCAN(eps=0.3, min_samples=3).fit(fruits_temp[fruit]["points_y"])
        # for index, x in enumerate(clustering_x.labels_):
        #     if x == -1:
        #         fruits_temp[fruit]["points_x"].pop(index)
        #         fruits_temp[fruit]["points_y"].pop(index)
        # for index, y in enumerate(clustering_y.labels_):
        #     if y == -1:
        #         fruits_temp[fruit]["points_x"].pop(index)
        #         fruits_temp[fruit]["points_y"].pop(index)
        
        # determine if there is 1 or 2 fruits
        # 10cm threshold
        fruits_temp[fruit]["all_points"] = np.vstack((fruits_temp[fruit]["points_x"], fruits_temp[fruit]["points_y"])).T
        fruits_temp[fruit]["average_dist"] = np.mean(pdist(fruits_temp[fruit]["all_points"]))
        fruits_temp[fruit]["clusters"] = 2 if fruits_temp[fruit]["average_dist"] > 0.25 else 1
        
    # calculate kmeans
    for fruit in fruits_temp:
        target_est[f"{fruit.lower()}_0"] = {}
        if fruits_temp[fruit]["clusters"] == 2:
            kmeans = KMeans(n_clusters=2, random_state=0, n_init="auto").fit(fruits_temp[fruit]["all_points"])      
            centrepoints = kmeans.cluster_centers_
            target_est[f"{fruit.lower()}_1"] = {}
            target_est[f"{fruit.lower()}_0"]["y"] = centrepoints[0][1]
            target_est[f"{fruit.lower()}_0"]["x"] = centrepoints[0][0]
            target_est[f"{fruit.lower()}_1"]["y"] = centrepoints[1][1]
            target_est[f"{fruit.lower()}_1"]["x"] = centrepoints[1][0]
        else:
            kmeans = KMeans(n_clusters=1, random_state=0, n_init="auto").fit(fruits_temp[fruit]["all_points"]) 
            centrepoints = kmeans.cluster_centers_
            target_est[f"{fruit.lower()}_0"]["y"] = centrepoints[0][1]
            target_est[f"{fruit.lower()}_0"]["x"] = centrepoints[0][0]
            
    return target_est

def parse_slam_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            if tag > 0 and tag < 11:
                aruco_dict[f"aruco_{tag}"] = {
                    "x": usr_dict["map"][0][i],
                    "y": usr_dict["map"][1][i]
                }
    return aruco_dict

# main loop
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--run", metavar='', type=str, default="NOT SUPPLIED")
    parser.add_argument("--threshold", metavar='', type=float, default=0.25)
    args, _ = parser.parse_known_args()

    if args.run == "NOT SUPPLIED":
        raise ValueError("YOU HAVE NOT SUPPLIED A RUN NUMBER. Please use --run N\nDONT PANIC THIS IS NOT A CODE ISSUE!!!")

    script_dir = os.path.dirname(os.path.abspath(__file__))     # get current script directory (TargetPoseEst.py)

    # read in camera matrix
    fileK = f'{script_dir}/calibration/param/intrinsic.txt'
    camera_matrix = np.loadtxt(fileK, delimiter=',')

    # init YOLO model
    model_path = f'{script_dir}/YOLO/model/yolov8_model.pt'
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
        input_image = cv2.imread(f"{script_dir}/{image_path}")
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
    target_est = merge_estimations(target_pose_dict, args.threshold)
    print(target_est)

    with open(f'{script_dir}/lab_output/targets_run{args.run}_411.txt', 'w') as fo:
        json.dump(target_est, fo, indent=4)
        
    try:
        aruco_dict = parse_slam_map(f'{script_dir}/lab_output/slam.txt')
        os.rename(f'{script_dir}/lab_output/slam.txt', f'{script_dir}/lab_output/slam_run{args.run}_411.txt')
    except FileNotFoundError:
        print("WARNING: Could not find slam.txt - File will not be renamed and truemap will not be created!!!\nMost likely you either forgot to save slam map or have already ran TargetPoseEst.py")
        
    print('Estimations saved!')
    
    aruco_dict.update(target_est)
    with open("TrueMap.txt", "w") as outfile: 
        json.dump(aruco_dict, outfile)

    print("Preprocessing complete.")