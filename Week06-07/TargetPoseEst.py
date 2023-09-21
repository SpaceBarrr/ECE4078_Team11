# estimate the pose of target objects detected
import numpy as np
import json
import os
import ast
import cv2
import re
from YOLO.detector import Detector
from sklearn.cluster import KMeans

# list of target fruits and vegs types
# Make sure the names are the same as the ones used in your YOLO model
TARGET_TYPES = ['Orange', 'Lemon', 'Lime', 'Tomato', 'Capsicum', 'Potato', 'Pumpkin', 'Garlic']


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

def merge_estimations(target_pose_dict):
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
    NUMBER_OF_CLUSTERS = 10
    coord_master = list()

    # KMeans() wants a list of lists (not a dict of dicts), so we convert here
    for key in target_pose_dict:
        coords = list(target_pose_dict[key].values()) # extract the sub values from the array
        coord_master.append(coords)

    # NOTE hardcoding 10 clusters
    kmeans = KMeans(n_clusters=NUMBER_OF_CLUSTERS, random_state=0, n_init="auto").fit(coord_master)      
    centrepoints = kmeans.cluster_centers_
    
    # at this point we have a list of clusters (given by kmeans.labels_), but we don't know which cluster is which fruit
    for fruit_predict in target_pose_dict:
        # ugly array nesting below bc kmeans expects an array in an array
        to_predict_nested = [target_pose_dict[fruit_predict]["y"],
                             target_pose_dict[fruit_predict]["x"]]
        to_predict_final = [to_predict_nested]
        cluster_prediction = kmeans.predict(np.array(to_predict_final))[0] # returns an array of an array, so have to dereference

        fruit = fruit_predict.split("_")[0] # extract the fruit name from the dict key

        if f"{fruit}_0" in target_est: # second 'discovery' of a fruit
            target_est[f"{fruit.lower()}_1"] = {
                "y": centrepoints[cluster_prediction][0],
                "x": centrepoints[cluster_prediction][1]
            }
        else:
            target_est[f"{fruit.lower()}_0"] = {
                "y": centrepoints[cluster_prediction][0],
                "x": centrepoints[cluster_prediction][1]
            }

        if len(list(target_est.keys())) >= NUMBER_OF_CLUSTERS: # once we find all 10 fruits and their centrepoints we don't need to keep searching
            break
    
    #########
   
    return target_est


# main loop
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--run", metavar='', type=str, default="NOT SUPPLIED")
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
    target_est = merge_estimations(target_pose_dict)
    print(target_est)
    # save target pose estimations
    # with open(f'{script_dir}/lab_output/target_pose_dict.txt', 'w') as fo_1 :
    #     json.dump(target_pose_dict, fo_1, indent=4)   # To change

    with open(f'{script_dir}/lab_output/targets_{args.run}.txt', 'w') as fo:
        json.dump(target_est, fo, indent=4)
        
    os.rename(f'{script_dir}/lab_output/slam.txt', f'{script_dir}/lab_output/slam_{args.run}.txt')

    print('Estimations saved!')
