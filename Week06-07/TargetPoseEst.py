# estimate the pose of target objects detected
import numpy as np
import json
import os
import ast
import cv2
from YOLO.detector import Detector


# list of target fruits and vegs types
# Make sure the names are the same as the ones used in your YOLO model
TARGET_TYPES = ['orange', 'lemon', 'lime', 'tomato', 'capsicum', 'potato', 'pumpkin', 'garlic']


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
    target_dimensions_dict = {'orange': [0.077,0.078,0.074], 'lemon': [0.070,0.051,0.053], 
                              'lime': [0.073,0.053,0.051], 'tomato': [0.072,0.073,0.062], 
                              'capsicum': [0.079,0.076,0.097], 'potato': [0.095,0.060,0.067], 
                              'pumpkin': [0.087,0.085,0.075], 'garlic': [0.064,0.061,0.073]}
    #########

    # estimate target pose using bounding box and robot pose
    target_class = obj_info[0]     # get predicted target label of the box
    target_box = obj_info[1]       # get bounding box measures: [x,y,width,height]
    true_height = target_dimensions_dict[target_class][2]   # look up true height of by class label

    # compute pose of the target based on bounding box info, true object height, and robot's pose
    pixel_height = target_box[3]
    pixel_center = target_box[0]
    distance = true_height/pixel_height * focal_length  # estimated distance between the object and the robot based on height
    # image size 640x480 pixels, 640/2=320
    x_shift = 320/2 - pixel_center              # x distance between bounding box centre and centreline in camera view
    theta = np.arctan(x_shift/focal_length)     # angle of object relative to the robot
    horizontal_relative_distance = distance * np.sin(theta)     # relative distance between robot and object on x axis
    vertical_relative_distance = distance * np.cos(theta)       # relative distance between robot and object on y axis
    relative_pose = {'y': vertical_relative_distance, 'x': horizontal_relative_distance}    # relative object location

    ang = theta + robot_pose[2]     # angle of object in the world frame

    # location of object in the world frame
    target_pose = {'y': (robot_pose[1]+relative_pose['y']*np.sin(ang))[0],
                   'x': (robot_pose[0]+relative_pose['x']*np.cos(ang))[0]}

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
    target_est = target_pose_dict
    #########
   
    return target_est


# main loop
if __name__ == "__main__":
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
    print(target_est)
    # save target pose estimations
    with open(f'{script_dir}/lab_output/targets.txt', 'w') as fo:
        json.dump(target_est, fo, indent=4)

    print('Estimations saved!')
