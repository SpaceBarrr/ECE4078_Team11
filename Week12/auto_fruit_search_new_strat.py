# teleoperate the robot and perform SLAM
# will be extended in following milestones for system integration

# basic python packages
import numpy as np
import cv2 
import os, sys
import time
import json

# import utility functions
#sys.path.insert(0, "{}/util".format(os.getcwd()))
from util.pibot import PenguinPi # access the robot
import util.DatasetHandler as dh # save/load functions
import util.measure as measure # measurements
import pygame # python package for GUI
import shutil # python package for file operations

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf_fixed import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

from YOLO.detector import Detector

# Astar imports
import astar

from txt_to_imagev2 import *
import pygamemapgui566

def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 5 target fruits&vegs to search for

    @param fname: filename of the map
    @return:
        1) list of targets, e.g. ['lemon', 'tomato', 'garlic']
        2) locations of the targets, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 4)
            y = np.round(gt_dict[key]['y'], 4)

            if key.startswith('aruco'):
                if key.startswith('aruco10') or key.startswith('aruco_10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5]) - 1
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


def read_search_list(shopping_list):
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open(shopping_list, 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """
    fruit_goal_list = []
    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(len(fruit_list)): # there are 5 targets amongst 10 objects
            if fruit.lower() == fruit_list[i].lower():
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
                fruit_goal_list.append([fruit_true_pos[i][0],fruit_true_pos[i][1]])
        n_fruit += 1
    return fruit_goal_list    

def clamp_angle(rad_angle=0, min_value=-np.pi, max_value=np.pi):
	"""
	Restrict angle to the range [min, max]
	:param rad_angle: angle in radians
	:param min_value: min angle value
	:param max_value: max angle value
	"""

	if min_value > 0:
		min_value *= -1

	angle = (rad_angle + max_value) % (2 * np.pi) + min_value

	return angle

class Operate:
    def __init__(self, args, aruco_true_pos):
        self.aruco_true_pos = aruco_true_pos
                
        self.robot_pose = list()
        self.driving_forward = False
        self.cur_waypoint = list()
        self.all_waypoints = list()
        self.minimum_seen_distance = np.inf
        self.last_3_dist = [np.inf, np.inf, np.inf]
        self.drive_iterations = 0
        self.turn_to_aruco = False
        self.turning_tick = 5
        self.tick = 10
        self.start_turn = 0
        self.initial_theta_diff = 0
        self.initial_robot_pose_theta = 0
        self.closestAruco = []
        self.closestArucoIndex = 1
        self.fruit_to_find_xy = []
        self.simplified_path = []
        self.reached_waypoint = 0
        self.reached_fruit = 0
        
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
            self.pibot = PenguinPi(args.ip, args.port)

        # initialise SLAM parameters
        self.ekf = self.init_ekf(args.calib_dir, args.ip, self.aruco_true_pos)
        self.aruco_det = aruco.aruco_detector(
            self.ekf.robot, marker_length=0.07)  # size of the ARUCO markers

        if args.save_data:
            self.data = dh.DatasetWriter('record')
        else:
            self.data = None
        self.output = dh.OutputWriter('lab_output',auto=True)
        self.command = {'motion': [0, 0],
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
        self.pred_notifier = False
        # a 5min timer
        self.count_down = 300
        self.start_time = time.time()
        self.control_clock = time.time()
        # initialise images
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.detector_output = np.zeros([240, 320], dtype=np.uint8)
        if args.yolo_model == "":
            self.detector = None
            self.yolo_vis = cv2.imread('pics/8bit/detector_splash.png')
        else:
            self.detector = Detector(args.yolo_model)
            self.yolo_vis = np.ones((240, 320, 3)) * 100
        self.bg = pygame.image.load('pics/gui_mask.jpg')

    # wheel control
    def control(self):
        if args.play_data:
            lv, rv = self.pibot.set_velocity()
        else:
            lv, rv = self.pibot.set_velocity(
                self.command['motion'],tick=operate.tick,
                turning_tick=operate.turning_tick) # slow down the robot
        if self.data is not None:
            self.data.write_keyboard(lv, rv)
        dt = time.time() - self.control_clock
        # running in sim
        if args.ip == 'localhost':
            drive_meas = measure.Drive(lv, rv, dt)
        # running on physical robot (right wheel reversed)
        else:
            drive_meas = measure.Drive(lv, -rv, dt)
        self.control_clock = time.time()
        return drive_meas

    # camera control
    def take_pic(self):
        self.img = self.pibot.get_image()

        if self.data is not None:
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
        elif self.ekf_on:  # and not self.debug_flag:
            self.ekf.predict(drive_meas)
            self.ekf.add_landmarks(lms)
            self.ekf.update(lms)

    # using computer vision to detect targets
    def detect_target(self):
        if self.command['inference'] and self.detector is not None:
            # need to convert the colour before passing to YOLO
            yolo_input_img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)

            self.detector_output, self.yolo_vis = self.detector.detect_single_image(yolo_input_img)

            # covert the colour back for display purpose
            self.yolo_vis = cv2.cvtColor(self.yolo_vis, cv2.COLOR_RGB2BGR)

            # self.command['inference'] = False     # uncomment this if you do not want to continuously predict
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
    def init_ekf(self, datadir, ip, aruco_true_pos):
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
        return EKF(robot, aruco_true_pos)           # Bryan changed this

    # save SLAM map
    def record_data(self):
        if self.command['output']:
            self.output.write_map(self.ekf)
            self.notification = 'Map is saved'
            self.command['output'] = False
        # save inference with the matching robot pose and detector labels
        if self.command['save_inference']:
            if self.file_output is not None:
                # image = cv2.cvtColor(self.file_output[0], cv2.COLOR_RGB2BGR)
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
        ekf_view = self.ekf.draw_slam_state(res=(320, 480 + v_pad),
                                            not_pause=self.ekf_on)
        canvas.blit(ekf_view, (2 * h_pad + 320, v_pad))
        robot_view = cv2.resize(self.aruco_img, (320, 240))
        self.draw_pygame_window(canvas, robot_view,
                                position=(h_pad, v_pad)
                                )

        # for target detector (M3)
        detector_view = cv2.resize(self.yolo_vis, (320, 240), cv2.INTER_NEAREST)
        self.draw_pygame_window(canvas, detector_view,
                                position=(h_pad, 240 + 2 * v_pad)
                                )

        # canvas.blit(self.gui_mask, (0, 0))
        self.put_caption(canvas, caption='SLAM', position=(2 * h_pad + 320, v_pad))
        self.put_caption(canvas, caption='Detector',
                         position=(h_pad, 240 + 2 * v_pad))
        self.put_caption(canvas, caption='PiBot Cam', position=(h_pad, v_pad))

        notifiation = TEXT_FONT.render(self.notification,
                                       False, text_colour)
        canvas.blit(notifiation, (h_pad + 10, 596))

        time_remain = self.count_down - time.time() + self.start_time
        if time_remain > 0:
            time_remain = f'Count Down: {time_remain:03.0f}s'
        elif int(time_remain) % 2 == 0:
            time_remain = "Time Is Up !!!"
        else:
            time_remain = ""
        count_down_surface = TEXT_FONT.render(time_remain, False, (50, 50, 50))
        canvas.blit(count_down_surface, (2 * h_pad + 320 + 5, 530))
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
        canvas.blit(caption_surface, (position[0], position[1] - 25))

    # keyboard teleoperation, replace with your M1 codes if preferred        
    def update_keyboard(self):
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONUP:
                # continue
                add_waypoint_from_click(pygame.mouse.get_pos())
                self.start_turn = 1
            if event.type == pygame.KEYDOWN:
                # drive forward
                if event.key == pygame.K_UP:
                    self.command['motion'] = [3, 0]
                # drive backward
                elif event.key == pygame.K_DOWN:
                    self.command['motion'] = [-3, 0]
                # turn left
                elif event.key == pygame.K_LEFT:
                    self.command['motion'] = [0, 2]
                # drive right
                elif event.key == pygame.K_RIGHT:
                    self.command['motion'] = [0, -2]
                # boost key
                if event.key == pygame.K_x:
                    self.command['motion'] = [x * 3 for x in self.command['motion']]
                    self.notification = 'Boost ACTIVATED!'
            if event.type == pygame.KEYUP:
                # stop driving on key release
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN or event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    self.command['motion'] = [0, 0]
                elif event.key == pygame.K_x:
                    self.command['motion'] = [int(x * 1/3) for x in self.command['motion']]
                    self.notification = 'Boost DEACTIVATED!'
            ####################################################
            # stop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                self.command['motion'] = [0, 0]
            # save image
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_i:
                self.command['save_image'] = True
            # save SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                self.command['output'] = True
            # reset SLAM map
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                if self.double_reset_comfirm == 0:
                    self.notification = 'Press again to confirm CLEAR MAP'
                    self.double_reset_comfirm += 1
                elif self.double_reset_comfirm == 1:
                    self.notification = 'SLAM Map is cleared'
                    self.double_reset_comfirm = 0
                    self.ekf.reset()
            # run SLAM
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            #     n_observed_markers = len(self.ekf.taglist)
            #     if n_observed_markers == 0:
            #         if not self.ekf_on:
            #             self.notification = 'SLAM is running'
            #             self.ekf_on = True
            #         else:
            #             self.notification = '> 2 landmarks is required for pausing'
            #     elif n_observed_markers < 3:
            #         self.notification = '> 2 landmarks is required for pausing'
            #     else:
            #         if not self.ekf_on:
            #             self.request_recover_robot = True
            #         self.ekf_on = not self.ekf_on
            #         if self.ekf_on:
            #             self.notification = 'SLAM is running'
            #         else:
            #             self.notification = 'SLAM is paused'
            # run object detector
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                self.command['inference'] = True
            # save object detection outputs
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_n:
                self.command['save_inference'] = True
            # quit
            elif event.type == pygame.QUIT:
                self.quit = True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.quit = True
        if self.quit:
            pygame.quit()
            sys.exit()
        
def add_waypoint_from_click(mouse_pos: tuple):
    map_size_px = 868 
    x_offset = 774 + map_size_px/2
    y_offset = 72 + map_size_px/2
    x_scaling = 3/map_size_px
    y_scaling = 3/map_size_px
    
    x = (mouse_pos[0] - x_offset)  * x_scaling
    y = (mouse_pos[1] - y_offset)  * y_scaling
    
    print(x,-y)

    # robot drives to the waypoint
    operate.cur_waypoint = [x, -y]
            
def drive(aruco_true_pos, initial = 0):
    '''
    Main logic for driving the robot
    Runs in the main pygame loop each clock cycle
    
    State machine logic:
    (1): Initially, turn to face the waypoint until the angle error is smaller than the threshold
    (2): Then, drive straight to the waypoint
    (3): NOT IMPLEMENTED: If the angle begins to increase, we can turn again to correct
    (4): If the distance to the waypoint begins to increase, arrive early & go next
    '''
    # TUNEABLE PARAMS:
    ANGLE_THRESHOLD = 0.05 # rad, 0.5 ~ 3 deg
    LINEAR_THRESHOLD = 0.2
    LINEAR_FUDGE_FACTOR = 0.1      # Woodside : 0.15 
    TURNING_SCALING = 10               # Woodside : 10
    TURNING_CONST = 20                  # Woodside : 20
    FORWARD_SCALING = 10                # Woodside : 50
    FORWARD_CONST = 20                  # Woodside : 20
    
    # user either hasn't clicked or path planning returned nothing
    if len(operate.cur_waypoint) == 0:
        return  
    
    # grab pose data and calculate angles
    robot_x = operate.robot_pose[0]
    robot_y = -operate.robot_pose[1]
    robot_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
    # print(robot_theta)
    waypoint_x = operate.cur_waypoint[0]
    waypoint_y = -operate.cur_waypoint[1]

    if initial == 1:
        robot_x = 0
        robot_y = 0 
    
    waypoint_theta = np.arctan2((waypoint_y-robot_y),(waypoint_x-robot_x))
    theta_diff = clamp_angle(robot_theta - waypoint_theta, -np.pi, np.pi)
    # print(f"Robot Pose : [{robot_x}, {robot_y}, {robot_theta}]")
    # print(f"Waypoint : [{waypoint_x}, {waypoint_y}]")
    # print(f"Theta diff : {theta_diff}")
    # print(f"operate.reached_waypoint = {operate.reached_waypoint}")
        
    # TURNING TO WAYPOINT
    if (not operate.driving_forward) and (not operate.turn_to_aruco):
        # operate.turning_tick = int(np.round(abs(theta_diff * TURNING_SCALING) + TURNING_CONST))
        print("Turning to Waypoint ...")
        print(f"operate tick when turning : {operate.turning_tick}")
        operate.turning_tick = 15
        # operate.tick = 10

        if theta_diff > 0: # turn right
            operate.command['motion'] = [0,1]
        elif theta_diff < 0: # turn left
            operate.command['motion'] = [0,-1]
        elif theta_diff == 0 : # should be impossible to end up in this situation
            operate.command['motion'] = [0,0]

        if operate.start_turn == 1 : 
            operate.initial_theta_diff = theta_diff
            operate.initial_robot_pose_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
            operate.start_turn = 0
        
        if abs(theta_diff) < ANGLE_THRESHOLD: # close enough, stop turning
            # operate.turning_tick = 5
            print("Finished turning to waypoint...")
            operate.command['motion'] = [0,0]
            operate.minimum_seen_distance = np.sqrt((waypoint_x-robot_x)**2 + (waypoint_y-robot_y)**2)
            operate.driving_forward = True
    
    # DRIVING FORWARD TO WAYPOINT
    if operate.driving_forward: 
        # operate.tick = 20
        new_distance = np.sqrt((waypoint_x-robot_x)**2 + (waypoint_y-robot_y)**2)
        operate.tick = int(np.round(new_distance * FORWARD_SCALING + FORWARD_CONST )  )             # Used for P tuning
        print(f"        operate tick when driving : {operate.tick}")

        # the below is basically a scuffed implementation of moving avg
        # TODO: is there a better of doing this?
        # np.roll will shift the last item of the array to the front
        # [D, A, B, C] = np.roll([A, B, C, D])
        operate.last_3_dist = np.roll(operate.last_5_dist, 1)
        operate.last_3_dist[0] = new_distance # replace the oldest value (now at the front) with the new distance
        moving_avg_dist = np.mean(operate.last_5_dist)
        
        if (moving_avg_dist > (operate.minimum_seen_distance + LINEAR_FUDGE_FACTOR) and operate.drive_iterations > 10): # distance increasing scenario (add some fudge factor for SLAM noise)
            operate.command['motion'] = [0,0]
            operate.drive_iterations = 0
            operate.minimum_seen_distance = np.inf
            operate.driving_forward = False
            operate.turn_to_aruco = True
            # operate.reached_waypoint = True
            
            # operate.closestAruco, operate.closestArucoIndex = finding_nearest_aruco(operate.cur_waypoint, aruco_true_pos, (operate.initial_robot_pose_theta+operate.initial_theta_diff))
            # try:
            #     new_waypoint = operate.all_waypoints.pop()
            #     operate.cur_waypoint = new_waypoint
            # except IndexError:
            #     print("last waypoint")
            #     operate.cur_waypoint = []
            
            print("Distance increasing, arriving at waypoint early...")
            
        else: # distance decreasing (good)
            print(f"DISTANCE: {moving_avg_dist}, THRESHOLD: {LINEAR_THRESHOLD}")
            if moving_avg_dist < LINEAR_THRESHOLD: # arrived at waypoint
                # reset for next run
                operate.command['motion'] = [0,0]
                operate.minimum_seen_distance = np.inf
                operate.drive_iterations = 0
                operate.driving_forward = False
                #  theta_diff = angle_aruco(operate.cur_waypoint, aruco_true_pos)          ########
                operate.turn_to_aruco = True
                # operate.reached_waypoint = True
                # print(operate.reached_waypoint)
                # operate.turning_tick = 5
                # operate.tick = 10
                # operate.closestAruco, operate.closestArucoIndex = finding_nearest_aruco(operate.cur_waypoint, aruco_true_pos,(operate.initial_robot_pose_theta+operate.initial_theta_diff))
                print("Arrived at waypoint")

            else: # drive forward
                print("Moving Foward")
                operate.drive_iterations += 1
                if new_distance < operate.minimum_seen_distance:
                    operate.minimum_seen_distance = new_distance 
                print("distance left to move : " + str(new_distance))
                operate.command['motion'] = [1,0]
        
        # TODO: implement logic if angle error has increased too much

    ###### TURNING TO Origin (IF Bryan messes this up, its his fault)
    if (not operate.driving_forward) and (operate.turn_to_aruco) :
        theta_diff, way_point_theta = angle_aruco(operate.cur_waypoint, [0,0], robot_theta)
        operate.turning_tick = 15
        print(f"Turning to Origin")
        # print("     waypoint_theta : " + str(way_point_theta))
        print("     theta_diff : " + str(theta_diff))
        #operate.turning_tick = int(np.round(abs(theta_diff * TURNING_SCALING) + TURNING_CONST))
        print(f"operate tick when turning : {operate.turning_tick}")

        if theta_diff > 0: # turn right
            operate.command['motion'] = [0,1]
            operate.turning_tick = 20
        elif theta_diff < 0: # turn left
            operate.command['motion'] = [0,-1]
            operate.turning_tick = 20
        elif theta_diff == 0 : # should be impossible to end up in this situation
            operate.command['motion'] = [0,0]
            operate.turn_to_aruco = False
            operate.reached_waypoint = True
    
        if abs(theta_diff) < ANGLE_THRESHOLD: # close enough, stop turning
            print(f"Finished turning to Aruco")
            operate.command['motion'] = [0,0]
            operate.turn_to_aruco = False
            operate.reached_waypoint = True
            time.sleep(0.5)

    # pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], operate.robot_pose[2], map_image, pibot, operate.simplified_path)

# Added the turn_360_deg
def turn_360_deg(threshold, map_image, pibot) : 
    # grab pose data and calculate angles
    original_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
    robot_x = operate.robot_pose[0]
    robot_y = -operate.robot_pose[1]

    waypoint_theta = original_theta + np.pi
    theta_diff = np.pi 
    operate.turning_tick = 20
    n = 0

    while (theta_diff > threshold) : 
        operate.take_pic()
        operate.command['motion'] = [0,-1]
        pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.robot_pose = operate.ekf.robot.state[:3,0]
        # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
        # print(operate.robot_pose)
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        robot_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
        theta_diff = abs(clamp_angle((robot_theta - waypoint_theta), -np.pi, np.pi))
        print(theta_diff)
        n += 1
        if (n > 10) : 
            n = 0
            operate.command['motion'] = [0,0]
            operate.take_pic()
            pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
            drive_meas = operate.control()
            operate.update_slam(drive_meas)
            operate.robot_pose = operate.ekf.robot.state[:3,0]
            operate.record_data()
            operate.save_image()
            operate.detect_target()
            # visualise
            operate.draw(canvas)
            pygame.display.update()
            time.sleep(1)

    print("Turned halfway...")
    operate.command['motion'] = [0,0]
    back_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
    theta_diff = np.pi
    n = 0

    while (theta_diff > threshold) : 
        operate.take_pic()
        operate.command['motion'] = [0,-1]
        print(theta_diff)
        pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.robot_pose = operate.ekf.robot.state[:3,0]
        # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
        # print(operate.robot_pose)
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        robot_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
        theta_diff = abs(robot_theta - original_theta)
        n += 1
        if (n > 10) : 
            n = 0
            operate.command['motion'] = [0,0]
            operate.take_pic()
            pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
            drive_meas = operate.control()
            operate.update_slam(drive_meas)
            operate.robot_pose = operate.ekf.robot.state[:3,0]
            operate.record_data()
            operate.save_image()
            operate.detect_target()
            # visualise
            operate.draw(canvas)
            pygame.display.update()
            time.sleep(1)
        

    print("Finished turning")
    operate.command['motion'] = [0,0]
    drive_meas = operate.control()
    operate.update_slam(drive_meas)
    operate.robot_pose = operate.ekf.robot.state[:3,0]
    # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
    # print(operate.robot_pose)
    operate.record_data()
    operate.save_image()
    operate.detect_target()
    # visualise
    operate.draw(canvas)

def finding_nearest_aruco(waypoint, aruco_true_pos, robot_theta) : 
    closest_aruco = aruco_true_pos[0]
    print(waypoint)
    distance_to_closest = np.sqrt((robot_x-closest_aruco[0])**2+(robot_y-closest_aruco[1])**2)      # Change this to neg
    index_aruco = 1
    print(f"robot theta : {robot_theta}")
    print("distance to closest" + str(distance_to_closest))
    #  print(f"Now facing to Aruco Marker {index_aruco}")

    for i in range(10) : 
        distance_to_aruco = np.sqrt((waypoint[0]-aruco_true_pos[i][0])**2+(waypoint[1]-aruco_true_pos[i][1])**2)
        # print("distance to aruco : " + str(distance_to_aruco))
        # print("distance to closest" + str(distance_to_closest))
        if distance_to_aruco < distance_to_closest :
            closest_aruco = aruco_true_pos[i]
            distance_to_closest = distance_to_aruco
            index_aruco = i + 1
    
    print(f"Now turning to ArucoMarker {index_aruco}")
    return closest_aruco, index_aruco

def drive_to_waypoint(obstacle_list, waypoint, aruco_true_pos,robot_pose,map_image, pibot, canvas) :
    waypoint_x = waypoint[0]
    waypoint_y = waypoint[1]
    operate.cur_waypoint = [waypoint_x, waypoint_y]

    while not operate.reached_waypoint:
        # operate.update_keyboard()
        operate.take_pic()
        drive(aruco_true_pos)
        pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.robot_pose = operate.ekf.robot.state[:3,0]
        # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
        # print(operate.robot_pose)
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        pygame.display.update()
    #this should delete current waypoint from map?
    # pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], (operate.robot_pose[2] - np.pi/2), map_image, pibot, operate.simplified_path)
    # operate.draw(canvas)
    # pygame.display.update()

        
def angle_aruco(waypoint, closest_aruco, robot_theta) : 
    y_diff = closest_aruco[1] - waypoint[1]
    x_diff = closest_aruco[0] - waypoint[0]

    theta_dif = clamp_angle(np.arctan2(y_diff, x_diff) + robot_theta,-np.pi,np.pi)      # Change this
    way_point_theta = clamp_angle(np.arctan2(y_diff, x_diff),-np.pi,np.pi)

    return theta_dif, way_point_theta

def turn_to_aruco(aruco_true_pos) : 
    # TUNEABLE PARAMS:
    ANGLE_THRESHOLD = 0.05 # rad, 0.5 ~ 3 deg
    TURNING_SCALING = 10               # Woodside : 10
    TURNING_CONST = 20                  # Woodside : 20

    robot_theta = clamp_angle(-operate.robot_pose[2], -np.pi, np.pi)
    
    # user either hasn't clicked or path planning returned nothing
    if len(operate.cur_waypoint) == 0:
        return  
    
    ###### TURNING TO ARUCO (IF Bryan messes this up, its his fault)
    if (not operate.driving_forward) and (operate.turn_to_aruco) :
        theta_diff, way_point_theta = angle_aruco(operate.cur_waypoint, [operate.closestAruco[0], operate.closestAruco[1]], robot_theta)
        operate.turning_tick = 45
        print(f"Turning to point [{operate.closestAruco[0]},{operate.closestAruco[1]}], Aruco {operate.closestArucoIndex}")
        # print("     waypoint_theta : " + str(way_point_theta))
        print("     theta_diff : " + str(theta_diff))
        operate.turning_tick = int(np.round(abs(theta_diff * TURNING_SCALING) + TURNING_CONST))
        print(f"operate tick when turning : {operate.turning_tick}")

        if theta_diff > 0: # turn right
            operate.command['motion'] = [0,1]
            operate.turning_tick = 20
        elif theta_diff < 0: # turn left
            operate.command['motion'] = [0,-1]
            operate.turning_tick = 20
        elif theta_diff == 0 : # should be impossible to end up in this situation
            operate.command['motion'] = [0,0]
            operate.turn_to_aruco = False
            operate.reached_waypoint = True
    
        if abs(theta_diff) < ANGLE_THRESHOLD: # close enough, stop turning
            print(f"Finished turning to Aruco")
            operate.command['motion'] = [0,0]
            operate.turn_to_aruco = False
            operate.reached_waypoint = True

def initial_turn_to_nearest_aruco(aruco_true_pos) : 
    operate.closestAruco, operate.closestArucoIndex = finding_nearest_aruco([0,0], aruco_true_pos, 0)
    operate.turn_to_aruco = True
    operate.driving_forward = False
    operate.cur_waypoint = [0, 0]
    operate.robot_pose = [0,0,0]

    while operate.turn_to_aruco == True : 
        # operate.update_keyboard()
        operate.take_pic()
        turn_to_aruco(aruco_true_pos)
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.robot_pose = operate.ekf.robot.state[:3,0]
        # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
        # print(operate.robot_pose)
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        pygame.display.update()

    time.sleep(1)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='192.168.50.1') # localhost
    parser.add_argument("--port", metavar='', type=int, default=8080) # 40000
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--map", type=str, default="TrueMap1.txt")
    parser.add_argument("--yolo_model", default='YOLO/model/yolov8_model.pt')
    parser.add_argument("--shopping_list", type=str, default="shopping_list.txt")
    parser.add_argument("--auto", type=int, default=1)
    parser.add_argument("--run", type=str, default="NOT SUPPLIED")
    args, _ = parser.parse_known_args()
    
    if args.run == "NOT SUPPLIED":
        raise ValueError("YOU HAVE NOT SUPPLIED A RUN NUMBER. Please use --run N\nDONT PANIC THIS IS NOT A CODE ISSUE!!!")

    true_map = f"TrueMap{args.run}.txt"
    
    pygame.font.init() 
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)
    
    width, height = 1266, 660  #original size 700, 660; map is 1100,660 for 400px map, 1266,660 for 566px map
    canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption('ECE4078 Lab')
    pygame.display.set_icon(pygame.image.load('pics/8bit/pibot5.png'))
    canvas.fill((0, 0, 0))
    splash = pygame.image.load('pics/loading.png')
    pibot_animate = [pygame.image.load('pics/8bit/pibot1.png'),
                     pygame.image.load('pics/8bit/pibot2.png'),
                     pygame.image.load('pics/8bit/pibot3.png'),
                     pygame.image.load('pics/8bit/pibot4.png'),
                     pygame.image.load('pics/8bit/pibot5.png')]
    pygame.display.update()
    # ========================================================================
    # create map_image.png from text file
    visualise_map(true_map)
    # params for gui
    background_colour = (45,45,45)
    rect_colour = (128,128,128)
    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    orange = (245, 117, 20)
    PIBOT_WIDTH = (218/1.5)*0.1
    PIBOT_HEIGHT = PIBOT_WIDTH

    # drawing map_image rectangle
    map_background_rect = pygame.Rect(700, 0, 566, 660)

    pygame.draw.rect(canvas,rect_colour,map_background_rect)
    # resizing map_image and drawing on the canvas
    map_image = pygame.image.load('map_imagev2.png')
    pibot = pygame.image.load('guipngs/pibot_top.png')
    pibot = pygame.transform.scale(pibot, (PIBOT_WIDTH, PIBOT_HEIGHT))
    map_image = pygamemapgui566.initialise_map(canvas, map_image)
    
    # ========================================================================
    start = False

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(true_map)
    # print(aruco_true_pos)

    search_list = read_search_list(args.shopping_list)
    fruit_goal_list = print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)
    
    obstacle_list = np.vstack((fruits_true_pos, aruco_true_pos))
    operate = Operate(args, aruco_true_pos)

    operate.ekf_on = True
    ppi = PenguinPi(args.ip,args.port)
    operate.command['motion'] = [0,0]
    
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

    if args.auto:
        ### for Autonomous Waypoints (COMMENT THIS OUT : Do not use thiss until its time to test)
        try:
            last_fruit_index = -1
            for K in range(5):
                last_fruit_pos = None
                
                fruit_to_find = search_list.pop(0)
                fruit_index = fruits_list.index(fruit_to_find.lower())
                print(f"Now Heading to Fruit number {fruit_index} : " + fruit_to_find)
                operate.fruit_to_find_xy = fruits_true_pos[fruit_index]
                fruits_true_pos = np.delete(fruits_true_pos, fruit_index, axis=0)        # Removes the finding fruit from obstacle list
                obstacle_list = np.vstack((fruits_true_pos, aruco_true_pos))

                STARTING_RADIUS = 1.2 # NOTE tunable param
                STARTING_ROBOT_RADIUS = 0.08

                if K==0 :
                    robot_x = 0
                    robot_y = 0
                
                radius = STARTING_RADIUS
                robot_radius = STARTING_ROBOT_RADIUS
                
                all_waypoints_reverse, simplified_path_reverse = None, None # bc python doesnt have a do while loop :(
                # while all_waypoints_reverse == None and simplified_path_reverse == None:
                #     all_waypoints_reverse, simplified_path_reverse = astar.a_start(robot_x, robot_y, operate.fruit_to_find_xy[0], operate.fruit_to_find_xy[1], obstacle_list, last_fruit_pos, radius)
                #     radius -= 0.3 # if this too small, the algorithm will be quite slow. too large, and we'll take shit paths. 
                #     if radius < 0:
                #         radius = 0
                while all_waypoints_reverse == None and simplified_path_reverse == None:
                    all_waypoints_reverse, simplified_path_reverse = astar.a_start(robot_x, robot_y, operate.fruit_to_find_xy[0], operate.fruit_to_find_xy[1], obstacle_list, last_fruit_pos, radius, robot_radius)
                    robot_radius -= 0.02 # if this too small, the algorithm will be quite slow. too large, and we'll take shit paths. 
                    if robot_radius < 0:
                        robot_radius = 0
                
                # get rid of orgin
                if K== 0 : 
                    simplified_path_reverse = np.delete(simplified_path_reverse, -1, axis=0)

                # Waypoints are reversed, trying to set it right
                operate.all_waypoints = all_waypoints_reverse[::-1]
                operate.simplified_path =  simplified_path_reverse[::-1]

                # Waypoints contain the goal, so we get the goal and the 2nd last waypoint, average them together and take that as our last waypoint
                # operate.all_waypoints = np.delete(operate.all_waypoints, -1, axis=0) 
                goal = []
                goal = operate.simplified_path[-1]
                # operate.simplified_path = np.delete(operate.simplified_path, -1, axis=0) 
                # final_waypoint = operate.simplified_path[-1]
                # final_waypoint = [(final_waypoint[0] + final_waypoint[0]+ goal[0])/3, (final_waypoint[1]+final_waypoint[1]+ goal[1])/3 ]
                # operate.simplified_path = np.vstack((operate.simplified_path, final_waypoint)) 
                
                # Add the finding fruit back to obstacle list for next time
                fruits_true_pos = np.insert(fruits_true_pos, fruit_index, operate.fruit_to_find_xy, axis = 0)        
                obstacle_list = np.vstack((fruits_true_pos, aruco_true_pos))
                print(operate.simplified_path)
                
                # initial_turn_to_nearest_aruco(aruco_true_pos)
                if (K != 0) : 
                    turn_360_deg(0.3, map_image, pibot)

                # Drive there
                for path in operate.simplified_path: 
                    operate.robot_pose = operate.ekf.robot.state[:3,0]
                    operate.reached_waypoint = False
                    # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
                    operate.cur_waypoint = path
                    print(f"Driving to waypoint: {path}")
                    drive_to_waypoint(obstacle_list, path, aruco_true_pos, operate.robot_pose, map_image, pibot, canvas)
                    operate.simplified_path = np.delete(operate.simplified_path,0,0)
                    turn_360_deg(0.3, map_image, pibot)
                    pygamemapgui566.update_gui_map(canvas, operate.robot_pose[0], operate.robot_pose[1], operate.robot_pose[2], map_image, pibot, operate.simplified_path)

                robot_x = operate.robot_pose[0]
                robot_y = operate.robot_pose[1]
                #turn_360_deg(0.3, map_image, pibot)
                operate.notification = f"Arrived at fruit: {fruit_to_find}"

                time.sleep(2)
                ####### Go back to the orgin #######
                all_waypoints_reverse, simplified_path_reverse = None, None # bc python doesnt have a do while loop :(
                while all_waypoints_reverse == None and simplified_path_reverse == None:
                    all_waypoints_reverse, simplified_path_reverse = astar.a_start(robot_x, robot_y, 0, 0, obstacle_list, last_fruit_pos, radius)
                    radius -= 0.3 # if this too small, the algorithm will be quite slow. too large, and we'll take shit paths. 
                    if radius < 0:
                        radius = 0
                
                operate.all_waypoints = all_waypoints_reverse[::-1]
                operate.simplified_path =  simplified_path_reverse[::-1]

                for path in operate.simplified_path: 
                    operate.robot_pose = operate.ekf.robot.state[:3,0]
                    operate.reached_waypoint = False
                    # operate.notification = f"[{operate.robot_pose[0]}, {operate.robot_pose[1]}, {operate.robot_pose[2]}]"
                    operate.cur_waypoint = path
                    print(f"Driving to waypoint: {path}")
                    drive_to_waypoint(obstacle_list, path, aruco_true_pos, operate.robot_pose, map_image, pibot, canvas)
                    operate.simplified_path = np.delete(operate.simplified_path,0,0)
                    turn_360_deg(0.3, map_image, pibot)
                
                operate.notification = f"Arrived at ORIGIN"
                robot_x = operate.robot_pose[0]
                robot_y = operate.robot_pose[1]

                # STOP SPINNING
                operate.command['motion'] = [0,0]
                drive_meas = operate.control()
                operate.update_slam(drive_meas)
                operate.robot_pose = operate.ekf.robot.state[:3,0]
                operate.record_data()
                operate.save_image()
                operate.detect_target()
                last_fruit_pos = operate.fruit_to_find_xy
                time.sleep(2)
                
        except KeyboardInterrupt:
            operate.command['motion'] = [0,0]
            drive_meas = operate.control()
            print("Done")
            raise
        
        ###
        operate.command['motion'] = [0,0]
        drive_meas = operate.control()
        operate.update_slam(drive_meas)
        operate.robot_pose = operate.ekf.robot.state[:3,0]
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        print("Done")

    else: # level 1 code
        while start:
            try:
                operate.update_keyboard()
                operate.take_pic()
                drive(aruco_true_pos)
                drive_meas = operate.control()
                operate.update_slam(drive_meas)
                operate.robot_pose = operate.ekf.robot.state[:3,0]
                # print(operate.robot_pose)
                operate.record_data()
                operate.save_image()
                operate.detect_target()
                # visualise
                operate.draw(canvas)
                pygame.display.update()
            except KeyboardInterrupt:
                operate.command['motion'] = [0,0]
                drive_meas = operate.control()
                print("Done")
                raise
        
    
