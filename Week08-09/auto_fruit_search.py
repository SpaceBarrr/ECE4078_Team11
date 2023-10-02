# teleoperate the robot and perform SLAM
# will be extended in following milestones for system integration

# basic python packages
import numpy as np
import cv2 
import os, sys
import time
import json
from time import sleep

# import utility functions
#sys.path.insert(0, "{}/util".format(os.getcwd()))
from util.pibot import PenguinPi # access the robot
import util.DatasetHandler as dh # save/load functions
import util.measure as measure # measurements
import pygame # python package for GUI
import shutil # python package for file operations

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

from YOLO.detector import Detector

from rrt import *
from Obstacle import *

from txt_to_image import *

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
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
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

# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# note that this function requires your camera and wheel calibration parameters from M2, and the "util" folder from M1
# fully automatic navigation:
# try developing a path-finding algorithm that produces the waypoints automatically
def drive_to_point(waypoint, robot_pose):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    wheel_vel = 30 # tick
    lin_vel = 1/12               # m/s -> 10.48
    ang_vel = 2*np.pi / 6         # rad/s
    
    robot_pose_x = robot_pose[0]
    robot_pose_y = robot_pose[1]
    robot_pose_theta = robot_pose[2]
    
    # turn towards the waypoint
    x_diff = waypoint[0] - robot_pose_x
    y_diff = waypoint[1] - robot_pose_y

    angle_to_turn = min(np.arctan2(y_diff, x_diff) - robot_pose_theta)

    if angle_to_turn > 0 :              # Change thus
        variable = -1
    elif angle_to_turn < 0 : 
        variable = 1 
    elif angle_to_turn == 0 : 
        variable = 0
    #clamp_angle(, 0, np.pi*2)
    #turn_time = abs(clamp_angle(angle_to_turn, -np.pi , np.pi) / ang_vel) # replace with your calculation
    turn_time = float(abs((baseline*np.pi)/(scale*wheel_vel))*angle_to_turn/(2*np.pi))

    print("Turning for {:.2f} seconds".format(turn_time))
    lv, rv = ppi.set_velocity([0, variable], turning_tick=wheel_vel, time=turn_time)
    drive_meas = measure.Drive(lv, -rv, turn_time)           # Changed
    slam_tings(drive_meas)
    
    # after turning, drive straight to the waypoint
    distance = np.sqrt((waypoint[0]-robot_pose_x)**2+(waypoint[1]-robot_pose_y)**2)
    #drive_time = distance / (lin_vel) # replace with your calculation
    drive_time = float(abs(distance/(scale*wheel_vel)))
    print("Driving for {:.2f} seconds".format(drive_time))
    
    while drive_time > 0:
        if drive_time >= 1:
            lv, rv = ppi.set_velocity([1, 0], turning_tick = 0, tick=wheel_vel, time=1)
            drive_meas = measure.Drive(lv, -rv, drive_time) 
            slam_tings(drive_meas) 
            drive_time -= 1
        else:
            lv, rv = ppi.set_velocity([1, 0], turning_tick = 0, tick=wheel_vel, time=drive_time)
            drive_meas = measure.Drive(lv, -rv, drive_time) 
            slam_tings(drive_meas) 
    
              # Changed

    ####################################################

    print("Arrived at [{}, {}]".format(waypoint[0], waypoint[1]))
    
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
        
        self.modifier = 1
        
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
        self.output = dh.OutputWriter('lab_output')
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
                self.command['motion'],tick=10,
                turning_tick=5) # slow down the robot
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
            # self.ekf.add_landmarks(lms)
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
                add_waypoint_from_click(pygame.mouse.get_pos())
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
            # save SLAM mapƒ
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
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                n_observed_markers = len(self.ekf.taglist)
                if n_observed_markers == 0:
                    if not self.ekf_on:
                        self.notification = 'SLAM is running'
                        self.ekf_on = True
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
    x_offset = 751 + 310/2
    y_offset = 48 + 310/2
    x_scaling = 3/310
    y_scaling = 3/310
    
    x = (mouse_pos[0] - x_offset)  * x_scaling
    y = (mouse_pos[1] - y_offset)  * y_scaling
    
    # print(x,y)

    # estimate the robot's pose
    robot_pose = operate.ekf.get_state_vector()[:3,0]

    # robot drives to the waypoint
    waypoint = [x,y]
    drive_to_point(waypoint,robot_pose)
    robot_pose = operate.ekf.get_state_vector()[:3,0]
    
    print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint, robot_pose))

    # exit
    ppi.set_velocity([0, 0])
    
def add_waypoint_from_rrt(waypoint):
    # estimate the robot's pose
    robot_pose = operate.ekf.get_state_vector()[:3,0]

    # robot drives to the waypoint
    drive_to_point(waypoint,robot_pose)
    robot_pose = operate.ekf.get_state_vector()[:3,0]
    
    print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint, robot_pose))

    # exit
    ppi.set_velocity([0, 0])
    
def rrt_waypoints(goal, start, obstacle_list):
    CIRCLE_RADIUS = 0.2
    
    goal = np.array([goal[0]+1.5, goal[1]+1.5])
    start = np.array([start[0]+1.5, start[1]+1.5])

    all_obstacles_offset = []
    all_obstacles = []
    for i in range(len(obstacle_list)):
        all_obstacles_offset.append(Circle(obstacle_list[i][0]+1.5, obstacle_list[i][1]+1.5, CIRCLE_RADIUS))
        all_obstacles.append(Circle(obstacle_list[i][0], obstacle_list[i][1], CIRCLE_RADIUS))

    rrt = RRT(start=start, goal=goal, width=3, height=3, obstacle_list=all_obstacles_offset,
          expand_dis=1, path_resolution=0.5)

    path_rev = rrt.planning()
    
    waypoints = []
    i = rrt.no_nodes 
    while i > 0 :
        waypoints.append([(path_rev[i][0]-1.5), (path_rev[i][1]-1.5)])
        i -= 1
    
    for j in range(rrt.no_nodes+1) : 
        print(waypoints[j])
        
    return waypoints, all_obstacles
   
def slam_tings(drive_meas):
    operate.take_pic()
    # drive_meas = operate.control()
    operate.update_slam(drive_meas)
    operate.record_data()
    operate.save_image()
    # operate.detect_target()
        
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='192.168.50.1') # localhost
    parser.add_argument("--port", metavar='', type=int, default=8080) # 40000
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--map", type=str, default="TrueMap.txt")
    parser.add_argument("--yolo_model", default='YOLO/model/yolov8_model.pt')
    parser.add_argument("--shopping_list", type=str, default="M4_prac_shopping_list.txt")
    args, _ = parser.parse_known_args()
    
    pygame.font.init() 
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)
    
    width, height = 1100, 660 #original size 700, 660; map is 400x400 added to the right
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

    # create map_image.png from text file
    visualise_map(args.map)

    # drawing map_image rectangle
    map_background_rect = pygame.Rect(700, 0, 400, 660) #
    map_background_colour = (45,45,45)
    pygame.draw.rect(canvas,map_background_colour,map_background_rect)
    # resizing map_image and drawing on the canvas
    map_image = pygame.image.load('map_image.png')
    map_image = pygame.transform.scale(map_image, (400, 400))
    canvas.blit(map_image, (700, 0))
    # adding origin marker for original pibot pos
    origin_dot = pygame.Rect(904,201,4,4) # origin is 906,203 but drawing two pixels either side
    origin_colour = (165,42,42)
    pygame.draw.rect(canvas,origin_colour,origin_dot)

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

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)

    # print(fruits_true_pos)
    # print(aruco_true_pos)

    operate = Operate(args, aruco_true_pos)

    ppi = PenguinPi(args.ip,args.port)

    search_list = read_search_list(args.shopping_list)
    fruit_goal_list = print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)
    
    obstacle_list = np.vstack((fruits_true_pos, aruco_true_pos))

    waypoint = [0.0,0.0]
    robot_pose = [0.0,0.0,0.0]

    operate.ekf_on = True

    #RRT stuff
    #=================
    # run this code inside main loop (start with a button press?)
    for i in range(len(fruit_goal_list)):
        #initialise map each loop to remove path drawings
        canvas.blit(map_image, (700, 0))
        origin_dot = pygame.Rect(904,201,4,4)
        origin_colour = (165,42,42)
        white = (255, 255, 255)
        red = (255, 0, 0)
        pygame.draw.rect(canvas,origin_colour,origin_dot)
        #draw obstacles
        scalefactor_x = 155/1.5
        scalefactor_y = -155/1.5

        # run rrt
        goal = fruit_goal_list[i]
        start = robot_pose
        waypoints_rrt, circles = rrt_waypoints(goal, start, obstacle_list) 

        for circle in circles:
            pygame.draw.circle(canvas, red, (int(circle.center[0] * scalefactor_x+906), int(circle.center[1] * scalefactor_y+203)), circle.radius * scalefactor_x)
        pygame.display.flip()

        #draw path as white line
        coordinates = [[scalefactor_x * x + 906, scalefactor_y * y + 203] for x, y in waypoints_rrt]

        for i in range(len(coordinates) - 1):
            pygame.draw.line(canvas, white, coordinates[i], coordinates[i + 1], 2)
        pygame.display.flip()
        
        for waypoint in waypoints_rrt:
            add_waypoint_from_rrt(waypoint)
        
        operate.draw(canvas)
        pygame.display.update()
        
        print(f"(Hopefully) arrived at {search_list[i]}...")
        
        sleep(2)

    # END RRT stuff  
    # =============

    # while start:
    #     operate.update_keyboard()
    #     operate.take_pic()
    #     drive_meas = operate.control()
    #     operate.update_slam(drive_meas)
    #     operate.record_data()
    #     operate.save_image()
    #     operate.detect_target()
    #     # visualise
    #     operate.draw(canvas)
    #     pygame.display.update()
        
