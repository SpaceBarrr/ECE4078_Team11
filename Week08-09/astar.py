"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import matplotlib.pyplot as plt
from Obstacle import *
import json

show_animation = True

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                return None, None
                # break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                # if len(closed_set.keys()) % 10 == 0:
                #     plt.pause(0.001)
                #     plt.savefig("astar_temp.png")

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
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

def calculate_gradient(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    if x2 - x1 == 0:
        return float('inf')  # Vertical line, use infinity as gradient
    return (y2 - y1) / (x2 - x1)

def simplify_path(waypoint_x, waypoint_y, threshold=1):
    if len(waypoint_x) != len(waypoint_y) or len(waypoint_x) <= 1:
        return None  # Invalid input

    waypoints = list(zip(waypoint_x, waypoint_y))
    simplified_path = [waypoints[0]]
    prev_point = waypoints[0]
    prev_gradient = None

    for current_point in waypoints[1:]:
        gradient = calculate_gradient(prev_point, current_point)
        if prev_gradient is None or abs(gradient - prev_gradient) >= threshold:
            simplified_path.append(current_point)
            prev_gradient = gradient
        prev_point = current_point

    return simplified_path

# ================================

def a_start(start_x,start_y,goal_x,goal_y,obstacle_list,last_fruit=None):
    print("Running astar...")
    plt.clf() # so we don't get previous plots overlaid

    # start and goal position
    sx = start_x  # [m]
    sy = start_y  # [m]
    gx = goal_x  # [m]
    gy = goal_y  # [m]
    grid_size = 0.02  # [m]
    robot_radius = 0.08  # [m]

    # OBSTACLE_RADIUS = 0 # 1 is 10cm 
    obstacle_list = (obstacle_list*10).astype(int)

    # set border positions
    ox, oy = [], []
    for i in range(-15, 15):
        ox.append(i*0.1)
        oy.append(-1.5)
    for i in range(-15,15):
        ox.append(1.5)
        oy.append(i*0.1)
    for i in range(-15, 15+1): # requires +1 at end for square
        ox.append(i*0.1)
        oy.append(1.5)
    for i in range(-15, 15):
        ox.append(-1.5)
        oy.append(i*0.1)

    #set obstacle positions
    for obst in range(len(obstacle_list)):
        if obst == last_fruit:
            OBSTACLE_RADIUS = 0
        else: 
            OBSTACLE_RADIUS = 1
        
        for i in range(obstacle_list[obst][0]-OBSTACLE_RADIUS, obstacle_list[obst][0]+OBSTACLE_RADIUS): # bottom
            ox.append(i*0.1)
            oy.append((obstacle_list[obst][1]-OBSTACLE_RADIUS)*0.1)
        for i in range(obstacle_list[obst][1]-OBSTACLE_RADIUS, obstacle_list[obst][1]+OBSTACLE_RADIUS): # right
            ox.append((obstacle_list[obst][0]+OBSTACLE_RADIUS)*0.1)
            oy.append(i*0.1)
        for i in range(obstacle_list[obst][0]-OBSTACLE_RADIUS, obstacle_list[obst][0]+OBSTACLE_RADIUS+1): # top , requires +1 at end for square
            ox.append(i*0.1)
            oy.append((obstacle_list[obst][1]+OBSTACLE_RADIUS)*0.1)
        for i in range(obstacle_list[obst][1]-OBSTACLE_RADIUS, obstacle_list[obst][1]+OBSTACLE_RADIUS): # left
            ox.append((obstacle_list[obst][0]-OBSTACLE_RADIUS)*0.1)
            oy.append(i*0.1)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        plt.savefig("astar_temp.png")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if rx == None and ry == None:
        return None, None
    #rsimpx = [-1.0, -1.0, -0.52, -0.21999, 0.0]
    #rsimpy = [-1.0, -0.7, -0.2199, -0.2199, 0.0]

    # Simplify waypoints
    waypoint_x = rx
    waypoint_y = ry

    simplified_path = simplify_path(waypoint_x, waypoint_y)
    if simplified_path:
        for point in simplified_path:
            print(point)
    else:
        print("Invalid input: Lengths of waypoint_x and waypoint_y must be the same.")
   
    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r") 
        plt.plot(waypoint_x,waypoint_y,"-k")
        # plt.pause(0.001)
        # plt.show()
        plt.savefig("astar_generated.png") # for cian, cant view plt.show() on my laptop
    
    waypoint = []
    for i in range(len(waypoint_x)) :
        waypoint.append( [waypoint_x[i], waypoint_y[i]] )
    
    print(waypoint)

    return waypoint, simplified_path
