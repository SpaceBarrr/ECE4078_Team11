import numpy as np
import heapq

class Circle:
    def __init__(self, c_x, c_y, radius):
        self.center = np.array([c_x, c_y])
        self.radius = radius

    def is_in_collision_with_point(self, point):
        dx = self.center[0] - point[0]
        dy = self.center[1] - point[1]
        return np.all(dx * dx + dy * dy <= self.radius ** 2)

def euclidean_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def astar(start, goal, obstacles):
    open_list = [(0, start)]  # Priority queue with initial cost
    came_from = {}
    g_score = {tuple(point.center): float('inf') for point in obstacles}
    g_score[tuple(start)] = 0

    while open_list:
        _, current = heapq.heappop(open_list)

        if np.all(current == goal):  # Compare individual elements
            path = [current]
            while tuple(current) in came_from:
                current = came_from[tuple(current)]
                path.append(current)
            return path[::-1]  # Reverse the path to start from the beginning

        for neighbor_circle in obstacles:
            if not neighbor_circle.is_in_collision_with_point(current):
                neighbor = neighbor_circle.center
                tentative_g_score = g_score[tuple(current)] + euclidean_distance(current, neighbor)

                if tentative_g_score < g_score[tuple(neighbor)]:
                    came_from[tuple(neighbor)] = current
                    g_score[tuple(neighbor)] = tentative_g_score
                    f_score = tentative_g_score + euclidean_distance(neighbor, goal)
                    heapq.heappush(open_list, (f_score, neighbor))

    return None  # No path found

# Example usage:
start_point = (0, 0)
goal_point = (10, 10)

obstacle1 = Circle(5, 5, 3)
obstacle2 = Circle(8, 8, 2)
obstacles = [obstacle1, obstacle2]

path = astar(start_point, goal_point, obstacles)
if path:
    print("Path found:", path)
else:
    print("No path found")