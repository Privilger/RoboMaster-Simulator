import numpy as np
from gazebo_msgs.srv import GetModelState
import rospy

class MyMap():
    def __init__(self):
        self.proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.map = [
            [                  None, np.array([2.03, 4.26]), np.array([3.00, 4.42]), np.array([4.21, 4.38]), np.array([5.36, 4.36]),                   None, np.array([7.53, 4.30])],
            [np.array([0.89, 3.19]), np.array([2.03, 3.64]), np.array([3.00, 3.19]), np.array([4.15, 3.31]), np.array([5.31, 3.31]), np.array([6.34, 3.38]), np.array([7.36, 3.31])],
            [np.array([0.80, 2.57]),                   None, np.array([3.14, 2.50]),                   None, np.array([5.11, 2.50]),                   None, np.array([7.36, 2.50])],
            [np.array([0.88, 1.72]), np.array([2.03, 1.72]), np.array([3.00, 1.72]), np.array([4.10, 1.79]), np.array([5.31, 1.84]), np.array([6.34, 1.78]), np.array([7.36, 1.95])],
            [np.array([0.80, 1.00]),                   None, np.array([3.03, 0.56]), np.array([4.01, 0.54]), np.array([5.51, 0.64]), np.array([6.58, 0.84]),                   None]
        ]
        # self._remove_invalid_place_occupied_by_obstacle()

    def _remove_invalid_place_occupied_by_obstacle(self):
        obstacles = ["unit_box_clone", "unit_box_clone_clone", "unit_box_clone_clone_clone",
                     "unit_box_clone_clone_clone_clone", "unit_box_clone_clone_clone_clone_clone"]
        for obstacle in obstacles:
            data = self.proxy(model_name=obstacle, relative_entity_name = "world")
            [x, y] = self._obstacle_location(data)
            self.map[y][x] = None


    def _obstacle_location(self, current):
        current_position = np.array([
            current.pose.position.x,
            current.pose.position.y
        ])

        x_min, y_min, dist = 100, 100, 100000
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                if (self.map[i][j] is None):
                    continue
                elif (np.linalg.norm(current_position - self.map[i][j]) < dist):
                    x_min, y_min, dist = j, i, np.linalg.norm(current_position - self.map[i][j])
        return [x_min, y_min]

my_map = MyMap()

# Can be optimize by KD tree
def my_location(current):
    current_position = np.array([
        current.transform.translation.x,
        current.transform.translation.y
    ])
    x_min, y_min, dist = 100, 100, 100000
    for i in range(len(my_map.map)):
        for j in range(len(my_map.map[0])):
            if (my_map.map[i][j] is None):
                continue
            elif (np.linalg.norm(current_position - my_map.map[i][j]) < dist):
                x_min, y_min, dist = j, i, np.linalg.norm(current_position - my_map.map[i][j])
    return [x_min, y_min]

def enemy_location(enemy):
    if len(enemy) == 0:
        return [-1, -1]

    current_position = np.array([
        enemy[0].center.x,
        enemy[0].center.y,
    ])

    x_min, y_min, dist = 100, 100, 100000
    for i in range(len(my_map.map)):
        for j in range(len(my_map.map[0])):
            if (my_map.map[i][j] is None):
                continue
            elif (np.linalg.norm(current_position - my_map.map[i][j]) < dist):
                x_min, y_min, dist = j, i, np.linalg.norm(current_position - my_map.map[i][j])

    return [x_min, y_min]

def get_action_goal(direction, current=None):
    [x, y] = my_location(current)
    if (np.array([x, y])==np.array([3, 0])).all() and (direction==np.array([0, -1])).all():
        return None
    if (np.array([x, y])==np.array([3, 1])).all() and (direction==np.array([0,  1])).all():
        return None
    if (np.array([x, y])==np.array([3, 4])).all() and (direction==np.array([0,  1])).all():
        return None
    if (np.array([x, y])==np.array([3, 3])).all() and (direction==np.array([0, -1])).all():
        return None
    x = x + direction[0]
    y = y - direction[1]
    if (0<=x and x<len(my_map.map[0])) and (0<=y and y<len(my_map.map)) and (not (my_map.map[y][x] is None)):
        goal_point = my_map.map[y][x]
        return goal_point
    else:
        return None