import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from obstacle_detector.msg import Obstacles
from std_msgs.msg import String
import a_star
import matplotlib.pyplot as plt
import numpy as np
from math import ceil
import time
import math
from scipy.spatial.transform import Rotation as R
import threading
import utility


class Brain:
    def __init__(self):
        self._show_animation = True
        self._resolution = 0.1
        self.w, self.h = 8.1, 5.1
        self.width, self.height = int(ceil(self.w / self._resolution)), int(ceil(self.h / self._resolution))

        self._map = self.initMap()
        self._decision_pub = [rospy.Publisher("/jackal0/move_base_simple/goal", PoseStamped, queue_size=10),
                              rospy.Publisher("/jackal1/move_base_simple/goal", PoseStamped, queue_size=10)]
        self._debuff_subscriber = rospy.Subscriber("/debuff", String, self.receiveDebuffSignal)
        self._robots_subscriber = [rospy.Subscriber("/jackal0/amcl_pose", PoseStamped, self.ownPositionCB0),
                                   rospy.Subscriber("/jackal1/amcl_pose", PoseStamped, self.ownPositionCB1)]
        self._enemies_subscriber = [rospy.Subscriber("/jackal0/obstacle_filtered", Obstacles, self.ownObservationCB0),
                                    rospy.Subscriber("/jackal1/obstacle_filtered", Obstacles, self.ownObservationCB1)]
        self._debuff = []
        self.robots = [self.Robot(0.0, 0.0, 0.0), self.Robot(0.0, 0.0, 0.0)]
        self.robot_obs = [[], []]
        self.observation = []
        self._rho = 1.2
        self._rangeAngle = 5
        self._attackDist = 1.8
        # self._a_star_planner_class = self._planner_def_inner_class()
        # self._a_star_planner = self._a_star_planner_class()
        #
        # time_start = time.time()
        # rx, ry = self._a_star_planner.planning(sx=2, sy=2,
        #                                  gx=20, gy=20)
        # plt.plot(rx, ry, "r")
        # plt.show()

    # Problem 给定的随机点不一定是可达点
    def callFriend(self):
        if not self.observation:
            return

        goal = PoseStamped()
        goal.header.frame_id = "/map"
        target = self.observation[0]
        for i in range(2):
            yaw_angle = math.atan2(target.y - self.robots[i].y, target.x - self.robots[i].x)

            theta = 0
            if yaw_angle >= 0:
                theta = yaw_angle - math.pi
            elif yaw_angle < 0:
                theta = math.pi + yaw_angle
            else:
                rospy.logerr("invalid yaw")

            theta = theta + np.random.uniform(-self._rangeAngle / 180 * math.pi, self._rangeAngle / 180 * math.pi)
            theta = np.clip(theta, -math.pi, math.pi)
            target.x, target.y = target.x + self._rho * math.cos(theta), target.y + self._rho * math.sin(theta)

            goal.pose.position.x, goal.pose.position.y = target.x, target.y
            [goal.pose.orientation.w,
             goal.pose.orientation.x,
             goal.pose.orientation.y,
             goal.pose.orientation.z] = self._createQuaternionFromYaw(yaw_angle)

            self._decision_pub[i].publish(goal)

    def isInDanger(self):
        if len(self.observation) < 2:
            return False
        robot0, robot1 = np.array(self.robots[0]), np.array(self.robots[1])
        enemy0, enemy1 = np.array(self.observation[0]), np.array(self.observation[1])

        flag = False
        if np.linalg.norm(robot0 - enemy0) < self._attackDist and np.linalg.norm(robot0 - enemy1) < self._attackDist:
            goal = self.selectGoal(self.robots[0].x, self.robots[0].y)
            self.escape(0, goal)
            flag = True

        if np.linalg.norm(robot1 - enemy0) < self._attackDist and np.linalg.norm(robot1 - enemy1) < self._attackDist:
            goal = self.selectGoal(self.robots[1].x, self.robots[1].y)
            self.escape(1, goal)
            flag = True

        return flag

    def selectGoal(self, x, y):
        areaID = utility.belongToDistrict(x, y)
        goalID = 7 - areaID
        goal = PoseStamped()
        goal.header.frame_id = "/map"
        goal.pose.position.x, goal.pose.position.y = utility.setGoal(goalID)
        goal.pose.orientation.w = 1
        print("now: ", areaID, "goal: ",goalID)
        return goal


    def escape(self, id, goal):
        self._decision_pub[id].publish(goal)

    def _createQuaternionFromYaw(self, yaw):
        # input: r p y
        r = R.from_euler('zyx', [0, 0, yaw], degrees=False).as_quat()
        # output: w x y z
        return [r[3], r[2], r[1], r[0]]

    def initMap(self):
        map = np.zeros( (self.width, self.height), dtype = int)
        map[0, :], map[-1, :], map[:, 0], map[:, -1] = 1, 1, 1, 1

        v_obsticle = np.array([[0, 3.85], [3.6, 1],
                               [3.5, 3.85], [7.1, 1]])
        v_min_obsticle = np.array([[1.5, 2.425],
                                   [5.8, 2.425]])
        h_obsticle = np.array([[1.5, 0], [8.1 - 1.5 - 0.25, 4.1]])
        c_obsticle = np.array([[4, 2.4]])

        for i in v_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(1 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in v_min_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.8 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in h_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.25 / self._resolution))):
                for k in range(int(ceil(1 / self._resolution))):
                    map[x + j][y + k] = 1

        for i in c_obsticle:
            x, y = int(ceil(i[0] / self._resolution)), int(ceil(i[1] / self._resolution))
            map[x][y] = 1
            for j in range(int(ceil(0.25 / self._resolution))):
                for k in range(int(ceil(0.25 / self._resolution))):
                    map[x + j][y + k] = 1

        return map

    def draw(self):
        plt.imshow(self._map)
        plt.colorbar()
        plt.show()

    def receiveDebuffSignal(self, new_buff):
        self._debuff = []
        for i in range(1,7):
            if new_buff.data[2*i] == '1':
                self._debuff.append(i)

    class Robot:
        x, y, yaw = 0.0, 0.0, 0.0
        theta = 0.25
        def __init__(self, x, y, yaw):
            self.x, self.y, self.yaw = x, y, yaw
        def __str__(self):
            return "x: %.2f, y: %.2f" % (self.x, self.y)
        def __eq__(self, other):
            return abs(self.x - other.x) < self.theta and abs(self.y - other.y) < self.theta

        def __sub__(self, other):
            return self.x - other.x, self.y - other.y

    def ownPositionCB0(self, msg):
        self.robots[0].x = msg.pose.position.x
        self.robots[0].y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robots[0].yaw = y

    def ownPositionCB1(self, msg):
        self.robots[1].x = msg.pose.position.x
        self.robots[1].y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robots[1].yaw = y

    def ownObservationCB0(self, data):
        self.robot_obs[0] = []
        for enemy in data.circles:
            self.robot_obs[0].append(self.Robot(enemy.center.x, enemy.center.y, 0.0))
        self.observation = self.robot_obs[0] + self.robot_obs[1]
        # print("robot0 obs", len(self.robot_obs[0]))
        # self.merge_observation()

    def ownObservationCB1(self, data):
        self.robot_obs[1] = []
        for enemy in data.circles:
            self.robot_obs[1].append(self.Robot(enemy.center.x, enemy.center.y, 0.0))
        self.observation = self.robot_obs[0] + self.robot_obs[1]
        # print("robot1 obs", len(self.robot_obs[1]))
        # self.merge_observation()

    def merge_observation(self):
        mutex = threading.Lock()
        with mutex:
            print("mutex")
            self.observation = self.robot_obs[0]
            print("1", len(self.observation))
            self.observation += self.robot_obs[1]
            print("2", len(self.observation))
            self.observation = [item for item in self.observation if self.observation.count(item) == 1]
            print("3", len(self.observation))
    def _planner_def_inner_class(self):
        outter_class = self

        class modifiedAStarPlanner(a_star.AStarPlanner):
            def __init__(self):
                self.outter = outter_class
                obstacles_matrix = self.outter._map
                ox, oy = [], []
                for i in range(self.outter.width):
                    for j in range(self.outter.height):
                        if obstacles_matrix[i, j] != 0:
                            ox.append(i)
                            oy.append(j)
                grid_size = 1
                robot_radius = 0.5
                # if self.outter._show_animation:
                #     plt.cla()
                #     plt.plot(ox, oy, ".")
                    # plt.xticks([])
                    # plt.yticks([])
                    # plt.grid(True)
                    # plt.axis("equal")
                    # plt.show()
                super().__init__(ox, oy, grid_size, robot_radius)

            def planning(self, sx, sy, gx, gy):
                """
                A star path search
                input:
                    sx: start x position [m]
                    sy: start y position [m]
                    gx: goal x position [m]
                    gx: goal x position [m]
                output:
                    rx: x position list of the final path
                    ry: y position list of the final path
                """
                nstart = self.Node(self.calc_xyindex(sx, self.minx),
                                   self.calc_xyindex(sy, self.miny), 0.0, -1)
                ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                                  self.calc_xyindex(gy, self.miny), 0.0, -1)

                open_set, closed_set = dict(), dict()
                open_set[self.calc_grid_index(nstart)] = nstart

                # print('start')
                while 1:
                    if len(open_set) == 0:
                        print("Open set is empty..")
                        break
                    c_id = min(
                        open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
                    current = open_set[c_id]

                    # show graph
                    plt.xlim((0, self.outter.width))
                    plt.ylim((0, self.outter.height))
                    if self.outter._show_animation:  # pragma: no cover
                        plt.plot(self.calc_grid_position(current.x, self.minx),
                                 self.calc_grid_position(current.y, self.miny), "xc")
                        if len(closed_set.keys()) % 10 == 0:
                            plt.pause(0.01)

                    if current.x == ngoal.x and current.y == ngoal.y:
                    # if self.outter.dist2points(np.array([current.x, 21 - current.y]),
                    #                            np.array([gx, 21 - gy])) <= self.outter.agent_view_size - 2 and \
                    #         self.outter.canSee(np.array([current.x, 21 - current.y]), np.array([gx, 21 - gy])):
                        print("Find goal")
                        ngoal.pind = current.pind
                        ngoal.cost = current.cost
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

                rx, ry = self.calc_final_path(ngoal, closed_set)

                return rx, ry

        return modifiedAStarPlanner


def call_rosspin():
    rospy.spin()

if __name__ == '__main__':

    try:
        # making_decision()
        rospy.init_node('listener', anonymous=True)
        rate = rospy.Rate(10)
        brain = Brain()
        spin_thread = threading.Thread(target=call_rosspin).start()
        # rospy.spin()

        while not rospy.core.is_shutdown():
            # self.makeDecision()
            # brain.callFriend()
            brain.isInDanger()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
