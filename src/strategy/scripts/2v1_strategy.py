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

class Brain:
    def __init__(self):
        self._show_animation = True
        self._resolution = 0.1
        self.w, self.h = 8.1, 5.1
        self.width, self.height = int(ceil(self.w / self._resolution)), int(ceil(self.h / self._resolution))

        self._map = self.initMap()
        self._decision0_pub = rospy.Publisher("/jackal0/move_base_simple/goal", PoseStamped, queue_size=10)
        self._decision1_pub = rospy.Publisher("/jackal1/move_base_simple/goal", PoseStamped, queue_size=10)
        self._debuff_subscriber = rospy.Subscriber("/debuff", String, self.receiveDebuffSignal)
        self._robot0_subscriber = rospy.Subscriber("/jackal0/amcl_pose", PoseStamped, self.ownPositionCB0)
        self._robot1_subscriber = rospy.Subscriber("/jackal1/amcl_pose", PoseStamped, self.ownPositionCB1)
        self._enemy0_subscriber = rospy.Subscriber("/jackal0/obstacle_filtered", Obstacles, self.ownObservationCB0)
        self._enemy1_subscriber = rospy.Subscriber("/jackal1/obstacle_filtered", Obstacles, self.ownObservationCB1)
        self._debuff = []
        self.robot0,     self.robot1     = self.Robot(0.0, 0.0, 0.0), self.Robot(0.0, 0.0, 0.0)
        self.robot0_obs, self.robot1_obs = [], []
        self.observation = []
        self._rho = 1.2
        self._range = 5
        self._a_star_planner_class = self._planner_def_inner_class()
        self._a_star_planner = self._a_star_planner_class()

        time_start = time.time()
        # rx, ry = self._a_star_planner.planning(sx=2, sy=2,
        #                                  gx=20, gy=20)
        # plt.plot(rx, ry, "r")
        # plt.show()





    def makeDecision(self):
        if not self.observation:
            return
        goal0, goal1 = PoseStamped(), PoseStamped()
        goal0.header.frame_id, goal0.header.frame_id = "/map", "/map"

        # robot0
        target = self.observation[0]
        yaw_angle = math.atan2(target.y - self.robot0.y, target.x - self.robot0.x)

        theta = 0
        if yaw_angle >= 0:
            theta = yaw_angle - math.pi
        elif yaw_angle < 0:
            theta = math.pi + yaw_angle
        else:
            rospy.logerr("invalid yaw")

        theta = theta + np.random.uniform(-self._range / 180 * math.pi, self._range / 180 * math.pi)
        theta = np.clip(theta, -math.pi, math.pi)
        target.x, target.y = target.x + self._rho * math.cos(theta), target.y + self._rho * math.sin(theta)

        goal0.pose.position.x, goal0.pose.position.y = target.x, target.y
        [goal0.pose.orientation.w,
         goal0.pose.orientation.x,
         goal0.pose.orientation.y,
         goal0.pose.orientation.z] = self._createQuaternionFromYaw(yaw_angle)
        print(yaw_angle * 180 / math.pi)

        # robot1
        target = self.observation[0]
        yaw_angle = math.atan2(target.y - self.robot1.y, target.x - self.robot1.x)

        theta = 0
        if yaw_angle >= 0:
            theta = yaw_angle - math.pi
        elif yaw_angle < 0:
            theta = math.pi + yaw_angle
        else:
            rospy.logerr("invalid yaw")

        theta = theta + np.random.uniform(-self._range / 180 * math.pi, self._range / 180 * math.pi)
        theta = np.clip(theta, -math.pi, math.pi)
        target.x, target.y = target.x + self._rho * math.cos(theta), target.y + self._rho * math.sin(theta)

        goal1.pose.position.x, goal1.pose.position.y = target.x, target.y
        [goal1.pose.orientation.w,
         goal1.pose.orientation.x,
         goal1.pose.orientation.y,
         goal1.pose.orientation.z] = self._createQuaternionFromYaw(yaw_angle)

        self._decision0_pub.publish(goal0)
        self._decision1_pub.publish(goal1)

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

    def ownPositionCB0(self, msg):
        self.robot0.x = msg.pose.position.x
        self.robot0.y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robot0.yaw = y

    def ownPositionCB1(self, msg):
        self.robot1.x = msg.pose.position.x
        self.robot1.y = msg.pose.position.y
        [y, p, r] = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w]).as_euler('zyx', degrees=True)
        self.robot1.yaw = y

    def ownObservationCB0(self, data):
        self.robot0_obs = []
        for enemy in data.circles:
            self.robot0_obs.append(self.Robot(enemy.center.x, enemy.center.y, 0.0))
        self.merge_observation()



    def ownObservationCB1(self, data):
        self.robot1_obs = []
        for enemy in data.circles:
            self.robot1_obs.append(self.Robot(enemy.center.x, enemy.center.y, 0.0))
        self.merge_observation()

    def merge_observation(self):
        self.observation = self.robot0_obs
        self.observation += self.robot1_obs
        self.observation = [item for item in self.observation if self.observation.count(item) == 1]

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


if __name__ == '__main__':

    # r = R.from_euler('zyx', [49.590, -37.455,   144.980], degrees=True).as_quat()
    # print(r)

    try:
        # making_decision()
        rospy.init_node('listener', anonymous=True)

        brain = Brain()

        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.2)
            # self.makeDecision()
            brain.makeDecision()

    except rospy.ROSInterruptException:
        pass
