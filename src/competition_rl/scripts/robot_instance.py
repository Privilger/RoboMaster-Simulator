from gym import spaces
import robot_underlying
from gym.envs.registration import register
import rospy
import numpy

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

# register(
#     id='AiChallengeEnv-v0',
#     entry_point='challengeTaskEnv:AiChallengeEnv',
#     max_episode_steps = timestep_limit_per_episode,
# )

class AiRobot(robot_underlying.Robot):
    def __init__(self, **kwargs):
        self.robot_name_space = kwargs['robot_ns']
        self.init_x   = kwargs['init_x']
        self.init_y   = kwargs['init_y']
        self.init_yaw = kwargs['init_yaw']
        self.quat = quaternion_from_euler(0, 0, self.init_yaw)
        self.reset_gazebo_simulation_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_localization_proxy = rospy.Publisher('/' + self.robot_name_space + '/initialpose',
                                                        PoseWithCovarianceStamped, queue_size=10)
        # Only variable needed to be set here
        # number_actions = rospy.get_param('/my_robot_namespace/n_actions')
        number_actions = 2
        self.action_space = spaces.Discrete(number_actions)

        # This is the most common case of Box observation type
        high = numpy.array([
            1,2
        ])

        self.observation_space = spaces.Box(-high, high)

        # Variables that we retrieve through the param server, loded when launch training launch.

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(AiRobot, self).__init__(self.robot_name_space)


    def _set_init_gazebo_pose(self):
        """Sets the Robot in its init pose in Gazebo
        """
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            robot_pose = ModelState()
            robot_pose.model_name = self.robot_name_space
            robot_pose.reference_frame = "/map"
            robot_pose.pose.position.x = self.init_x
            robot_pose.pose.position.y = self.init_y
            robot_pose.pose.position.z = 0
            robot_pose.pose.orientation.x = self.quat[0]
            robot_pose.pose.orientation.y = self.quat[1]
            robot_pose.pose.orientation.z = self.quat[2]
            robot_pose.pose.orientation.w = self.quat[3]
            self.reset_gazebo_simulation_proxy(robot_pose)
            return True
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")
            return False

    def _set_init_ros(self):
        # self.move_turret(0)
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.stamp = rospy.Time.now()
        init_msg.header.frame_id = 'map'
        init_msg.pose.pose.position.x = self.init_x
        init_msg.pose.pose.position.y = self.init_y
        init_msg.pose.pose.orientation.x = self.quat[0]
        init_msg.pose.pose.orientation.y = self.quat[1]
        init_msg.pose.pose.orientation.z = self.quat[2]
        init_msg.pose.pose.orientation.w = self.quat[3]
        # 迷之reset 第一次reset会有问题， 还需要sleep一下等它反应过来
        self.reset_localization_proxy.publish(init_msg)
        rospy.sleep(0.1)
        self.reset_localization_proxy.publish(init_msg)
        rospy.sleep(0.1)
        return True

    def _init_env_variables(self, **kwargs):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # TODO
        # self.init_x = kwargs['init_x']
        # self.init_y = kwargs['init_y']
        # self.init_yaw = kwargs['init_yaw']
        # rospy.logerr('!!!x:',self.init_x)
        # rospy.logerr('!!!y:',self.init_y)
        self.total_distance_moved = 0.0

    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        # TODO: Move robot
        # We tell the OneDiskCube to spin the RollDisk at the selected speed
        rospy.logerr("turret_position >>" + str(action))
        # self.move_turret(action)

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        To know which Variables we have acces to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """
        # TODO
        observations = [
            round(0, 1),
            round(0, 1),
        ]
        return observations

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        # TODO
        done=False
        return done

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        # TODO
        reward=0
        return reward
