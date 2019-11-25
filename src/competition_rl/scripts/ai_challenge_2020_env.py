#! /usr/bin/env python

import rospy
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class AiChallengeEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """

    def __init__(self):
        """Initializes a new CubeSingleDisk environment.
        """
        # Variables that we give through the constructor.
        # self.init_roll_vel = init_roll_vel

        self.controllers_list = ['ridgeback_joint_publisher','ridgeback_velocity_controller', 'turret_position_controller']

        self.robot_name_space = ""

        reset_controls_bool = False

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(AiChallengeEnv, self).__init__(controllers_list=self.controllers_list,
                                                  robot_name_space=self.robot_name_space,
                                                  reset_controls=reset_controls_bool,
                                                  start_init_physics_parameters=False)

        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't flow. This is for simulations
        that are pause for whatever reason
        2) If the simulation was running already for some reason, we need to reset the controllers.
        This has to do with the fact that some plugins with tf don't understand the reset of the simulation
        and need to be reset to work properly.
        """
        self.gazebo.unpauseSim()
        self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/amcl_pose", PoseStamped, self._pose_callback)
        rospy.Subscriber("/odometry/filtered", Odometry, self._odom_callback)

        self._turret_position_pub = rospy.Publisher('/turret_controller/position',
                                        Float32, queue_size=1)
        self._goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped, queue_size=1)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

    def _pose_callback(self, data):
        self.amcl_pose = data
    def _odom_callback(self, data):
        self.odom = data

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        self._check_publishers_connection()
        return True


    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        # self._check_joint_states_ready()
        self._check_amcl_pose_ready()
        self._check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    # def _check_joint_states_ready(self):
    #     self.joints = None
    #     while self.joints is None and not rospy.is_shutdown():
    #         try:
    #             self.joints = rospy.wait_for_message("/moving_cube/joint_states", JointState, timeout=1.0)
    #             rospy.logdebug("Current moving_cube/joint_states READY=>" + str(self.joints))
    #
    #         except:
    #             rospy.logerr("Current moving_cube/joint_states not ready yet, retrying for getting joint_states")
    #     return self.joints

    def _check_amcl_pose_ready(self):
        self.amcl_pose = None
        while self.amcl_pose is None and not rospy.is_shutdown():
            try:
                self.amcl_pose = rospy.wait_for_message("/amcl_pose", PoseStamped, timeout=1.0)
                rospy.logdebug("Current /amcl_pose READY=>" + str(self.amcl_pose))

            except:
                rospy.logerr("Current /amcl_pose not ready yet, retrying for getting amcl_pose")

        return self.amcl_pose

    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=1.0)
                rospy.logdebug("Current /odometry/filtered READY=>" + str(self.odom))

            except:
                rospy.logerr("Current /odometry/filtered not ready yet, retrying for getting odom")

        return self.odom

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._turret_position_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _turret_position_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_turret_position_pub Publisher Connected")

        while self._goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _goal_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_goal_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")


    # Methods that the TaskEnvironment will need.
    # ----------------------------
    def move_turret(self, position):
        turret_position = Float32()
        turret_position.data = position
        rospy.logdebug("turret_position >>" + str(turret_position))
        self._turret_position_pub.publish(turret_position)
        # self.wait_until_roll_is_in_vel(turret_position.data)

    # def wait_until_roll_is_in_vel(self, velocity):
    #
    #     rate = rospy.Rate(10)
    #     start_wait_time = rospy.get_rostime().to_sec()
    #     end_wait_time = 0.0
    #     epsilon = 0.1
    #     v_plus = velocity + epsilon
    #     v_minus = velocity - epsilon
    #     while not rospy.is_shutdown():
    #         joint_data = self._check_joint_states_ready()
    #         roll_vel = joint_data.velocity[0]
    #         rospy.logdebug("VEL=" + str(roll_vel) + ", ?RANGE=[" + str(v_minus) + ","+str(v_plus)+"]")
    #         are_close = (roll_vel <= v_plus) and (roll_vel > v_minus)
    #         if are_close:
    #             rospy.logdebug("Reached Velocity!")
    #             end_wait_time = rospy.get_rostime().to_sec()
    #             break
    #         rospy.logdebug("Not there yet, keep waiting...")
    #         rate.sleep()
    #     delta_time = end_wait_time- start_wait_time
    #     rospy.logdebug("[Wait Time=" + str(delta_time)+"]")
    #     return delta_time

    def get_amcl_pose(self):
        return self.amcl_pose


    # Methods that the TaskEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TaskEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()