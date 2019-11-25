from gym import spaces
import ai_challenge_2020_env
from gym.envs.registration import register
import rospy
import numpy

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
    id='AiChallengeEnv-v0',
    entry_point='challengeTaskEnv:AiChallengeEnv',
    max_episode_steps = timestep_limit_per_episode,
)

class AiChallengeEnv(ai_challenge_2020_env.AiChallengeEnv):
    def __init__(self):

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
        super(AiChallengeEnv, self).__init__()


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        # TODO
        self.move_turret(0)
        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # TODO
        self.total_distance_moved = 0.0


    def _set_action(self, action):
        """
        Move the robot based on the action variable given
        """
        # TODO: Move robot
        # We tell the OneDiskCube to spin the RollDisk at the selected speed
        self.move_joints(action)

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
