from gym import spaces
import ai_challenge_2020_env
from openai_ros import multi_robot_gazebo_env
from gym.envs.registration import register
import rospy
import numpy

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import challengeTaskEnv

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
    id='AiChallengeEnv-v1',
    entry_point='multiAgentChallengeTaskEnv:MultiAgentAiChallengeEnv',
    max_episode_steps = timestep_limit_per_episode,
)

class MultiAgentAiChallengeEnv(multi_robot_gazebo_env.MultiRobotGazeboEnv):
    def __init__(self, **kwargs):
        print('robot0 spawn')
        self.robot0 = challengeTaskEnv.AiChallengeEnv(robot_ns="jackal0", init_x=0.6, init_y=0.6, init_yaw=0)
        print('robot1 spawn')
        self.robot1 = challengeTaskEnv.AiChallengeEnv(robot_ns="jackal1", init_x=0.6, init_y=4.7, init_yaw=0)
        print('robot2 spawn')
        self.robot2 = challengeTaskEnv.AiChallengeEnv(robot_ns="jackal2", init_x=7.6, init_y=4.7, init_yaw=3.14)

        print('null spawn')
        super(MultiAgentAiChallengeEnv, self).__init__(
            robot_name_space = 'null',
            start_init_physics_parameters=False)



    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        print('null _set_init_pose')
        self.robot0._set_init_pose()
        self.robot1._set_init_pose()
        self.robot2._set_init_pose()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # print('null _check_all_systems_ready start')
        self.robot1._check_all_systems_ready()
        # print('null _check_all_systems_ready end')

    def _get_obs(self):
        """Returns the observation.
        """
        pass

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        pass

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        self.robot1._set_action(action)

    def _is_done(self, observations):
        """Indicates whether or not the episode is done ( the robot has fallen for example).
        """
        done=False
        return done

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        reward=0
        return reward

    # def _env_setup(self, initial_qpos):
    #     """Initial configuration of the environment. Can be used to configure initial state
    #     and extract information from the simulation.
    #     """
    #     raise NotImplementedError()
