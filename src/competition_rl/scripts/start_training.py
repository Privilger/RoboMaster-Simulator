#!/usr/bin/env python

import gym
import time
import numpy
import random
import time
from gym import wrappers

# ROS packages required
import rospy
import rospkg

import challengeTaskEnv

# from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

from gym import envs

def main():
    rospy.init_node('aichallenge_gym_node', anonymous=True, log_level=rospy.WARN)

    env = gym.make('AiChallengeEnv-v0')

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('competition_rl')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo ( "Monitor Wrapper started")

    # last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    # Alpha = rospy.get_param("/moving_cube/alpha")

    # Initialize the environment and get first state of the robot
    observation = env.reset()
    # state = ''.join(map(str, observation))

if __name__ == '__main__':
    main()
