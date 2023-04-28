from spinup import ppo_tf1 as ppo
import tensorflow as tf
import gym
import gym_gazebo

#!/usr/bin/env python
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time
import liveplot
import qlearn
import numpy as np
from baselines import ppo2




import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

env_fn = lambda : gym.make('GazeboRoundTurtlebotLidar-v0')

ac_kwargs = dict(hidden_sizes=[64,64], activation=tf.nn.relu)

logger_kwargs = dict(output_dir='/home/jimycool/spinningup', exp_name='test-gazebo')

ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=5000, epochs=10, logger_kwargs=logger_kwargs)
