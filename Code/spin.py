from spinup import ppo_tf1 as ppo
from spinup import ppo_pytorch as ppopy
from spinup import sac_tf1 as sac
from spinup import trpo_tf1 as trpo
from spinup import vpg_tf1 as vpg
from spinup import ddpg_tf1 as ddpg
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

ac_kwargs = dict(hidden_sizes=[256,256], activation=tf.nn.relu)#64,64

logger_kwargs = dict(output_dir='/home/jimycool/Training/Level_2_ppo2', exp_name='PPO')

# ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=200, pi_lr=0.0005, vf_lr=0.003, clip_ratio=0.3, target_kl=0.05, lam=0.999,gamma=0.999,logger_kwargs=logger_kwargs)
ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=100, pi_lr=0.0005, vf_lr=0.003, clip_ratio=0.2, target_kl=0.01, lam=0.99,gamma=0.99,logger_kwargs=logger_kwargs)
# ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=100, logger_kwargs=logger_kwargs)
# ddpg(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=100, logger_kwargs=logger_kwargs)
# vpg(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=100, logger_kwargs=logger_kwargs)
# trpo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=1000, epochs=100, logger_kwargs=logger_kwargs)




# spinup.ppo_pytorch(env_fn, actor_critic=<MagicMock spec='str' id='140554322637768'>, ac_kwargs={}, seed=0, steps_per_epoch=4000, epochs=50, gamma=0.99, clip_ratio=0.2, pi_lr=0.0003, vf_lr=0.001, train_pi_iters=80, train_v_iters=80, lam=0.97, max_ep_len=1000, target_kl=0.01, logger_kwargs={}, save_freq=10)