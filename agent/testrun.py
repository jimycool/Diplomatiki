from spinup.utils.test_policy import load_policy_and_env, run_policy
import gym_gazebo
import gym
from gym import wrappers
from gym.wrappers.monitoring.video_recorder import VideoRecorder


import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy



_, get_action = load_policy_and_env('/home/jimycool/spinningup')
env = gym.make('GazeboRoundTurtlebotLidar-v0')

#VIDEO REC
#env = gazebo_env.GazeboEnv
#rec = VideoRecorder(env,"/home/jimycool/Videos")
#env.reset()
#rec.capture_frame()
#rec.close()
#assert not rec.empty
#assert not rec.broken
#assert os.path.exists(rec.path)
#f = open(rec.path)
#assert os.fstat(f.fileno()).st_size > 100 



run_policy(env, get_action,num_episodes=5,render=False)