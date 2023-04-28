import gym
import rospy
import roslaunch
import time
import numpy as np
import math

import geometry_msgs.msg

from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactState
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

from gym.utils import seeding

class GazeboRoundTurtlebotLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self,  observation_size=0,min_range = 0.13,max_range = 3.5,max_env_size=3,got_key=False,got_key_reward=False,opened_door=False):
        # Launch the simulation with the given launchfile name
        #gazebo_env.GazeboEnv.__init__(self, "humanlaunch.launch")
        gazebo_env.GazeboEnv.__init__(self, "GazeboRoundTurtlebotLidar_v0.launch")
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        # self.contact_sub = rospy.Subscriber("/my_bumper",ContactState,contact_callback)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #F,L,R
        
        #MINEEEEEEEEEEe
        self.got_key = got_key
        self.got_key_reward = got_key_reward
        self.opened_door = opened_door
        
        
        self.observation_size = observation_size
        self.min_range = min_range
        self.max_range = max_range
        self.max_env_size = max_env_size
       # low, high = self.get_observation_space_values()
        self.observation_space = spaces.Box(low=0.0599, high=20.0, shape=(2, ), dtype=np.float64)
      #  self.observation_space =spaces.Dict({
         #   'image':
         #   spaces.Box(low=0,
                #      high=255,
                  #     shape=(2,),dtype=np.float64)
                #       })
      #  self.observation_space = spaces.Box(low, high, dtype=np.float32)
        #self.observation_space=   self.observation_space.reshape([2,])
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        
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


   # def get_observation_space_values(self):
        #low = np.append(np.full(self.observation_size, self.min_range), np.array([-math.pi, 2], dtype=object))
        #high = np.append(np.full(self.observation_size, self.max_range), np.array([math.pi, self.max_env_size], dtype=object))
        #return low, high


    # def contact_callback(msg):
        # collision1 = msg.collision1_name
        # collision2 = msg.collision2_name
        # print(collision1_name)
        # print(collision2_name)
        
        
    
    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 0.2
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf'):
                    discretized_ranges.append(6)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data.ranges[i]))
            if (min_range > data.ranges[i] > 0):
                done = True
        #print(discretized_ranges)
        #print(type(discretized_ranges))
        #print(discretized_ranges.shape)
         
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3  #0.3
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = 0.3 # 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.1
            vel_cmd.angular.z = -0.3  # -0.3
            self.vel_pub.publish(vel_cmd)


        #LASER
        
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
               # print(data)
               # print(type(data))
                #print(data.shape)
            except:
                pass
                
        
        #HANDLE
        
        handle_contact =None
        if not self.got_key :
            handle_contact = rospy.wait_for_message('/my_contact_handle', ContactsState, timeout=5)
        
        #DOOR    

        door_contact =None
        door_contact = rospy.wait_for_message('/my_contact_door', ContactsState, timeout=5)  
        # door_contact_2 =None
        # door_contact_2 = rospy.wait_for_message('/my_contact_door_2', ContactsState, timeout=5)
        
 
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed") 
            
            
        
      



        state,done = self.discretize_observation(data,2)
    
        # print(self.got_key)
        # print(self.got_key_reward)
        # print(self.opened_door)
        # # print(door_contact)
   
          #FOUND HANDLE!!!
        if not self.got_key :
            if handle_contact !=None:
                if handle_contact.states[0].collision2_name != "ground_plane::link::collision":     
                    self.got_key=True
                    print ("GOT HANDLE!!" )
                                    # Initialize Gazebo model state service client
                    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                    # Create a ModelState message
                    model_state_msg = ModelState()

                    # Set the name of the model to be changed
                    model_state_msg.model_name = 'door_handle'

                    # Set the new pose of the model
                    new_pose = Pose()
                    new_pose.position = Point(x=10.0, y=10.0, z=0.0)
                    new_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    model_state_msg.pose = new_pose

                    # # Set the twist of the model (optional)
                    # model_state_msg.twist = geometry_msgs.msg.Twist()

                    # # Set the reference frame (optional)
                    # model_state_msg.reference_frame = 'world'

                    # Call the set_model_state service to update the pose of the model
                    resp = set_model_state(model_state_msg)
                    
                    
                    
                    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                    # Create a ModelState message
                    model_state_msg = ModelState()

                    # Set the name of the model to be changed
                    model_state_msg.model_name = 'hinged_door'

                    # Set the new pose of the model
                    new_pose = Pose()
                    new_pose.position = Point(x=3.95572, y=-3.21795, z=0.000194)
                    new_pose.orientation = Quaternion(x=0.000104, y=7e-06, z=1.5, w=1.0)
                    model_state_msg.pose = new_pose
                    # # Set the twist of the model (optional)
                    # model_state_msg.twist = geometry_msgs.msg.Twist()

                    # # Set the reference frame (optional)
                    # model_state_msg.reference_frame = 'world'

                    # Call the set_model_state service to update the pose of the model
                    resp = set_model_state(model_state_msg)
                    # rospy.wait_for_service('/gazebo/delete_model')
                    # delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                    # delete_model('door_handle')
                # reward = 1000
                # print(reward)
                # done= True   
             
             
            # #FOUND DOOR!!!
        if door_contact != None:
            if door_contact.states[0] !=None:
                if door_contact.states[0].collision2_name != "ground_plane::link::collision":           
                    if got_key == False:
                        print ("NEED KEY :( " )
                    else :
                        opened_door = True
                        print ("OPENED DOOR 1 !!!!! " )
                        done= True
                
        # if door_contact_2.states[0].collision2_name != "ground_plane::link::collision":           
            # if got_key == False:
                # print ("NEED KEY :( " )
            # else :
                # opened_door = True
                # print ("OPENED DOOR 2 !!!!! " )
                # done= True
    
        
        # contact = rospy.wait_for_message('/my_bumper', ContactsState, timeout=5)
        # print(contact)
        # print(contact.states[0].collision1_name)
       
        
        
        
        #DONE SHOULD BE AFTER  : 1. HANDLE , 2. DOOR , 3. HUMAN
        
        if not done:
            if  self.got_key_reward == False and self.got_key==True :     #reward for getting key (only once)     +++   self.got_key_reward == False and
                reward = 1000
                print(reward)               
                self.got_key_reward = True
            elif action == 0:
                reward = 3 # +5
            else:
                reward = 5  # +1
          
        else:
            if self.opened_door:                         #reward for opening the door with the key
                reward = 2000
            # if handle_contact.states[0].collision2_name != "ground_plane::link::collision":      
                # reward = 1000
                # print ("GOT HANDLE!!" )
            # elif door_contact.states[0].collision2_name != "ground_plane::link::collision":      
                # reward = 500
                # print ("GOT DOOR!!" )
            else:                                   # negative reward for crashing in the wall (or door without the key )
                reward = -1000 # -200
                # print ("WALL!!!")
            # print (reward)
           # print(contact)
           
        
        # if got_key_reward:
            # got_key_reward =False
       

         
        #return state, reward, done, {}
        return np.asarray(state,dtype=float), reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        
        
        # # #RESPAWN KEY
        if self.got_key:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
            # Create a ModelState message
            model_state_msg = ModelState()
    
            # Set the name of the model to be changed
            model_state_msg.model_name = 'door_handle'
    
            # Set the new pose of the model
            new_pose = Pose()
            new_pose.position = Point(x=2.0, y=-1.96161, z=  -0.000845 )
            new_pose.orientation = Quaternion(x=-1.7e-05, y= 0.000454 , z=-4e-06, w=1.0)
            model_state_msg.pose = new_pose
    
            # # Set the twist of the model (optional)
            # model_state_msg.twist = geometry_msgs.msg.Twist()
    
            # # Set the reference frame (optional)
            # model_state_msg.reference_frame = 'world'
    
            # Call the set_model_state service to update the pose of the model
            resp = set_model_state(model_state_msg)
            # spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            # # Spawn model at the same position
            # model_name = 'door_handle'
            # model_path = '/home/jimycool/.gazebo/models/door_handle/model.sdf'
            # with open(model_path, 'r') as f:
                # model_xml = f.read()
            # model_pose = Pose()
            # model_pose.position.x = 2.0
            # model_pose.position.y = -1.96161
            # model_pose.position.z =  -0.000845 
            # model_pose.orientation.x = -1.7e-05
            # model_pose.orientation.y =  0.000454 
            # model_pose.orientation.z = -4e-06
            # model_pose.orientation.w = 1.0
            # spawn_model_service(model_name, model_xml, '', model_pose, 'world')
            self.got_key = False
            self.got_key_reward = False
            self.opened_door = False



        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state ,done= self.discretize_observation(data,2)
       # print(state)
        #print(type(state))
        #print(state.shape)
        #print(np.asarray(state,dtype=float))
        #print(np.asarray(state,dtype=object))
        return np.asarray(state,dtype=float)
        #return state
