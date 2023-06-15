import gym
import rospy
import roslaunch
import time
import numpy as np
import math
from math import pi

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
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gym.utils import seeding

class GazeboRoundTurtlebotLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self,  observation_size=0,min_range = 0.13,max_range = 3.5,max_env_size=3,got_key=False,got_key_reward=False,opened_door=False,reached_human= False, goal_x=4,goal_y=-2,heading = -1):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboRoundTurtlebotLidar_v0.launch")
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #F,L,R
        
        self.got_key = got_key
        self.got_key_reward = got_key_reward
        self.opened_door = opened_door
        self.reached_human = reached_human
        
        
        self.goal_x=goal_x
        self.goal_y=goal_y
        self.heading = heading
        self.steps = 0
        self.current_steps = 0
        self.iterations = 1
        self.num_keys = 0
        self.successes = 0
        
        self.observation_size = observation_size
        self.min_range = min_range
        self.max_range = max_range
        self.max_env_size = max_env_size
        self.observation_space = spaces.Box(low=0.0599, high=20.0, shape=(4, ), dtype=np.float64)       # shape 4  --> 2 from laser, 1 goal_distance , 1 heading
      
        self.reward_range = (-np.inf, np.inf)

        self._seed()
        
      
        
    def next_target(model_name):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        model_name = mode_name
        reference_frame = 'world'
        
        response = get_model_state(model_name=model_name, relative_entity_name=reference_frame)
        self.goal_x = response.pose.position.x
        self.goal_y = response.pose.position.y
        # self.goal_z = response.pose.position.z
            
            
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
       
         
        return discretized_ranges,done

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        if not self.got_key:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            # Create a ModelState message
            model_state_msg = ModelState()

            # Set the name of the model to be changed
            model_state_msg.model_name = 'grey_wall'

            # Set the new pose of the model
            new_pose = Pose()
            # new_pose.position = Point(x=-20, y=-10, z=0.0) # Level 4
            # new_pose.position = Point(x=-2, y=-1, z=0.0) # Level 3
            new_pose.position = Point(x=4, y=1, z=0.0) # Level 2
            # new_pose.position = Point(x=2.85, y=-2.6, z=0.0)  # Level 1
            new_pose.orientation = Quaternion(x=0.0, y=0.0, z=0, w=1.0)
            # new_pose.orientation = Quaternion(x=0.0, y=0.0, z=1.57, w=1.0) # Level 1            
            model_state_msg.pose = new_pose
            resp = set_model_state(model_state_msg) 
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
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.3 # 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = -0.3  # -0.3
            self.vel_pub.publish(vel_cmd)


        #LASER
        
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass
                
        
        #HANDLE
        
        handle_contact =None
        if not self.got_key :
            while handle_contact is None:
                try:
                    handle_contact =rospy.wait_for_message('/my_contact_handle', ContactsState, timeout=5) 
                except:
                    pass
        
        
        #HANDLE
        
        person_contact =None
        if self.got_key :
           while person_contact is None:
                try:
                    person_contact = rospy.wait_for_message('/my_contact_person', ContactsState, timeout=5)
                except:
                    pass
        
       
        
        #TURTLEBOT POSITION
        
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        model_name = 'mobile_base'
        reference_frame = 'world'
        
        response = get_model_state(model_name=model_name, relative_entity_name=reference_frame)
        turtle_x = response.pose.position.x
        turtle_y = response.pose.position.y
        turtle_z = response.pose.position.z
        
        
        orientation = response.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        x=orientation.x
        y=orientation.y
        z=orientation.z
        w=orientation.w
        # _, _, yaw = euler_from_quaternion(orientation_list)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        goal_angle = math.atan2(self.goal_y - turtle_y, self.goal_x - turtle_x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi
        
        
        #if heading = 0 we are facing the target 
        
        heading = round(heading, 2)         # we want this close to 0 ( low abs value)
        
        self.heading = heading

        if heading != 0 :
            heading_reward = 5* 1 / abs(heading)  #5
        else :
            heading_reward = 100  #100
     
        
        #DISTANCE TO HANDLE
        
        goal_distance = math.sqrt((turtle_x - self.goal_x)**2 + (turtle_y - self.goal_y)**2)
        # Define the scale factor for the reward
        scale = 200  #100   200 4room
    
        # Compute the reward as a negative exponential of the distance
        distance_reward =scale* math.exp(- goal_distance*2)
        # print(distance_reward)
 
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed") 
            
            
        
      



        state,done = self.discretize_observation(data,2)
    
    
        state = state + [ goal_distance , heading ]
    
        
        self.steps += 1
        self.current_steps += 1

        
          #FOUND HANDLE!!!
        if not self.got_key :
            if handle_contact !=None:
                if handle_contact.states[0].collision2_name != "ground_plane::link::collision":     
                    self.got_key=True
                    self.num_keys += 1
                  
                    reward = 10000
                    
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
                    resp = set_model_state(model_state_msg) 

                    #WALL 
                    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                    # Create a ModelState message
                    model_state_msg = ModelState()
        
                    # Set the name of the model to be changed
                    model_state_msg.model_name = 'grey_wall'
        
                    # Set the new pose of the model
                    new_pose = Pose()
                    new_pose.position = Point(x=12.85, y=-12.6, z=0.0)
                    new_pose.orientation = Quaternion(x=0.0, y=0.0, z=0, w=1.0)
                    model_state_msg.pose = new_pose
                    resp = set_model_state(model_state_msg) 
                        # set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

                
    
        
       
        elif self.got_key:
            rospy.wait_for_service('/gazebo/get_model_state')
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

            model_name = "person_standing"
            reference_frame = 'world'
            
            response = get_model_state(model_name=model_name, relative_entity_name=reference_frame)
            self.goal_x = response.pose.position.x
            self.goal_y = response.pose.position.y
            
            if person_contact !=None:
                if person_contact.states[0].collision2_name != "ground_plane::link::collision":     
                    reward = 40000
                    self.reached_human= True
                    self.successes += 1
                    done = True
            
        
        if self.steps == 1000 :
            print ("KEYS: {} / {} ".format(self.num_keys,self.iterations))
            print(("{}  % " .format((self.num_keys / self.iterations) *100) ))
            print ("SUCCESSES : {} / {} ".format(self.successes,self.iterations))
            print(("{}  % " .format((self.successes / self.iterations) *100) ))
            self.steps = 0 
            self.iterations = 1
            self.num_keys = 0
            self.successes = 0
            self.current_steps = 0
        #DONE SHOULD BE AFTER  : 1. HANDLE , 2. DOOR , 3. HUMAN
        
        if not done:
            if  self.got_key_reward == False and self.got_key==True :     #reward for getting key (only once)     +++   self.got_key_reward == False and
                reward = 10000 #10000
                self.got_key_reward = True
            elif action == 0:
                reward = 100 #-0.5*self.current_steps  # +5   100
            else:
                reward =   -150 #-1*self.current_steps   # +1    -150
          
        else:
            self.iterations += 1
            if  self.got_key_reward == False and self.got_key==True :     #reward for getting key (only once)     +++   self.got_key_reward == False and
                reward = 10000
                self.got_key_reward = True
            if self.opened_door:                         #reward for opening the door with the key
                reward = 2000
            if self.reached_human:
                reward = 40000  #20000
            else:                                   # negative reward for crashing in the wall (or door without the key )
                reward = -10000 # -6000
                
       

        reward = reward + distance_reward + heading_reward
       
        return np.asarray(state,dtype=float), reward, done, {}

    def reset(self):
      
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        
     
        
        

        if self.got_key:
            set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
            # Create a ModelState message
            model_state_msg = ModelState()
    
            # Set the name of the model to be changed
            model_state_msg.model_name = 'door_handle'
    
            # Set the new pose of the model
            new_pose = Pose()
            # new_pose.position = Point(x=2.2, y=1.5, z=  0 )      # Level 4
            # new_pose.position = Point(x=2, y=2, z=  0 )      # Level 3
            new_pose.position = Point(x=4, y=-2, z=  0 )      # Level 2
            # new_pose.position = Point(x=2.5, y=-3.3, z=  0 )      # Level 1
            new_pose.orientation = Quaternion(x=0, y= 0 , z=0, w=1.0)
            model_state_msg.pose = new_pose
            resp = set_model_state(model_state_msg)

            
          
            
            self.got_key = False
            self.got_key_reward = False
            self.opened_door = False 
            self.reached_human = False
            
           

        self.got_key = False
        self.got_key_reward = False
        self.opened_door = False
        self.reached_human = False
        
        
        self.got_key = False
        self.got_key_reward = False
        self.opened_door = False 
        self.reached_human = False
        self.current_steps = 0

       
        # # Level 4
        # self.goal_x = 2.2
        # self.goal_y = 1.5

        # Level 3
        # self.goal_x = 2
        # self.goal_y = 2
        
        # # Level 2
        self.goal_x = 4
        self.goal_y = -2

        #Level 1
        # self.goal_x = 2.5
        # self.goal_y = -3.3
        
        self.heading = -1 
       
       
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass


        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        model_name = 'mobile_base'
        reference_frame = 'world'
        
        response = get_model_state(model_name=model_name, relative_entity_name=reference_frame)
        turtle_x = response.pose.position.x
        turtle_y = response.pose.position.y
        turtle_z = response.pose.position.z
        
        
        #DISTANCE TO HANDLE
        
        goal_distance = math.sqrt((turtle_x - self.goal_x)**2 + (turtle_y - self.goal_y)**2)
        
        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Create a ModelState message
        model_state_msg = ModelState()

        # Set the name of the model to be changed
        model_state_msg.model_name = 'grey_wall'

        # Set the new pose of the model
        new_pose = Pose()
        # new_pose.position = Point(x=-20, y=-10, z=0.0)  # Level 4
        # new_pose.position = Point(x=-2, y=-1, z=0.0)  # Level 3
        new_pose.position = Point(x=4, y=1, z=0.0)  # Level 2
        # new_pose.position = Point(x=2.85, y=-2.6, z=0.0)  # Level 1
        new_pose.orientation = Quaternion(x=0.0, y=0.0, z=0, w=1.0)
        # new_pose.orientation = Quaternion(x=0.0, y=0.0, z=1.57, w=1.0) # Level 1
        model_state_msg.pose = new_pose
        resp = set_model_state(model_state_msg) 

        state ,done= self.discretize_observation(data,2)
        
        state = state + [ goal_distance , self.heading]
     
        return np.asarray(state,dtype=float)
