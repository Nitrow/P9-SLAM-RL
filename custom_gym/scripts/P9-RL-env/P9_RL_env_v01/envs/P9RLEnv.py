import rospy
import gym
from gym import spaces
import numpy as np
from tf import TransformListener
import os
from nav_msgs.msg import Odometry
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, geometry_msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, actionlib_msgs
import math
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import sys
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
np.set_printoptions(threshold=sys.maxsize)

class P9RLEnv(gym.Env):

    def __init__(self):
        self.safetyLimit = 1
        self.collisionParam = 0.8
        self.obsProximityParam = 5
        self.scan_range = []
        rospy.init_node('RLEnv', anonymous=True)
        self.tf = TransformListener()
        self.position = []
        self.quaternion = []
        self.pubAction = Twist()
        self.reward = 0
        self.data = []
        self.state = []
        self.data2 = []
        self.action = []
        self.rewardMapOld = 0
        self.done = False
        self.TimeoutCounter = 0
        self.lidarDiscretization = 30
        self.minScan = 0

        map_width = 0
        map_height = 0

        current_x = 0
        current_y = 0

        map_x_origin = 0
        map_y_origin = 0

        x_in_map = 0
        y_in_map = 0

        resolution = 0

        horizontal_img = np.array([])


        self.pub = rospy.Publisher('/vehicle_blue/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/syscommand', String, queue_size=1)
        self.xaction = 0.5
        self.yaction = 0.5

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.Subscriber('/vehicle_blue/odom', Odometry, self.getOdometry)
        rospy.Subscriber("/map_metadata", MapMetaData, self.meta_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.action_space = spaces.Box(low=np.array([-self.xaction, -self.yaction]),
                                       high=np.array([self.xaction, self.yaction]), dtype=np.float16)
        self.observation_space = spaces.Box(low=0, high=255, shape=(64, 64, 3), dtype=np.uint8)



    def meta_callback(self, data):
        self.map_width = data.width
        self.map_height = data.height
        self.map_x_origin = data.origin.position.x
        self.map_y_origin = data.origin.position.y
        self.resolution = data.resolution

    def map_callback(self, data):

        self.map = np.array(data.data)

        map_data = np.array(data.data)

        if (self.map_width * self.map_height != 0):
            # turn 1D array into 2D array for image
            map_data = np.reshape(map_data, (self.map_height, self.map_width))

            # convert values in costmap
            map_data[map_data == -1] = 150
            map_data[map_data == 100] = 255
            # Add square where robot is

            for i in range(2):
                for j in range(2):
                    map_data[int(self.y_in_map) + i, int(self.x_in_map) + j] = 255

            im = np.array(map_data, dtype=np.uint8)

            img = cv2.resize(im, (64, 64))

            img2 =  cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


            # resize and flip image
            self.horizontal_img = cv2.flip(img2, 0)


            dim = (256, 256)

            resized = cv2.resize(self.horizontal_img, dim, interpolation=cv2.INTER_AREA)
            cv2.imshow('image', resized)
            cv2.waitKey(2)

    def getOdometry(self, data):
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        # Convert position to be relative to bottom left of map
        self.x_in_map = (self.current_x + 15) / self.resolution
        self.y_in_map = (self.current_y + 15) / self.resolution


    def lidar_callback(self, data):
        self.minScan = min(list(filter(lambda a: a != 0, data.ranges[:])))


    def reset(self):
        # RESET ENVIRONMENT
        self.client.cancel_all_goals()
        self.TimeoutCounter = 0

        state_msg = ModelState()
        state_msg.model_name = 'vehicle_blue'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.35
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        rospy.wait_for_service('/gazebo/set_model_state')


        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

        self.pub2.publish("reset")
        self.done = False

        self.rewardMapOld = 98

        self.unpause_proxy()
        time.sleep(0.1)
        self.pause_proxy()
        state = self.horizontal_img

        return state

    def step(self, action):

        self.TimeoutCounter += 1
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "vehicle_blue/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        #goal.target_pose.pose.position.x = 0.5 * math.cos(action[0])
        goal.target_pose.pose.position.x = action[0]
        goal.target_pose.pose.position.y = action[1]

        goal_angle = math.atan2(action[0], action[1])

        quat = quaternion_from_euler(0, 0, goal_angle)
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.client.send_goal(goal)

        self.unpause_proxy()

        self.client.wait_for_result()
        self.client.get_result()

        self.pause_proxy()
        state = self.horizontal_img
        reward = self.setReward()
        if self.TimeoutCounter == 200000 or self.minScan < self.collisionParam:
            self.done = True

        return [state, float(reward), self.done, {}]

    def setReward(self):
        self.rewardMap = np.sum(np.array(self.map) > -1)
        if self.rewardMap == self.rewardMapOld:
            self.reward += -1

        self.reward = self.rewardMap - self.rewardMapOld
        self.rewardMapOld = self.rewardMap

        return self.reward

    def render(self, mode='human'):
        pass