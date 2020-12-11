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







class P9RLEnv(gym.Env):

    def __init__(self):
        self.safetyLimit = 1
        self.collisionParam = 1
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

        self.pub = rospy.Publisher('/vehicle_blue/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/syscommand', String, queue_size=1)
        self.maxAngularAction = 60

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.sub_odom = rospy.Subscriber('/vehicle_blue/odom', Odometry, self.getOdometry)

        self.action_space = spaces.Box(low=np.array([-self.maxAngularAction]),
                                       high=np.array([self.maxAngularAction]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(4099,), dtype=np.float16)

    def getOdometry(self, odom):

        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        return self.position, self.yaw



    def reset(self):
        # RESET ENVIRONMENT

        self.TimeoutCounter = 0

        state_msg = ModelState()
        state_msg.model_name = 'vehicle_blue'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        rospy.wait_for_service('/gazebo/set_model_state')


        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

        self.pub2.publish("reset")

        self.done = False



        self.unpause_proxy()
        time.sleep(3)
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=1)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1)

        self.pause_proxy()
        self.scan_range, minScan = self.scanRange(scan)

        self.rewardMapOld = 96
        state, self.done = self.setStateAndDone(gmap, scan)





        return state

    def step(self, action):

        self.unpause_proxy()


        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()



        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "vehicle_blue/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 1 * math.cos(action[0])
        goal.target_pose.pose.position.y = 1 * math.sin(action[0])

        quat = quaternion_from_euler(0, 0, action[0])
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            time.sleep(1)
            self.client.wait_for_result()

        self.client.get_result()
        self.unpause_proxy()

        scan = rospy.wait_for_message("/scan", LaserScan, timeout=1000)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()
        state, self.done = self.setStateAndDone(gmap, scan)
        reward = self.setReward(self.done, gmap)
        print("step")
        return [state, reward, self.done, {}]

    def setReward(self, done, gmap):

        self.rewardMap = np.sum(np.array(gmap.data) > -1, axis=0)
        self.reward = self.rewardMap - self.rewardMapOld
        self.rewardMapOld = self.rewardMap

        #self.reward += self.rewardObstacleProximity()
        if done:
            self.reward += -100
        return self.reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, gmap, scan):

        # # Set state and done

        done = False
        self.scan_range, minScan = self.scanRange(scan)
        if minScan < self.collisionParam or self.TimeoutCounter == 100:
            done = True

        self.splitZones(gmap)
        state = self.zoneValue + [self.position.x, self.position.y, self.yaw]

        return state, done



    def rewardObstacleProximity(self):
        closestObstacle = min(self.scan_range)
        if closestObstacle <= self.safetyLimit:
            return self.obsProximityParam * -(1 - (closestObstacle / self.safetyLimit))
        else:
            return 0
    def scanRange(self, scan):
    #
        scan_range = []

        for x in range(0, 360, self.lidarDiscretization):
            end = x + self.lidarDiscretization
            scan_range.append(min(scan.ranges[x:end]))

        minScan = min(list(filter(lambda a: a != 0, scan_range[:])))
        return scan_range, minScan

    def splitZones(self, gmap):
        self.zoneValue = []
        arr = np.asarray(gmap.data)
        chunks = np.array_split(arr, 4096)

        for x in range(4096):
            self.zoneValue.append(np.sum(a=chunks[x], axis=0, dtype=np.int32))


