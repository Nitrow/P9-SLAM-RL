import rospy
import gym
from gym import spaces
import numpy as np
from custom_gym.srv import StepFunction

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class P9RLEnv(gym.Env):

    def __init__(self):
        rospy.init_node('RLEnv', anonymous=True)
        self.pubAction = Twist()
        self.reward = 0
        self.data = []
        self.state = []
        self.data2 = []
        self.action = []
        self.pub = rospy.Publisher('/model/vehicle_blue/cmd_vel', Twist, queue_size=10)
        #rospy.wait_for_service('/stepper')
        #self.stepper = rospy.ServiceProxy('/stepper', StepFunction, persistent=True)
        self.maxAngSpeed = 1
        self.maxLinSpeed = 1

        self.action_space = spaces.Box(low=np.array([0, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(4,), dtype=np.int8)
        rospy.spin()

    def reset(self):
        # RESET ENVIRONMENT
        self.state = [1,1]
        return np.asarray(self.state)

    def step(self, action):
        self.pubAction.linear.x = 100
        self.pubAction.angular.z = 100
        self.pub.publish(self.pubAction)


        #os.system("ign service -r -i -s /world/diff_drive/control --reqtype ignition.msgs.WorldControl --reptype "
        #          "ignition.msgs.Boolean --timeout 1000 --req 'pause: true, multi_step: 1'")




        #self.data = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)
        #self.data2 = rospy.wait_for_message("/lidar", LaserScan, timeout=5)


        # Get reward
        self.state, done = self.setStateAndDone(self.data, self.data2)
        reward = self.setReward(self.state, done)

        return [np.asarray(self.state), reward, done, {}]

    def setReward(self, state, done):
        #reward = np.sum(a=state, axis=0, dtype=np.int8)
        self.reward += -1
        if done:
            self.reward += 10000000000
        return self.reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, data, data2):
        state = data, data2
        done = False
        # Set state and done
        return state, done
