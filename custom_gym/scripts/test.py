import rospy
import time
import gym
from gym import spaces
import numpy as np
from custom_gym.srv import *

stepper = rospy.ServiceProxy('stepper', StepFunction)




stepper(1)

time.sleep(2)
stepper(1)

