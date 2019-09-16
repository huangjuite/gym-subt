import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

STEP_TIME = 0.01

# [x,y,z,x,y,z,w]
INITIAL_STATES = [[14.38, -0.05, -0.74, 0, 0.14, 0, 1],
                  [89.1, 0, -19.87, 0, 0, 0, -1],
                  [100, 22.5, -20, 0, 0, -0.7, -0.7],
                  [100, 63.13, -20, 0, 0, -0.7, -0.7],
                  [80, 51, -20, 0, 0, -0.74, 0.66],
                  [159, 0, -20, 0, 0, 0, -1]]


class SubtEnv(gym.Env):
    metadata = {'render.modes': ['laser']}

    def __init__(self):
        rospy.init_node('gym_subt')
        self.laser_upper = LaserScan()
        self.laser_lower = LaserScan()
        self.image = CompressedImage()
        self.cv_bridge = CvBridge()
        self.fig, self.axs = plt.subplots(2,1)

        # [Twist.linear.x, Twist.angular.z]
        self.actions = [[0.5, -0.8],
                        [1.5, -0.8],
                        [1.5, -0.4],
                        [1.5, 0.0],
                        [1.5, 0.4],
                        [1.5, 0.8],
                        [0.5, 0.8]]
        self.reward = 0
        self.laser_len = 42

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)

        self.pub_twist = rospy.Publisher('/X1/cmd_vel', Twist, queue_size=1)
        self.sub_laser_upper = rospy.Subscriber(
            '/RL/scan/upper', LaserScan, self.cb_laser_upper, queue_size=1)
        self.sub_laser_lower = rospy.Subscriber(
            '/RL/scan/lower', LaserScan, self.cb_laser_lower, queue_size=1)
        # self.sub_image_raw = rospy.Subscriber('X1/rgbd_camera/rgb/image_raw/compressed',CompressedImage,self.cb_image,queue_size=1)
        self.reset()

    def get_initial_state(self, id):
        # start position
        state_msg = ModelState()
        state_msg.model_name = 'X1'
        state_msg.pose.position.x = INITIAL_STATES[id][0]
        state_msg.pose.position.y = INITIAL_STATES[id][1]
        state_msg.pose.position.z = INITIAL_STATES[id][2]
        state_msg.pose.orientation.x = INITIAL_STATES[id][3]
        state_msg.pose.orientation.y = INITIAL_STATES[id][4]
        state_msg.pose.orientation.z = INITIAL_STATES[id][5]
        state_msg.pose.orientation.w = INITIAL_STATES[id][6]
        return state_msg

    def cb_image(self, msg):
        self.image = msg

    def cb_laser_upper(self, msg):
        self.laser_upper = msg

    def cb_laser_lower(self, msg):
        self.laser_lower = msg

    def step(self, action):
        # self.unpause_physics()

        cmd_vel = Twist()
        cmd_vel.linear.x = self.actions[action][0]
        cmd_vel.angular.z = self.actions[action][1]
        self.pub_twist.publish(cmd_vel)

        time.sleep(STEP_TIME)

        done = False
        info = None
        laser = self.get_observation()

        # reward design
        for i, dis in enumerate(laser):
            # dis = 10-dis
            if dis > 2.2:
                self.reward += 1
            elif dis > 1.5 and dis < 2.2:
                self.reward -= 1
            elif dis > 0.9 and dis < 1.5:
                self.reward -= 3
            elif dis < 0.9:
                done = True
        if done:
            self.reward = -10000

        # self.pause_physics()
        return np.array(laser), self.reward, done, info

    def reset(self):
        self.reset_model(self.get_initial_state(
            np.random.randint(0, len(INITIAL_STATES))))
        # self.unpause_physics()
        self.reward = 0
        rospy.loginfo('reset model')
        time.sleep(0.5)
        return np.array(self.get_observation())

    def get_observation(self):
        # image_np = self.cv_bridge.compressed_imgmsg_to_cv2(self.image)
        laser_u = list(self.laser_upper.ranges)
        laser_l = list(self.laser_lower.ranges)
        laser = []
        for i, dis in enumerate(laser_u):
            if dis == np.inf:
                dis = 100
            laser.append(dis)
        for i, dis in enumerate(laser_l):
            if dis == np.inf:
                dis = 100
            laser.append(dis)
        return laser

    def render(self, mode='laser'):
        observation = self.get_observation()
        self.axs[0].set_title("upper laser")
        self.axs[1].set_title("lower laser")
        self.axs[0].clear()
        self.axs[0].plot(observation[:21])
        self.axs[1].clear()
        self.axs[1].plot(observation[21:])
        plt.pause(0.001)
        
    def close(self):
        self.unpause_physics()
        rospy.signal_shutdown('WTF')
