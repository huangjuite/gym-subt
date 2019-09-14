import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

STEP_TIME = 0.02


class SubtEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        rospy.init_node('gym_subt')
        self.laser_upper = LaserScan()
        self.laser_lower = LaserScan()
        self.image = CompressedImage()
        self.cv_bridge = CvBridge()

        # start position
        self.state_msg = ModelState()
        self.state_msg.model_name = 'X1'
        self.state_msg.pose.position.x = 158.66
        self.state_msg.pose.position.y = 0
        self.state_msg.pose.position.z = -19.8
        self.state_msg.pose.orientation.x = 0
        self.state_msg.pose.orientation.y = 0
        self.state_msg.pose.orientation.z = 0
        self.state_msg.pose.orientation.w = -1

        # [Twist.linear.x, Twist.angular.z]
        self.actions = [[0.75, -0.8],
                        [1.5, -0.8],
                        [1.5, -0.4],
                        [1.5, 0],
                        [1.5, 0.4],
                        [1.5, 0.8],
                        [0.75, 0.8]]
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
        self.reward += 0.05
        for i, dis in enumerate(laser):
            dis = 10-dis
            if dis > 1.5 and dis < 2.5:
                self.reward -= 0.01
            elif dis > 0.8 and dis < 1.5:
                self.reward -= 0.03
            elif dis < 0.8:
                done = True
        if done:
          self.reward = -100
        
        # self.pause_physics()
        return np.array(laser), self.reward, done, info

    def reset(self):
        self.reset_model(self.state_msg)
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
                dis = 10
            laser.append(10-dis)
        for i, dis in enumerate(laser_l):
            if dis == np.inf:
                dis = 10
            laser.append(10-dis)
        return laser

    def render(self, mode='human'):
        pass

    def close(self):
        self.unpause_physics()
        rospy.signal_shutdown('WTF')
