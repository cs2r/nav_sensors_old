import pygame
import yaml, time
from pygame.locals import *
import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class display_sensors():
    def __init__(self):
        map_name = "lab_map2"
        with open(map_name + ".yaml") as file:
            self.map_conf = yaml.load(file)
        pygame.init()
        self.zoom = 2
        self.background = pygame.image.load(map_name + ".pgm")
        self.size = self.background.get_rect().size
        self.background = pygame.transform.scale(self.background, (self.size[0] * self.zoom, self.size[1] * self.zoom))
        self.size = self.background.get_rect().size
        self.heatbar = pygame.image.load("heatbar.png")
        self.heatbar = pygame.transform.scale(self.heatbar, (30, self.size[1]/4-20))
        self.barSize = self.heatbar.get_rect().size
        self.robot = pygame.image.load("robot.png")
        self.robot = pygame.transform.scale(self.robot, (16, 28))
        self.red = (255,0,0)
        self.screen = pygame.display.set_mode((self.size[0] + self.barSize[0], self.size[1] + 0))
        self.screen.blit(self.background, (0, 0))
        self.drow_goal = False
        self.ranges = [[0, 1000],
                        [400, 2000],
                        [0, 1000],
                        [0, 1000]]
        self.sensors = [0,400,0,0]
        self.sensors_name = ['VOC', 'CO2', 'pm2.5', 'pm10']
        rospy.init_node("air_sensor_map")
        rospy.Subscriber("/sensors", Int64MultiArray, self.callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_position)
        self.position = PoseWithCovarianceStamped()

        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def rgb(self, minimum, maximum, value):
        minimum, maximum = float(minimum), float(maximum)
        ratio = 2 * (value - minimum) / (maximum - minimum)
        b = int(max(0, 255 * (1 - ratio)))
        r = int(max(0, 255 * (ratio - 1)))
        g = 255 - b - r
        return r, g, b

    def get_level(self, min, max, value):
        level = -self.barSize[1]/float(max-min)*value + self.barSize[1] * (1 + min  / float(max-min))
        return int(level)


    def show_text(self, text, pos, size=20, color=(255,255,255)):
        font = pygame.font.SysFont("Times New Roman", size, True)
        textToShow = font.render(text, 1, color)
        text_rect = textToShow.get_rect()
        text_rect.center = pos
        self.screen.blit(textToShow, text_rect)
        #pygame.display.flip()

    def mtr2pix(self):
        resolution = self.map_conf["resolution"]
        origin = self.map_conf["origin"][:2]
        pos_m = [self.position.pose.pose.position.x, self.position.pose.pose.position.y]
        pos_px = [int((a - b) * self.zoom / resolution) for a, b in zip(pos_m, origin)]
        pos_px[1] = self.size[1] - pos_px[1]
        #print pos_px
        return pos_px

    def callback(self, data):
        self.sensors = data.data

    def get_position(self, data=None):
        if data is not None:
            self.position = data
            '''
            quaterniom = [self.position.pose.pose.orientation.x,self.position.pose.pose.orientation.y,
                          self.position.pose.pose.orientation.z,self.position.pose.pose.orientation.w]
            print self.position.pose.pose'''
        pos_px = self.mtr2pix()
        self.screen.blit(self.background, (0, 0))
        self.screen.blit(self.robot, (pos_px[0] - 8, pos_px[1] - 28))
        if self.drow_goal:
            pygame.draw.circle(self.screen, self.red, self.pos, 3)
            pygame.draw.line(self.screen, self.red, self.pos, self.pos2)
        row = 20
        for i in range(len(self.sensors)):
            level = self.get_level(self.ranges[i][0], self.ranges[i][1], self.sensors[i])
            clr = self.rgb(self.ranges[i][0], self.ranges[i][1], self.sensors[i])
            self.show_text(self.sensors_name[i], (self.size[0] + 15, row-10), 12)
            self.screen.blit(self.heatbar, (self.size[0], row))
            pygame.draw.line(self.screen, clr, (self.size[0]-10, level+row), (self.size[0]+self.barSize[0], level+row))
            self.show_text(str(self.sensors[i]), (self.size[0] - 20, level + row), color=clr)
            row += (20+self.barSize[1])
        pygame.display.update()

    def set_position(self):
        self.pos = pygame.mouse.get_pos()
        pygame.draw.circle(self.screen, self.red, self.pos, 3)
        pygame.display.update()

    def set_goal(self):
        self.pos2 = pygame.mouse.get_pos()
        self.drow_goal = True
        self.yaw = np.arctan2(self.pos[1]-self.pos2[1], self.pos2[0]-self.pos[0])
        self.orientation = quaternion_from_euler(0, 0, self.yaw)
        self.position_M = self.pix2mtr()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position.x = self.position_M[0]
        goal.target_pose.pose.position.y = self.position_M[1]
        goal.target_pose.pose.position.z = self.position_M[2]

        goal.target_pose.pose.orientation.x = self.orientation[0]
        goal.target_pose.pose.orientation.y = self.orientation[1]
        goal.target_pose.pose.orientation.z = self.orientation[2]
        goal.target_pose.pose.orientation.w = self.orientation[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return 'success'

    def pix2mtr(self):
        resolution = self.map_conf["resolution"]
        origin = self.map_conf["origin"][:2]
        pos_px = [self.pos[0], self.size[1]-self.pos[1]]
        pos_m = [a*resolution/self.zoom + b  for a, b in zip(pos_px, origin)]
        #print pos_m
        return (pos_m[0], pos_m[1], 0.0)

if __name__ == "__main__":
    maps = display_sensors()
    try:

        while True:
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONDOWN:
                    maps.set_position()
                    pressed = True
                    while pressed:
                        for event in pygame.event.get():
                            if event.type == pygame.MOUSEBUTTONUP:
                                maps.set_goal()
                                pressed = False
                                maps.get_position()

            time.sleep(0.05)
    except KeyboardInterrupt, rospy.ROSInterruptException:
        pass
