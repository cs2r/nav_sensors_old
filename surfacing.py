import pygame
import yaml, time
from pygame.locals import *
import rospy
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class display_sensors():
    def __init__(self):
        map_name = "lab_map2"
        with open(map_name + ".yaml") as file:
            self.map_conf = yaml.load(file)
        pygame.init()
        self.background = pygame.image.load(map_name + ".pgm")
        self.size = self.background.get_rect().size
        self.heatbar = pygame.image.load("heatbar.png")
        self.heatbar = pygame.transform.scale(self.heatbar, (30, self.size[1]))
        self.barSize = self.heatbar.get_rect().size
        self.red = (255,0,0)
        self.screen = pygame.display.set_mode((2 * self.size[0] + 2 * self.barSize[0], 2 * self.size[1] + 100))

        self.screen.blit(self.background, (0, 0))
        self.screen.blit(self.heatbar, (self.size[0], 50))
        self.show_text("VOC", (self.size[0] / 2, 25))
        vocRange = [0, 1000]

        self.screen.blit(self.background, (self.size[0] + self.barSize[0], 50))
        self.screen.blit(self.heatbar, (2 * self.size[0] + self.barSize[0], 50))
        self.show_text("CO2", (self.size[0] + self.size[0] / 2, 25))
        co2Range = [400, 2000]

        self.screen.blit(self.background, (0, self.size[1] + 100))
        self.screen.blit(self.heatbar, (self.size[0], self.size[1] + 100))
        self.show_text("pm2.5", (self.size[0] / 2, self.size[1] + 75))
        pm25Range = [0, 1000]

        self.screen.blit(self.background, (self.size[0] + self.barSize[0], self.size[1] + 100))
        self.screen.blit(self.heatbar, (2 * self.size[0] + self.barSize[0], self.size[1] + 100))
        self.show_text("pm10", (self.size[0] + self.size[0] / 2, self.size[1] + 75))
        pm10Range = [0, 1000]

        rospy.init_node("air_sensor_map")
        rospy.Subscriber("/sensors", Int64MultiArray, self.callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_position)
        self.position = PoseWithCovarianceStamped()

    def rgb(self, minimum, maximum, value):
        minimum, maximum = float(minimum), float(maximum)
        ratio = 2 * (value - minimum) / (maximum - minimum)
        b = int(max(0, 255 * (1 - ratio)))
        r = int(max(0, 255 * (ratio - 1)))
        g = 255 - b - r
        return r, g, b


    def show_text(self, text, pos):
        font = pygame.font.SysFont("Times New Roman", 40)
        textToShow = font.render(text, 1, self.red)
        text_rect = textToShow.get_rect()
        text_rect.center = pos
        self.screen.blit(textToShow, text_rect)
        pygame.display.flip()

    def mtr2pix(self):
        resolution = self.map_conf["resolution"]
        origin = self.map_conf["origin"][:2]
        pos_m = [self.position.pose.pose.position.x, self.position.pose.pose.position.y]
        pos_px = [int((a - b) / resolution) for a, b in zip(pos_m, origin)]
        pos_px[1] = self.size[1] - pos_px[1]
        #print pos_px
        return pos_px

    def callback(self, data):
        sensors = data.data


    def get_position(self, data):
        self.position = data
        pos_px = self.mtr2pix()
        pygame.draw.circle(self.screen, self.red, pos_px, 2)

if __name__ == "__main__":
    try:
        maps = display_sensors()
        maps.mtr2pix()
        while True:
            pygame.display.flip()
            time.sleep(0.1)
    except KeyboardInterrupt, rospy.ROSInterruptException:
        pass
