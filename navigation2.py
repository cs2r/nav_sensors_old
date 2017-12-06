import pyglet, yaml, rospy
import numpy as np
from pyglet.window import mouse
from GameObject import GameObject
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class display_sensors(pyglet.window.Window):
    def __init__(self, name, range, *args, **kwargs):
        self.name = name
        self.range = range
        super(display_sensors, self).__init__(*args, **kwargs)
        map_name = "lab_map3"
        with open(map_name + ".yaml") as file:
            self.map_conf = yaml.load(file)
        map = pyglet.image.load(self.map_conf["image"])
        heatbar = pyglet.image.load("heatbar.png")
        self.map = pyglet.sprite.Sprite(map, x=0, y=0)
        self.heatbar = pyglet.sprite.Sprite(heatbar, x=map.width, y=0)
        self.height = self.heatbar.height
        print self.height
        self.set_caption(name)
        self.set_size(map.width + heatbar.width, map.height)
        self.robot = GameObject(0, 0, "robot.png")
        self.pos = []
        self.pxPos = [[], []]
        self.color = []


        rospy.Subscriber(self.name, Int64, self.callback)
        self.sensor = 0
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_position)
        self.position = PoseWithCovarianceStamped()
        self.waypoints = []
        self.frame_id = rospy.get_param('~goal_frame_id', 'map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')


    def callback(self, data):
        self.sensor = data.data
        pos = self.mtr2pix()
        '''
        for position in self.pos:
            dis = (pos[0]-position[0])**2 + (pos[0]-position[0])**2
            print dis'''
        self.pos.append(self.mtr2pix())
        self.color.append(self.rgb(self.range[0], self.range[1], self.sensor))

    def mtr2pix(self):
        resolution = self.map_conf["resolution"]
        origin = self.map_conf["origin"][:2]
        pos_m = [self.position.pose.pose.position.x, self.position.pose.pose.position.y]
        pos_px = [int((a - b) / resolution) for a, b in zip(pos_m, origin)]
        return pos_px

    def pix2mtr(self):
        resolution = self.map_conf["resolution"]
        origin = self.map_conf["origin"][:2]
        pos_m = [a*resolution + b  for a, b in zip(self.pxPos[0], origin)]
        #print pos_m
        return (pos_m[0], pos_m[1], 0.0)

    def rgb(self, minimum, maximum, value):
        minimum, maximum = float(minimum), float(maximum)
        ratio = 2 * (value - minimum) / (maximum - minimum)
        b = int(max(0, 255 * (1 - ratio)))
        r = int(max(0, 255 * (ratio - 1)))
        g = 255 - b - r
        return [r, g, b]

    def get_position(self, data):
        self.position = data

    def draw_mark(self, x, y, width, color):
        mark = pyglet.graphics.vertex_list(4, ('v2i', (x-width,y ,x,y-width, x+width,y, x,y+width)),
                                           ('c3B', tuple(color*4)))
        mark.draw(pyglet.gl.GL_QUADS)

    def text(self, text, posx, posy, clr):
        label = pyglet.text.Label(text,
                                  font_name='Times New Roman',
                                  font_size=24,
                                  color=(clr[0], clr[1], clr[2], 255),
                                  x=posx, y=posy,
                                  anchor_x='right', anchor_y='center')
        label.draw()

    def get_level(self, min, max, value):
        level = self.height/float(max-min)*value - self.height/float(max-min)*min
        return int(level)

    def on_draw(self):
        self.clear()
        self.map.draw()
        self.heatbar.draw()
        self.robot.draw()
        if (self.pos != []) & (self.color != []):
            for i in range(len(self.pos)):
                self.draw_mark(self.pos[i][0], self.pos[i][1], 3, self.color[i])
        self.text(str(self.sensor) + ">>", self.map.width, self.get_level(self.range[0], self.range[1], self.sensor), self.rgb(self.range[0], self.range[1], self.sensor))
        if self.pxPos[1]:
            self.draw_mark(self.pxPos[0][0], self.pxPos[0][1], 5, (0, 255, 0))
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (self.pxPos[0][0], self.pxPos[0][1],
                                                                 self.pxPos[1][0], self.pxPos[1][1])),
                                                        ('c3B', tuple([255, 0, 255]*2)))


    def update(self,dt):
        pos = self.mtr2pix()
        self.robot.set_pos(pos[0] - self.robot.get_width()/2, pos[1] - self.robot.get_height()/2)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        #if buttons & mouse.LEFT:
        self.pxPos[1] = [x, y]

    def on_mouse_press(self, x, y, button, modifiers):
        self.pxPos[0] = [x, y]


    def on_mouse_release(self, x, y, button, modifiers):
        self.yaw = np.arctan2(self.pxPos[1][1] - self.pxPos[0][1], self.pxPos[1][0] - self.pxPos[0][0])
        self.orientation = quaternion_from_euler(0, 0, self.yaw)
        print self.orientation
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
        #if button & mouse.RIGHT:
        #    self.waypoints.append(goal)

'''
    def waypoint(self, dt):
        if self.waypoints:
            goal = self.waypoints.pop(0)
            self.client.send_goal(goal)
            self.client.wait_for_result()
'''

if __name__ == "__main__":
    sensors_name = ['VOC', 'CO2', 'pm2_5', 'pm10']
    ranges = [[0, 1000], [400, 2000], [0, 1000], [0, 1000]]
    rospy.init_node("air_sensor_map")
    window={}
    i=0
    for name in sensors_name:
        window[name] = display_sensors(name, ranges[i])
        pyglet.clock.schedule_interval(window[name].update, 1)
        #pyglet.clock.schedule_interval(window[name].waypoint, 0.1)
        i+=1
    pyglet.app.run()