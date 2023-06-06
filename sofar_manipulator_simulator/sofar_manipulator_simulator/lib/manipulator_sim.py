import arcade
from .robot import RobotArm
import random
import numpy as np
import math

from threading import Thread

SCREEN_W = 800
SCREEN_H = 600
TITLE = "Manipulator Simulator"

LINK1_LENGTH = 100
LINK2_LENGTH = 80
LINK3_LENGTH = 50

R_MIN = 130
R_MAX = 190

# Thread to run simulation in background
class SimThread(Thread):
   def __init__(self):
      Thread.__init__(self)
   
   def run(self):
      arcade.run()

# Simulation class
class ManipulatorSim(arcade.Window):

    def __init__(self, resources_path):
        super().__init__(SCREEN_W, SCREEN_H, TITLE)        
        self.background_color = arcade.color.WHITE

        # Package path to retrive resources
        self.resources_path = resources_path

        # Utility variables for controlling grasped ball
        self.red_grasped = False
        self.blue_grasped = False

        # sim thread object
        self.thread = SimThread()

    def face_to_sign(self, idx, axis='x'):
        if idx == 1 or (idx == 2 and axis == 'y') or (idx == 4 and axis == 'x'):
            return 1
        elif idx == 3 or (idx == 2 and axis == 'x') or (idx == 4 and axis == 'y'):
            return -1

    def setup(self):
        # Define robot origin (mid-screen)
        self.origin = (SCREEN_W/2, SCREEN_H/2)
        # Instantiate robot 
        self.robot = RobotArm(
            self.origin, 
            LINK1_LENGTH, 
            LINK2_LENGTH, 
            LINK3_LENGTH, 
            self.resources_path
        )
        # Instantiate red and blue balls and corresponding targets
        red_ball = arcade.Sprite(self.resources_path + "/resource/red.png", scale=0.9)
        blue_ball = arcade.Sprite(self.resources_path + "/resource/blue.png", scale=0.9)
        self.balls = [red_ball, blue_ball]
        red_target = arcade.Sprite(self.resources_path + "/resource/red_out.png", scale=0.9)
        blue_target = arcade.Sprite(self.resources_path + "/resource/blue_out.png", scale=0.9)
        self.targets = [red_target, blue_target]
        # Randomly place balls and targets on the map
        r = random.randint(R_MIN, R_MAX)
        alpha = math.radians(random.randint(10, 70))
        faces = [1, 2, 3, 4]
        random.shuffle(faces)
        for i in range(2):
            self.balls[i].center_x = self.origin[0] + self.face_to_sign(faces[2*i], 'x') * r * math.cos(alpha)
            self.balls[i].center_y = self.origin[1] + self.face_to_sign(faces[2*i], 'y') * r * math.sin(alpha)
            self.targets[i].center_x = self.origin[0] + self.face_to_sign(faces[2*i+1], 'x') * r * math.cos(alpha)
            self.targets[i].center_y = self.origin[1] + self.face_to_sign(faces[2*i+1], 'y') * r * math.sin(alpha)

    # Method to let robot grasp closest ball
    def grasp(self):
        # Find closest ball
        distances = []
        for ball in self.balls:
            bx = ball.center_x
            by = ball.center_y
            b = np.array([bx, by])
            eex = self.robot.ee.center_x
            eey = self.robot.ee.center_y
            ee = np.array([eex, eey])
            distances.append(np.linalg.norm(b - ee))
        if any(item < 25 for item in distances):
            closest_idx = np.argmin(distances)
            if closest_idx == 0:
                self.red_grasped = True
            elif closest_idx == 1:
                self.blue_grasped = True
    
    # Method to drop whichever grasped ball
    def drop(self):
        self.red_grasped = False
        self.blue_grasped = False

    # On draw
    def on_draw(self):
        self.clear()
        # Render robot's arm
        self.robot.render()
        for ball in self.balls:
            ball.draw()
        for target in self.targets:
            target.draw()

    def on_update(self, delta_time: float):
        if self.red_grasped:
            self.balls[0].center_x = self.robot.ee.center_x + 10 * math.cos(math.radians(self.robot.ee.angle))
            self.balls[0].center_y = self.robot.ee.center_y + 10 * math.sin(math.radians(self.robot.ee.angle))
        if self.blue_grasped:
            self.balls[1].center_x = self.robot.ee.center_x + 10 * math.cos(math.radians(self.robot.ee.angle))
            self.balls[1].center_y = self.robot.ee.center_y + 10 * math.sin(math.radians(self.robot.ee.angle))