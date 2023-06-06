import arcade
import math
import numpy as np

class Link(arcade.Sprite):

    def __init__(self, x: float, y: float, length: float, resource: str):
        super().__init__(resource)   
        self.center_x  = x
        self.center_y  = y
        self.length = length

    def render(self):
        self.draw()


class RobotArm:

    def __init__(self, spawn_position: tuple, L1: float, L2: float, L3: float, resources_path: str):
        self.o_x = float(spawn_position[0])
        self.o_y = float(spawn_position[1])

        self.resources_path = resources_path

        # Define links and respective positions
        self.L1 = Link(self.o_x + L1/2, self.o_y, L1, self.resources_path + "/resource/l1.png")
        self.L2 = Link(self.o_x + L1 + L2/2, self.o_y, L2, self.resources_path + "/resource/l2.png")
        self.L3 = Link(self.o_x + L1 + L2 + L3/2, self.o_y, L3, self.resources_path + "/resource/l3.png")
        # Define end effector
        self.ee = arcade.Sprite(
            self.resources_path + "/resource/ee.png",
            center_x = self.L3.center_x + L3/2 + 2.5,
            center_y = self.L3.center_y
        )
        # Define joint angles
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        # Define max iterations for IK computation
        self.max_iterations = 1000
        # IK error tolerance
        self.tolerance = 0.005

    # Explicitely rotate links to comply with joint configuration
    def set_joint_angles(self, q1: float, q2: float, q3: float):
        self.L1.angle = math.degrees(q1)
        self.L1.center_x = self.o_x + self.L1.length/2 * math.cos(q1)
        self.L1.center_y = self.o_y + self.L1.length/2 * math.sin(q1)

        self.L2.center_x = self.o_x + self.L1.length * math.cos(q1) + self.L2.length/2 * math.cos(q2)
        self.L2.center_y = self.o_y + self.L1.length * math.sin(q1) + self.L2.length/2 * math.sin(q2)
        self.L2.angle = math.degrees(q2)

        self.L3.center_x = self.o_x + self.L1.length * math.cos(q1) + self.L2.length * math.cos(q2) + \
            self.L3.length/2 * math.cos(q3)
        self.L3.center_y = self.o_y + self.L1.length * math.sin(q1) + self.L2.length * math.sin(q2) + \
            self.L3.length/2 * math.sin(q3)
        self.L3.angle = math.degrees(q3)
        

        self.ee.center_x = self.L3.center_x + (self.L3.length/2 + 2.5) * math.cos(q3)
        self.ee.center_y = self.L3.center_y + (self.L3.length/2 + 2.5) * math.sin(q3)
        self.ee.angle = math.degrees(q3)

        
    # Compute end-effector's pose based on current joint configuration
    def forward_kinematics(self, a1, a2, a3):
        x_ee = self.L1.length * math.cos(a1) + self.L2.length * math.cos(a1 + a2) + \
              self.L3.length * math.cos(a1 + a2 + a3)
        y_ee = self.L1.length * math.sin(a1) + self.L2.length * math.sin(a1 + a2) + \
              self.L3.length * math.sin(a1 + a2 + a3)
        theta_ee = a1 + a2 + a3
        return x_ee, y_ee, theta_ee

    # Compute desired joint angles to reach 2D end-effector's pose
    def inverse_kinematics(self, x_d: float, y_d: float, theta_d: float):
        # Iteratively solve IK problem...
        iteration = 0
        # Copy joint angles values
        a1 = self.q1
        a2 = self.q2
        a3 = self.q3
        # Get links lengths
        L1 = self.L1.length
        L2 = self.L2.length
        L3 = self.L3.length
        # Add minor offset
        if x_d > 0:
            x_d -= 10
        else:
            x_d += 10
        while True:
            # Get forward kinematics
            x_ee, y_ee, theta_ee = self.forward_kinematics(a1, a2, a3)
            # Compute errors
            err_x = x_d - x_ee
            err_y = y_d - y_ee
            err_a = theta_d - theta_ee
            # If error lower than tolerance, exit from while loop
            if abs(err_x) < self.tolerance and abs(err_y) < self.tolerance and abs(err_a) < self.tolerance:
                break
            # Compute Jacobian matrix            
            J = np.array([
               [-L1*math.sin(a1) - L2*math.sin(a1 + a2) - L3*math.sin(a1 + a2 + a3),
                   -L2*math.sin(a1 + a2) - L3*math.sin(a1 + a2 + a3),
                   -L3*math.sin(a1 + a2 + a3)],
               [L1*math.cos(a1) + L2*math.cos(a1 + a2) + L3*math.cos(a1 + a2 + a3),
                   L2*math.cos(a1 + a2) + L3*math.cos(a1 + a2 + a3),
                   L3*math.cos(a1 + a2 + a3)],
                [1, 1, 1]
            ])
            # Compute change in joint angles
            d_a = np.matmul(np.linalg.pinv(J), np.array([err_x, err_y, err_a]))
            # Update the joint angles
            a1 += d_a[0]
            a2 += d_a[1]
            a3 += d_a[2]
            # Increase iteration index..
            iteration += 1
            if iteration >= self.max_iterations:
                # No feasible solution found
                return []
            
        # Normalize angles in [-pi, pi]
        a1 = a1 % (2*math.pi)
        if a1 > math.pi:
            a1 -= 2 * math.pi
        elif a1 < -math.pi:
            a1 += 2 * math.pi
        a2 = a2 % (2*math.pi)
        if a2 > math.pi:
            a2 -= 2 * math.pi
        elif a2 < -math.pi:
            a2 += 2 * math.pi
        a3 = a3 % (2*math.pi)
        if a3 > math.pi:
            a3 -= 2 * math.pi
        elif a3 < -math.pi:
            a3 += 2 * math.pi
        
        # Retrieve normalized angles for control purposes
        q1_n = a1
        # Normalize q1+q2
        q2_n = (a1 + a2) % (2*math.pi)
        if q2_n > math.pi:
            q2_n -= 2*math.pi
        elif q2_n < -math.pi:
            q2_n += 2*math.pi
        # Normalize q1+q2+q3
        q3_n = (a1 + a2 + a3) % (2*math.pi)
        if q3_n > math.pi:
            q3_n -= 2*math.pi
        elif q3_n < -math.pi:
            q3_n += 2*math.pi

        return [q1_n, q2_n, q3_n]
        
    # Method to render robot's arm in arcade
    def render(self):
        self.L1.render()
        self.L2.render()
        self.L3.render()
        self.ee.draw()