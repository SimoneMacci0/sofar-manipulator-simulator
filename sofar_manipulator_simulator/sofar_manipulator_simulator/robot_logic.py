import rclpy
from rclpy.node import Node
import math
import time
from enum import Enum
from threading import Thread

from std_msgs.msg import Bool, Int64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from sofar_manipulator_simulator_interface.srv import LocService, IKService

# Enum for pick/place phases
class GripperCmd(Enum):
    GRASP = 1
    RELEASE = -1

class RobotLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")

        # bool variable to keep track of controller's state
        self.controller_idle = True

        # desired joint config publisher 
        self.desired_joint_state_pub = self.create_publisher(JointState, '/controller/desired_joint_state', 10)

        # ack subscriber
        self.create_subscription(Bool, "/controller/ack", self.ack_callback, 10)

        # Define client to retrieve object location
        self.loc_cli = self.create_client(LocService, "/objects/location")
        while not self.loc_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('LocService not available, waiting...') 

        # Define client for computing IK
        self.ik_cli = self.create_client(IKService, "/robot/ik")
        while not self.ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('IKService not available, waiting...')

        # Define publisher for desired joint state
        self.des_joint_state_pub = self.create_publisher(JointState, "/controller/desired_joint_state", 10)

        # Define publisher to control robot's gripper
        self.gripper_cmd_pub = self.create_publisher(Int64, "/robot/gripper_cmd", 10)
        

    # Callback invoked on receiving ack from controller
    def ack_callback(self, msg: Bool):
        self.controller_idle = msg.data

    # Retrieve location of ball/target by color
    def send_object_location_request(self, item: str, color: str):
        req = LocService.Request()
        req.item.data = item
        req.color.data = color
        response = self.loc_cli.call(req)
        return response.location
    
    # Retrieve location of ball/target by color
    def send_ik_request(self, x: float, y: float, theta: float):
        req = IKService.Request()
        req.desired_ee_pose.x = x
        req.desired_ee_pose.y = y
        req.desired_ee_pose.theta = theta
        response = self.ik_cli.call(req)
        if response.feasible:
            return response.desired_angles
        else:
            return []
        

    # Logic node routine
    def routine(self):
        for color in ["RED", "BLUE"]:
            for i in range(2):
                loc = None
                # forward trip, move to ball and grasp it...
                if i == 0:
                    # Get location of ball
                    loc = self.send_object_location_request("BALL", color)
                # Round trip, move to target...
                else:
                    # Get location of target
                    loc = self.send_object_location_request("TARGET", color)
                # Regardless of destination, compute IK to reach ball/target
                if loc.x > 0:
                    a = 0.0
                else:
                    a = math.pi
                q_des = self.send_ik_request(loc.x, loc.y, a)
                k = 0
                while len(q_des) < 3:
                    a = k * math.pi / 10
                    k += 1
                    q_des = self.send_ik_request(loc.x, loc.y, a)
                # Send desired joint configuration to controller...
                msg = JointState()
                msg.name = ["q1", "q2", "q3"]
                msg.position = [q_des[0].data, q_des[1].data, q_des[2].data]
                self.des_joint_state_pub.publish(msg)
                self.controller_idle = False
                # Wait for controller ack
                while not self.controller_idle:
                    time.sleep(1)
                # If forward trip, grasp ball...
                grasp_msg = Int64()
                if i == 0:
                    grasp_msg.data = GripperCmd.GRASP.value
                # Else, release ball...
                else:
                    grasp_msg.data = GripperCmd.RELEASE.value
                self.gripper_cmd_pub.publish(grasp_msg)
        msg = JointState()
        msg.name = ["q1", "q2", "q3"]
        msg.position = [0.0, 0.0, 0.0]
        self.des_joint_state_pub.publish(msg)
        self.controller_idle = False
                

def main(args=None):
    rclpy.init(args=args)
    logic = RobotLogic()
    
    # Spinning thread to make sure callbacks are executed
    spin_thread = Thread(target=rclpy.spin, args=(logic,))
    spin_thread.start()

    # Start node routine
    time.sleep(20)
    logic.routine()

    # On shutdown..
    logic.get_logger().info("Shutdown logic node...")
    logic.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

