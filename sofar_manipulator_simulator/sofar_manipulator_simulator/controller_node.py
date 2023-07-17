import rclpy
from rclpy.node import Node
from simple_pid import PID

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# Base gain values
KP = 0.75
KI = 0.001
KD = 0.05

# Internal joint controller class
class JointController:

    def __init__(self, id):
        self.id = id
        # Variable for current joint pos (always starts from 0)
        self.joint_pos = 0
        # internal PID controller
        self.pid = PID(KP, KI, KD)
        # Set PID's update rate
        self.pid.sample_time = 0.0333
        # Control clip value
        self.clip_val = 1.0

    def set_new_goal(self, target_position: float):
        # Reset internal pid variables and set new goal position, starting from current one
        self.pid.reset()
        self.pid.setpoint = target_position

    def get_joint_error(self):
        return abs(self.joint_pos - self.pid.setpoint)

    def update_joint_position(self):
        # Compute velocity control based on current joint position
        joint_vel = self.pid(self.joint_pos)
        # Saturate control value
        if joint_vel > self.clip_val:
            joint_vel = self.clip_val
        elif joint_vel < -self.clip_val:
            joint_vel = -self.clip_val
        # Update current position
        self.joint_pos += joint_vel * self.pid.sample_time

    def log(self):
        return "Joint {0} - current_position: {1}".format(self.id, self.joint_pos)

    
# Main robot controller node class
class ControllerNode(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        # Internal controllers for the three joints
        self.joint1_controller = JointController(1)
        self.joint2_controller = JointController(2)
        self.joint3_controller = JointController(3)

        # Subscriber for desired joint configuration
        self.create_subscription(JointState, "/controller/desired_joint_state", self.set_desired_joint_state, 10)
        
        # Publisher for joint velocity command
        self.joint_cmd_pub = self.create_publisher(JointState, "/robot/joint_cmd", 10)

        # Publisher for controller's ack
        self.ack_pub = self.create_publisher(Bool, "/controller/ack", 10)

        # Variales for control loop
        self.joints_controller_timer = None
        self.tolerance = 0.01

        self.get_logger().info("Robot controller module up and running!")


    # Callback for setting up new control loop on receiving new target config
    def set_desired_joint_state(self, msg: JointState):
        self.get_logger().info(
            "Received desired joint state:  \
                [q1: {}, q2: {}, q3: {}]".format(msg.position[0], msg.position[1], msg.position[2]))
        # Set current and target positions for each joint controller
        self.joint1_controller.set_new_goal(msg.position[0])
        self.joint2_controller.set_new_goal(msg.position[1])
        self.joint3_controller.set_new_goal(msg.position[2])
        # Start timer for control cycle
        self.joints_controller_timer = self.create_timer(self.joint1_controller.pid.sample_time, self.control_loop)

    # Inner control loop method
    def control_loop(self):

        # Stop timer if joint position errors fall below threshold
        err1 = self.joint1_controller.get_joint_error()
        err2 = self.joint2_controller.get_joint_error()
        err3 = self.joint3_controller.get_joint_error()
        if abs(err1) < self.tolerance and abs(err2) < self.tolerance and abs(err3) < self.tolerance:
            self.joints_controller_timer.cancel()
            # Publish ack to ensure robot's action manager moves on
            ack = Bool()
            ack.data = True
            self.ack_pub.publish(ack)
            
        # Update joint position according to control law
        self.joint1_controller.update_joint_position()
        #self.get_logger().info(self.joint1_controller.log())
        self.joint2_controller.update_joint_position()
        self.joint3_controller.update_joint_position()
        # Build and publish updated joint cmd message
        joint_cmd_msg = JointState()
        joint_cmd_msg.name = ["q1", "q2", "q3"]
        joint_cmd_msg.position = [
            self.joint1_controller.joint_pos,
            self.joint2_controller.joint_pos,
            self.joint3_controller.joint_pos,
        ]
        self.joint_cmd_pub.publish(joint_cmd_msg)
        


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    # Spin indefinitely
    rclpy.spin(controller_node)
    # On shutdown...
    controller_node.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == "__main__":
    main()
