from .lib.manipulator_sim import ManipulatorSim
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64, Int64
from sensor_msgs.msg import JointState

from sofar_manipulator_simulator_interface.srv import IKService, LocService

from ament_index_python.packages import get_package_share_directory


class ManipulatorSimNode(Node):

    def __init__(self):
        super().__init__("manipulator_sim_node")

        # Manipulator simulator
        self.sim = ManipulatorSim(
            get_package_share_directory("sofar_manipulator_simulator")
        )
        self.sim.setup()
        self.sim.thread.start()

        # Subscriber for setting joint state
        self.create_subscription(JointState, "/robot/joint_cmd", self.on_joint_cmd, 10)

        # Subscriber for controlling robot's gripper
        self.create_subscription(Int64, "/robot/gripper_cmd", self.on_gripper_cmd, 10)

        # Service for computing inverse kinematics
        self.create_service(IKService, "/robot/ik", self.compute_ik)

        # Service for retrieving coordinates of ball/target
        self.create_service(LocService, "/objects/location", self.get_object_location)


    # Callback invoked whenever desired joint configuration is received
    def on_joint_cmd(self, msg: JointState):
        self.sim.robot.set_joint_angles(
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        )

    # Callback for controlling robot's gripper
    def on_gripper_cmd(self, msg: Int64):
        if msg.data == 1:
            self.sim.grasp()
        elif msg.data == -1:
            self.sim.drop()

    # Callback to compute ik 
    def compute_ik(self, request: IKService.Request, response: IKService.Response):
        # Get desired end-effector pose from request
        x_d = request.desired_ee_pose.x
        y_d = request.desired_ee_pose.y
        theta_d = request.desired_ee_pose.theta
        # Compute IK and retrive desired joint angles (if any)
        q_des = self.sim.robot.inverse_kinematics(x_d, y_d, theta_d)
        # If IK computation was successful...
        if len(q_des) > 0:
            response.feasible.data = True
            for q in q_des:
                q_float = Float64()
                q_float.data = q
                response.desired_angles.append(q_float)
        # Else, if pose was not reachable...
        else:
            response.feasible.data = False
        return response
    
    
    # Callback for retrieving ball/target locations for computing IK
    def get_object_location(self, request: LocService.Request, response: LocService.Response):
        # Get request parameters
        item = request.item.data
        color = request.color.data
        # convert desired color to integer
        color_idx = 0 if color == "RED" else 1
        # If user requested location of a ball, fill data...
        if item == "BALL":
            response.location.x = self.sim.balls[color_idx].center_x - self.sim.origin[0]
            response.location.y = self.sim.balls[color_idx].center_y - self.sim.origin[1]
            response.location.z = 0.0
        # Else, if user requested the coordinates of a target location...
        elif item == "TARGET":
            response.location.x = self.sim.targets[color_idx].center_x - self.sim.origin[0]
            response.location.y = self.sim.targets[color_idx].center_y - self.sim.origin[1]
            response.location.z = 0.0
        return response

  
def main(args=None):
    
    rclpy.init(args=args)
    sim_node = ManipulatorSimNode()

    print("Press Ctrl+C to exit...")
    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()
