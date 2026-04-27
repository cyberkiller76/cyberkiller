import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, roscpp_initialize
from geometry_msgs.msg import Pose
import time
import sys
from tf_transformations import quaternion_from_euler

class RoboticArmAutomation(Node):
    def __init__(self):
        super().__init__('robotic_arm_automation')
        
        roscpp_initialize(sys.argv)
        
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")
        
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(5)
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.01)

    def move_to_joint_positions(self, joint_positions):
        self.arm.set_joint_value_target(joint_positions)
        success = self.arm.go(wait=True)
        return success

    def move_to_cartesian_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        
        self.arm.set_pose_target(pose_goal)
        success = self.arm.go(wait=True)
        return success

    def home_position(self):
        home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        return self.move_to_joint_positions(home_joints)

    def automation_sequence(self):
        self.home_position()
        time.sleep(1)
        
        self.move_to_cartesian_pose(x=0.4, y=0.0, z=0.3, pitch=3.14)
        time.sleep(1)
        
        self.move_to_cartesian_pose(x=0.4, y=0.0, z=0.15, pitch=3.14)
        time.sleep(1)
        
        self.close_gripper()
        
        self.move_to_cartesian_pose(x=0.4, y=0.0, z=0.35, pitch=3.14)
        time.sleep(1)
        
        self.move_to_cartesian_pose(x=0.0, y=0.4, z=0.25, pitch=3.14)
        time.sleep(1)
        
        self.open_gripper()
        
        self.home_position()

    def close_gripper(self):
        try:
            self.gripper.set_named_target("close")
            self.gripper.go(wait=True)
        except:
            pass

    def open_gripper(self):
        try:
            self.gripper.set_named_target("open")
            self.gripper.go(wait=True)
        except:
            pass


def main():
    rclpy.init()
    node = RoboticArmAutomation()
    
    try:
        node.automation_sequence()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
