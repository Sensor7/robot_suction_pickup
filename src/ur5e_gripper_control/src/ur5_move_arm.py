#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import cv_bridge
import pymoveit2
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import Image
from ur5e_vision_msgs.msg import Tracker
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy
import numpy as np
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur as robot
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from geometry_msgs.msg import TransformStamped
import array
from threading import Thread
import time

# reference_frame
REFERENCE_FRAME = "left_base_link"
tracker = Tracker()
class UR5Move(Node):
    def __init__(self):
        super().__init__('ur5_move')
        # ROS2 components
        self.objectTracker_sub = self.create_subscription(Tracker, '/objects', self.tracking_callback, 10)
        self.objectTracker_pub = self.create_publisher(Tracker, '/objects_to_box', 10)
        self.gripper_status_sub = self.create_subscription(Bool, '/epick/grasping', self.gripper_callback, 10)
        # wait for the service to be available
        self.gripper_client = self.create_client(SetBool, '/epick/switch')
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.update_current_pose)

        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points = []
        self.current_pose = None
        self.state_change_time = self.get_clock().now()
        self.grasp = False

        self.get_logger().info("Starting node ur5_move")

        # Initialize MoveItPy to control the UR5e
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
        node=self,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.synchronous = True

        # Default joint positions
        self.default_joint_states = array.array('d', [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0])
        self.target_joint_states = array.array('d', [0.8199, -1.4479, 1.6921, -1.8665, -1.5002, 0.0872])
        self.drop_joint_states = array.array('d', [0.9245, -0.9245, 2.006, -2.6515, -1.5002, 0.0872])

        # Add scene objects for collision avoidance
        object_id = "table"
        self.moveit2.add_collision_box(id = object_id, position = [0.5, -0.6, -0.53], quat_xyzw = [0.0, 0.0, 0.0, 1.0], size = [1.0, 1.5, 1.0])

        self.return_to_default_pose()

    
    def tracking_callback(self, msg):
        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y

        if self.phase == 1:
            self.points.append(msg.x)

        if len(self.points) > 50:
            self.track_flag = True
            self.phase = 2

        if self.phase == 2:
            self.execute_pick_and_place()
    
    def gripper_callback(self, msg):
        pass
    
    def request_gripper_service(self):
        request = SetBool.Request()
        request.data = not self.grasp
        self.gripper_client.call_async(request).add_done_callback(self.gripper_service_callback)
        self.get_logger().info(f"Sution: {request.data}")

    def suction_on(self):
        self.grasp = False
        if not self.grasp:
            self.request_gripper_service()
    
    def suction_off(self):
        self.grasp = True
        if self.grasp:
            self.request_gripper_service()

    def gripper_service_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
    
    def execute_pick_and_place(self):
        if self.track_flag:
            if self.get_current_pose() is None:
                return
            else:
                start_pose = Pose()
                start_pose.position.x = self.get_current_pose().translation.x
                start_pose.position.y = self.get_current_pose().translation.y
                start_pose.position.z = self.get_current_pose().translation.z
                start_pose.orientation.x = self.get_current_pose().rotation.x
                start_pose.orientation.y = self.get_current_pose().rotation.y
                start_pose.orientation.z = self.get_current_pose().rotation.z
                start_pose.orientation.w = self.get_current_pose().rotation.w
                self.waypoints = []
                wpose = deepcopy(start_pose)
                
                # Move to pre-grasp pose *(above the object)*
                self.get_logger().info("Moving to pre-grasp pose")
                wpose.position.x += self.error_x * 1.55 / 105
                wpose.position.y += self.error_y * 0.032 / 105
                wpose.position.z = 0.35
                self.cartesian_movement(
                    position=array.array('d', [wpose.position.x, wpose.position.y, wpose.position.z]),
                    quat_xyzw=array.array('d', [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]),
                )

                # Move to grasp pose *(on top of the object)*
                self.get_logger().info("Moving to grasp pose")
                wpose.position.z = 0.16
                self.cartesian_movement(
                    position=array.array('d', [wpose.position.x, wpose.position.y, wpose.position.z]),
                    quat_xyzw=array.array('d', [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]),
                )
                time.sleep(2)

                # Turn on suction
                self.get_logger().info("Turning on suction")
                self.suction_on()
                time.sleep(2)

                # Move to pre-grasp pose *(above the object)*
                self.get_logger().info("Moving to pre-grasp pose")
                wpose.position.z = 0.35
                self.cartesian_movement(
                    position=array.array('d', [wpose.position.x, wpose.position.y, wpose.position.z]),
                    quat_xyzw=array.array('d', [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w]),
                )

                # Move to pre-place pose *(above the target location)*
                self.get_logger().info("Moving to pre-place pose")
                self.joint_states_movement(self.target_joint_states)
                time.sleep(2)

                # Move to place pose *(on top of the target location)*
                self.get_logger().info("Moving to place pose")
                self.joint_states_movement(self.drop_joint_states)
                time.sleep(2)

                # Turn off suction
                self.get_logger().info("Turning off suction")
                self.suction_off()
                time.sleep(2)

                # Move to default pose
                self.get_logger().info("Moving to default pose")
                self.return_to_default_pose()

                self.phase = 3
                self.points.clear()
                self.track_flag = False
                self.get_logger().info("demo finished")


    def get_current_pose(self):
        """ Returns the last updated TF transform as the current pose. """
        if self.current_pose is None:
            self.get_logger().warn("TF pose not available yet.")
        return self.current_pose

    def update_current_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'left_base_link', 'left_tool0', rclpy.time.Time())
            self.current_pose = transform.transform
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF Lookup failed: {str(e)}")
            return None
        
    def return_to_default_pose(self):
        self.joint_states_movement(self.default_joint_states)

    def joint_states_movement(self, joint_states):
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.moveit2.move_to_configuration(joint_states)
        if self.synchronous:
            self.moveit2.wait_until_executed()
    
    def cartesian_movement(self, position, quat_xyzw):
        self.get_logger().info("move_to_pose: " + str(position) + ", " + str(quat_xyzw))
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.get_logger().info("Moving to pose")
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=True,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )
        if self.synchronous:
            self.moveit2.wait_until_executed()

def main(args=None):
    rclpy.init(args=args)
    ur5_move = UR5Move()
    rclpy.spin(ur5_move)

    # Cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()