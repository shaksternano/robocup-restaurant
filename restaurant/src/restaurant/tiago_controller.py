import math
from typing import cast, List, Tuple

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest, YoloDetection3DResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectoryPoint


class YoloDetection:

    def __init__(
        self,
        name: str,
        confidence: float,
        xywh: List[int],
        target_frame: str,
        xyseg: List[int],
        point: Point,
    ):
        self.name: str = name
        self.confidence: float = confidence
        self.xywh: List[int] = xywh
        self.target_frame: str = target_frame
        self.xyseg: List[int] = xyseg
        self.point: Point = point


class TiagoController:

    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.head_controller_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self.yolo_service = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self.yolo_service.wait_for_service()

    def move_to(self, pose: Pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        rospy.loginfo(
            "base is going to (%.2f, %.2f, %.2f) pose",
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result(rospy.Duration(60))

    def look_at(self, joint1: float, joint2: float):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.head_controller_client.send_goal(goal)
        self.head_controller_client.wait_for_result(rospy.Duration(10))

    def look_straight(self):
        self.look_at(0, 0)

    def turn_to_face(self, x: float, y: float):
        self.move_to(compute_face_quaternion(x, y))

    def get_detections(self) -> List[YoloDetection]:
        point_cloud = cast(PointCloud2, rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2))
        request = YoloDetection3DRequest()
        request.pcl = point_cloud
        request.dataset = "yolov8n-seg.pt"
        request.confidence = 0.5
        request.nms = 0.3
        response: YoloDetection3DResponse = self.yolo_service(request)
        return response.detected_objects


def get_current_pose() -> Tuple[float, float, Quaternion]:
    msg: PoseWithCovarianceStamped = cast(
        PoseWithCovarianceStamped,
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    )
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    quaternion = msg.pose.pose.orientation
    return x, y, quaternion


def compute_face_quaternion(x: float, y: float) -> Pose:
    robot_x, robot_y, robot_quaternion = get_current_pose()
    dist_x = x - robot_x
    dist_y = y - robot_y
    theta_deg: float = np.degrees(math.atan2(dist_y, dist_x))
    x1, y1, z, w = Rotation.from_euler("z", theta_deg, degrees=True).as_quat()
    quaternion = Quaternion(x1, y1, z, w)
    pose = Pose(position=Point(robot_x, robot_y, 0), orientation=quaternion)
    return pose
