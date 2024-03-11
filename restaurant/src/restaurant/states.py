import math
from typing import cast, List, Tuple

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from sensor_msgs.msg import PointCloud2
from smach import State, UserData
from trajectory_msgs.msg import JointTrajectoryPoint

from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest, YoloDetection3DResponse


class Position:

    def __init__(self, x: float, y: float, orientation: float = 0):
        self.x: float = x
        self.y: float = y
        self.orientation: float = orientation


dining_area_entrance: Position = Position(1.4, -4.5)

table_positions: List[Position] = [
    Position(-1, -6.8),
    Position(-1, -8.8),
    Position(-1, -10.8),
]

table_front_positions: List[Position] = [
    Position(1, -6.8, math.pi),
    Position(1, -8.8, math.pi),
    Position(1, -10.8, math.pi),
]

move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
head_controller_client = actionlib.SimpleActionClient(
    "/head_controller/follow_joint_trajectory",
    FollowJointTrajectoryAction
)


def move_to(pose: Pose):
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
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result(rospy.Duration(60))


def look_at(joint1: float, joint2: float):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
    point = JointTrajectoryPoint()
    point.positions = [joint1, joint2]
    point.time_from_start = rospy.Duration(1)
    goal.trajectory.points.append(point)
    head_controller_client.send_goal(goal)
    head_controller_client.wait_for_result(rospy.Duration(10))


def get_current_pose() -> Tuple[float, float, Quaternion]:
    msg: PoseWithCovarianceStamped = cast(
        PoseWithCovarianceStamped,
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    )
    x = round(msg.pose.pose.position.x, 2)
    y = round(msg.pose.pose.position.y, 2)
    quat = msg.pose.pose.orientation
    return x, y, quat


def compute_face_quat(x: float, y: float) -> Pose:
    robot_x, robot_y, robot_quat = get_current_pose()
    dist_x = x - robot_x
    dist_y = y - robot_y
    theta_deg: float = np.degrees(math.atan2(dist_y, dist_x))
    try:
        from scipy.spatial.transform import Rotation as R
        (x, y, z, w) = R.from_euler("z", theta_deg, degrees=True).as_quat()
        quaternion = Quaternion(x, y, z, w)
    except ImportError:
        quaternion = robot_quat
    pose = Pose(position=Point(robot_x, robot_y, 0), orientation=quaternion)
    return pose


def look_straight():
    look_at(0, 0)


def turn_to_face(x, y):
    move_to(compute_face_quat(x, y))


class MoveToDiningArea(State):

    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata: UserData) -> str:
        pose = Pose()
        pose.position.x = dining_area_entrance.x
        pose.position.y = dining_area_entrance.y
        pose.orientation.z = 1
        pose.orientation.w = 0
        move_to(pose)
        return "success"


class LocateTable(State):

    def __init__(self):
        super().__init__(
            outcomes=["success", "end"],
            output_keys=["table_x", "table_y", "robot_table_x", "robot_table_y", "table_orientation"],
        )
        self.table_index: int = 0

    def execute(self, userdata: UserData) -> str:
        look_straight()
        if self.table_index >= len(table_positions):
            return "end"
        table_position = table_positions[self.table_index]
        table_front_position = table_front_positions[self.table_index]
        userdata["table_x"] = table_position.x
        userdata["table_y"] = table_position.y
        userdata["robot_table_x"] = table_front_position.x
        userdata["robot_table_y"] = table_front_position.y
        userdata["table_orientation"] = table_front_position.orientation
        self.table_index += 1
        return "success"


class LookAtTable(State):

    def __init__(self):
        super().__init__(
            outcomes=["success"],
            input_keys=["table_x", "table_y", "robot_table_x", "robot_table_y", "table_orientation"],
            output_keys=["robot_table_x", "robot_table_y", "table_orientation"],
        )

    def execute(self, userdata: UserData) -> str:
        table_x: float = userdata["table_x"]
        table_y: float = userdata["table_y"]
        look_straight()
        turn_to_face(table_x, table_y)
        return "success"


class CheckForPerson(State):

    def __init__(self):
        super().__init__(
            outcomes=["person", "no_person"],
            input_keys=["robot_table_x", "robot_table_y", "table_orientation"],
            output_keys=["robot_table_x", "robot_table_y", "table_orientation"],
        )
        self.detect_service = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self.detect_service.wait_for_service()

    def execute(self, userdata: UserData) -> str:
        detections = get_detections()
        for detection in detections:
            if detection.name == "person":
                return "person"
        return "no_person"


class MoveToTable(State):

    def __init__(self):
        super().__init__(
            outcomes=["success"],
            input_keys=["robot_table_x", "robot_table_y", "table_orientation"],
        )

    def execute(self, userdata: UserData) -> str:
        table_x = userdata["robot_table_x"]
        table_y = userdata["robot_table_y"]
        table_orientation = userdata["table_orientation"]
        pose = Pose()
        pose.position.x = table_x
        pose.position.y = table_y
        pose.orientation.z = math.sin(table_orientation / 2)
        pose.orientation.w = math.cos(table_orientation / 2)
        move_to(pose)
        return "success"


class IdentifyItems(State):

    def __init__(self):
        super().__init__(outcomes=["success"])
        self.detect_service = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)

    def execute(self, userdata: UserData) -> str:
        detections = get_detections()
        print("Detections:")
        print(detections)
        print()
        return "success"


class GetDetections3D(State):

    def __init__(self):
        super().__init__(outcomes=["success"])

    def execute(self, userdata: UserData) -> str:
        detections = get_detections()
        print("Detections:")
        print(detections)
        return "success"


class Detection:

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


def get_detections() -> List[Detection]:
    pcl_msg = cast(PointCloud2, rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2))
    detect_service = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
    detect_service.wait_for_service()
    # Create request
    request = YoloDetection3DRequest()
    request.pcl = pcl_msg
    request.dataset = "yolov8n-seg.pt"  # YOLOv8 model, auto-downloads
    request.confidence = 0.5  # Minimum confidence to include in results
    request.nms = 0.3  # Non-maximal suppression
    # Send request
    response: YoloDetection3DResponse = detect_service(request)
    return response.detected_objects
