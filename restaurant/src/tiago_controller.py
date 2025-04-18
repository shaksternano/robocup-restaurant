import json
import math
from typing import cast, List, Tuple, Dict, Any

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point, Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectoryPoint

from lasr_rasa.srv import Rasa, RasaRequest
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest, YoloDetection3DResponse


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

    def __init__(self, await_ready: bool = False):
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.head_controller_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.vel_publisher = rospy.Publisher("mobile_base_controller/cmd_vel", Twist, queue_size=1)
        self.yolo_service = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self.rasa_service = rospy.ServiceProxy("/lasr_rasa/parse", Rasa)
        self.moving: bool = False

        if await_ready:
            rospy.loginfo("Waiting for action servers and services to start")
            self.move_base_client.wait_for_server()
            self.head_controller_client.wait_for_server()
            self.yolo_service.wait_for_service()
            self.rasa_service.wait_for_service()
            rospy.loginfo("Action servers and services are ready")

    @staticmethod
    def get_current_pose() -> Tuple[float, float, Quaternion]:
        msg: PoseWithCovarianceStamped = cast(
            PoseWithCovarianceStamped,
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        )
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        quaternion = msg.pose.pose.orientation
        return x, y, quaternion

    def change_pose(self, pose: Pose, prevent_crashes: bool = False) -> int:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        rospy.loginfo(
            "Base is going to (%.2f, %.2f, %.2f) pose",
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        self.moving = True
        self.move_base_client.wait_for_server()
        self.move_base_client.send_goal(goal, done_cb=lambda state, result: self.finish_moving())
        if prevent_crashes:
            while self.moving:
                current_x, current_y, _ = self.get_current_pose()
                distance_from_goal = math.sqrt(
                    (pose.position.x - current_x) ** 2
                    + (pose.position.y - current_y) ** 2
                )
                if distance_from_goal < 3:
                    detections = self.get_detections()
                    for detection in detections:
                        detection_x = detection.point.x
                        detection_y = detection.point.y
                        distance = math.sqrt((detection_x - current_x) ** 2 + (detection_y - current_y) ** 2)
                        if distance < 2:
                            rospy.loginfo("Obstacle detected at (%.2f, %.2f)", detection_x, detection_y)
                            self.move_base_client.cancel_goal()
                            return GoalStatus.PREEMPTED
                rospy.sleep(0.1)
        else:
            self.move_base_client.wait_for_result(rospy.Duration(60))
        return self.move_base_client.get_state()

    def finish_moving(self):
        self.moving = False

    def look_at(self, joint1: float, joint2: float):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.head_controller_client.wait_for_server()
        self.head_controller_client.send_goal(goal)
        self.head_controller_client.wait_for_result(rospy.Duration(10))

    def look_straight(self):
        self.look_at(0, 0)

    def compute_face_quaternion(self, x: float, y: float) -> Pose:
        robot_x, robot_y, robot_quaternion = self.get_current_pose()
        dist_x = x - robot_x
        dist_y = y - robot_y
        theta_radians = math.atan2(dist_y, dist_x)
        x1, y1, z, w = Rotation.from_euler("z", theta_radians).as_quat()
        quaternion = Quaternion(x1, y1, z, w)
        pose = Pose(position=Point(robot_x, robot_y, 0), orientation=quaternion)
        return pose

    def turn_to_face(self, x: float, y: float):
        self.change_pose(self.compute_face_quaternion(x, y))

    def rotate(self, radians: float):
        vel_msg = Twist()
        angular_velocity = 0.8
        clockwise = radians < 0
        if clockwise:
            vel_msg.angular.z = -abs(angular_velocity)
        else:
            vel_msg.angular.z = abs(angular_velocity)

        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        while current_angle < radians:
            self.vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_velocity * (t1 - t0)
        rotation_seconds = abs(radians) / angular_velocity
        rospy.sleep(rotation_seconds)

    def get_detections(self) -> List[YoloDetection]:
        point_cloud = cast(PointCloud2, rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2))
        request = YoloDetection3DRequest()
        request.pcl = point_cloud
        request.dataset = "yolov8s-seg.pt"
        request.confidence = 0.5
        request.nms = 0.3
        self.yolo_service.wait_for_service()
        response: YoloDetection3DResponse = self.yolo_service(request)
        return response.detected_objects

    def locate_person(self) -> Point:
        turn_times = 8
        rotation_angle = 2 * math.pi / turn_times
        current_x, current_y, _ = self.get_current_pose()
        for i in range(turn_times):
            detections = self.get_detections()
            person_detections = [detection for detection in detections if "person" in detection.name]
            closest_person = min(
                person_detections,
                key=lambda person: (person.point.x - current_x) ** 2 + (person.point.y - current_y) ** 2,
                default=None,
            )
            if closest_person is not None:
                return closest_person.point
            self.rotate(rotation_angle)
        return Point()

    def get_order_item(self, order_sentence: str) -> str:
        request = RasaRequest(order_sentence)
        self.rasa_service.wait_for_service()
        response = self.rasa_service(request)
        json_string: str = response.json_response
        # noinspection PyBroadException
        try:
            json_object: Dict[str, Any] = json.loads(json_string)
            entities: Dict[str, List[Dict[str, Any]]] = json_object["entities"]
            entity = next(iter(entities.values()))[0]
            return entity["value"]
        except Exception:
            pass
        return ""
