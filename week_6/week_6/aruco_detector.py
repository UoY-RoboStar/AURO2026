"""Detect ArUco tags in /camera/image_raw, and publish them on /aruco_detections."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class ArucoVisionPublisher(Node):
    """Publish each tag's id, as the class_id, with its position and size in pixels."""

    def __init__(self):
        super().__init__('aruco_vision_publisher')

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.detector_params = cv2.aruco.DetectorParameters()
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publish standard ROS 2 vision detections
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/aruco_detections', 10
        )

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )

        if ids is not None:
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_corners = corners[i][0]

                # Center point in pixels
                cx = float(np.mean(marker_corners[:, 0]))
                cy = float(np.mean(marker_corners[:, 1]))

                # Bounding box width and height
                bbox_w = float(np.max(marker_corners[:, 0]) - np.min(marker_corners[:, 0]))
                bbox_h = float(np.max(marker_corners[:, 1]) - np.min(marker_corners[:, 1]))

                detection = Detection2D()
                detection.header = msg.header

                # Store center offset and pixel dimensions
                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = cy
                detection.bbox.size_x = bbox_w
                detection.bbox.size_y = bbox_h

                # Store the ArUco ID as the class ID
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(marker_id)
                hypothesis.hypothesis.score = 1.0  # Detection confidence

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

            self.pub_detections.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoVisionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
