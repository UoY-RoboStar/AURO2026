"""
Detect barrels by colour in /camera/image_raw, and publish them on /barrel_detections.

The same message as aruco_detector, with the colour as the class_id, and published for every frame
so that an empty list means no barrel is in view. Annotated images are on /camera/image_barrels,
made only while something subscribes to them.
"""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

MIN_SATURATION = 100
MIN_VALUE = 20
HUES = {'red': ((0, 10), (170, 180)), 'blue': ((110, 130),)}  # OpenCV's hue goes up to 180
MIN_AREA = 150  # pixels
MAX_ASPECT_RATIO = 5


def colour_mask(hsv, hues):
    """Return the mask of the pixels in any of the ranges of hues."""
    masks = [cv2.inRange(hsv, (low, MIN_SATURATION, MIN_VALUE), (high, 255, 255))
             for low, high in hues]
    return np.bitwise_or.reduce(masks)


def find_barrels(image):
    """Return each barrel's colour, centroid (x, y) and bounding box (width, height), in pixels."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    barrels = []
    for colour, hues in HUES.items():
        count, _, stats, centroids = cv2.connectedComponentsWithStats(colour_mask(hsv, hues))
        for label in range(1, count):  # 0 is the background
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if (stats[label, cv2.CC_STAT_AREA] >= MIN_AREA
                    and max(width, height) <= MAX_ASPECT_RATIO * min(width, height)):
                barrels.append((colour, *map(float, centroids[label]), width, height))
    return barrels


def annotate(image, barrels):
    """Draw each barrel's bounding box and colour on image."""
    for colour, x, y, width, height in barrels:
        corner = (int(x - width / 2), int(y - height / 2))
        cv2.rectangle(image, corner, (corner[0] + width, corner[1] + height), (255, 255, 255), 2)
        cv2.putText(image, colour, (corner[0], corner[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA)


class VisualSensor(Node):
    """Publish the barrels in each camera image."""

    def __init__(self):
        super().__init__('visual_sensor')
        self.bridge = CvBridge()
        self.pub_detections = self.create_publisher(Detection2DArray, '/barrel_detections', 10)
        self.pub_image = self.create_publisher(Image, '/camera/image_barrels', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        barrels = find_barrels(image)

        detections = Detection2DArray(header=msg.header)
        for colour, x, y, width, height in barrels:
            detection = Detection2D(header=msg.header)
            detection.bbox.center.position.x = x
            detection.bbox.center.position.y = y
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = colour
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)
            detections.detections.append(detection)
        self.pub_detections.publish(detections)

        if self.pub_image.get_subscription_count() > 0:
            annotate(image, barrels)
            annotated = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            annotated.header = msg.header
            self.pub_image.publish(annotated)


def main(args=None):
    rclpy.init(args=args)
    node = VisualSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
