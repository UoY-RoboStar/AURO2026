"""
Reactor monitor for the assessment world.

Publishes, both as a percentage of maximum_power_level
  current_power_level (std_msgs/Float64, 1 Hz): sum of the power levels of the fuel rods inserted
      in the reactor, times (1 + control_rod_coefficient x number of control rods not in the
      reactor). A rod is inserted if it stands in one of the reactor's cells; rods in the storage
      pool or hanging from the winch do not count. Can exceed 100 with control rods removed.
  target_power_level (std_msgs/Float64, 5 Hz): the target_power_level parameter.

maximum_power_level is in the units of that sum (each rod contributes its level, 0-100). If it is
not set (0), it is the power of a full reactor with all control rods in: one rod at 100 in every
reactor fuel cell.
Serves
  /crane/winch/rod_power_level (assessment_interfaces/srv/RodPowerLevel): power level of the fuel
      rod directly below the winch, if the winch is close enough above it.
  cell_pose (assessment_interfaces/srv/CellPose): centre of a cell, by index (column letter west to
      east, A-M reactor and N-Z storage; row number north to south), e.g. "C4".

The power_display node shows both levels on the north wall of the world (see power_display.py).
This node makes no gz-transport requests: in Python, they do not get their replies while the
process has a gz-transport subscription, like the one this node follows the rods with.

Rod positions come straight from Gazebo (gz-transport, the world's dynamic_pose/info topic), so
they are not published on ROS. Power levels come from the manifest that the launch file hands
over, once, through the named pipe at rod_manifest (see fuel_layout.serve_manifest()): they are
not in any file or parameter that other nodes could read.
"""

import math
import signal
import threading

from assessment.fuel_layout import read_manifest
from assessment_interfaces.srv import CellPose, RodPowerLevel
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode, SubscribeOptions
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

WINCH_HALF_HEIGHT = 0.125  # winch collision box is 0.25 m tall, centred on its link origin
# Pose messages taken from Gazebo per second (of wall time). Gazebo sends one per physics step,
# each with every moving object's pose (some 270), which would cost a large share of a core to go
# through in Python; the power level is published once a second, and the winch has to be resting on
# a rod for rod_power_level, so 10 a second is plenty. Gazebo's transport drops the rest before
# they are decoded.
POSE_RATE = 10


class ReactorMonitor(Node):

    def __init__(self):
        super().__init__('reactor_monitor')
        p = self.declare_parameter
        manifest_path = p('rod_manifest', '').value
        self.target = float(p('target_power_level', 50.0).value)  # percent of maximum
        self.maximum = float(p('maximum_power_level', 0.0).value)
        self.coefficient = float(p('control_rod_coefficient', 0.05).value)
        self.max_offset = float(p('sense_max_horizontal_offset', 0.15).value)
        self.max_gap = float(p('sense_max_vertical_gap', 0.5).value)
        self.seat_xy = float(p('seated_xy_tolerance', 0.15).value)
        self.seat_z = float(p('seated_z_tolerance', 0.1).value)
        world = p('world_name', 'world_demo').value

        manifest = read_manifest(manifest_path)
        self.levels = manifest['fuel_rods']
        self.control_rods = set(manifest['control_rods'])
        self.reactor_cells = [tuple(c) for c in manifest['reactor_cells']]
        self.seated_z = manifest['rod_seated_z']
        self.rod_half_height = manifest['rod_half_height']
        self.cells = manifest['cells']
        self.cell_top_z = manifest['cell_top_z']
        if self.maximum <= 0.0:
            self.maximum = 100.0 * manifest['reactor_fuel_cell_count']
        self.get_logger().info(
            f'{len(self.levels)} fuel rods, {len(self.control_rods)} control rods '
            f'(seed {manifest["seed"]}); maximum power level {self.maximum:g}, '
            f'target {self.target:g}%')

        self.lock = threading.Lock()
        self.rods = {}      # rod model name -> (x, y, z)
        self.winch = None   # (x, y, z) of the winch link
        self.gz = GzNode()
        self.pose_topic = f'/world/{world}/dynamic_pose/info'
        self.tracked = set(self.levels) | self.control_rods | {'crane', 'winch'}
        options = SubscribeOptions()
        options.msgs_per_sec = POSE_RATE
        if not self.gz.subscribe(Pose_V, self.pose_topic, self.on_poses, options):
            raise RuntimeError(f'cannot subscribe to {self.pose_topic}')

        self.current_pub = self.create_publisher(Float64, 'current_power_level', 10)
        self.target_pub = self.create_publisher(Float64, 'target_power_level', 10)
        self.create_timer(1.0, self.publish_current)
        self.create_timer(0.2, self.publish_target)
        self.create_service(RodPowerLevel, '/crane/winch/rod_power_level', self.on_rod_power_level)
        self.create_service(CellPose, 'cell_pose', self.on_cell_pose)

    def on_poses(self, msg):
        """gz-transport thread: keep the latest position of every rod and of the winch."""
        rods, winch = {}, None
        crane = (0.0, 0.0, 0.0)
        tracked = self.tracked
        for pose in msg.pose:
            if pose.name not in tracked:
                continue
            pos = (pose.position.x, pose.position.y, pose.position.z)
            if pose.name in self.levels or pose.name in self.control_rods:
                rods[pose.name] = pos
            elif pose.name == 'crane':
                crane = pos
            elif pose.name == 'winch':
                winch = pos  # relative to the crane model
        if winch is not None:
            winch = tuple(w + c for w, c in zip(winch, crane))
        with self.lock:
            self.rods.update(rods)
            if winch is not None:
                self.winch = winch

    def inserted(self, pos):
        x, y, z = pos
        return abs(z - self.seated_z) <= self.seat_z and any(
            math.hypot(x - cx, y - cy) <= self.seat_xy for cx, cy in self.reactor_cells)

    def power_level(self):
        with self.lock:
            rods = dict(self.rods)
        if not rods:
            return None
        fuel = sum(level for name, level in self.levels.items()
                   if name in rods and self.inserted(rods[name]))
        removed = sum(1 for name in self.control_rods
                      if name in rods and not self.inserted(rods[name]))
        return fuel * (1.0 + self.coefficient * removed)

    def publish_current(self):
        power = self.power_level()
        if power is None:
            self.get_logger().info('waiting for rod poses from Gazebo', throttle_duration_sec=5.0)
            return
        percent = 100.0 * power / self.maximum
        self.current_pub.publish(Float64(data=percent))

    def publish_target(self):
        self.target_pub.publish(Float64(data=self.target))

    def on_rod_power_level(self, request, response):
        with self.lock:
            winch, rods = self.winch, dict(self.rods)
        response.success = False
        if winch is None:
            return response
        winch_bottom = winch[2] - WINCH_HALF_HEIGHT
        best = None
        for name, (x, y, z) in rods.items():
            offset = math.hypot(x - winch[0], y - winch[1])
            gap = winch_bottom - (z + self.rod_half_height)
            if offset <= self.max_offset and -0.05 <= gap <= self.max_gap:
                if best is None or offset < best[0]:
                    best = (offset, name)
        # The nearest rod in range decides; a control rod there has no power level to report.
        if best is not None and best[1] in self.levels:
            response.success = True
            response.power_level = float(self.levels[best[1]])
        return response

    def on_cell_pose(self, request, response):
        cell = self.cells.get(request.cell.strip().upper())
        response.success = cell is not None
        if cell is not None:
            x, y, response.area, _ = cell  # whether it is a control cell is not revealed
            response.centre.x, response.centre.y, response.centre.z = x, y, self.cell_top_z
        return response

    def destroy_node(self):
        # Unsubscribe before the gz-transport node is destroyed, or Python can crash on exit.
        self.gz.unsubscribe(self.pose_topic)
        super().destroy_node()


def main():
    rclpy.init()
    node = ReactorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches the node twice (from the terminal and forwarded by ros2 launch); don't let
        # the second one interrupt the cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
