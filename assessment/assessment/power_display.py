"""
Power display on a wall of the assessment world, drawn with Gazebo GUI markers.

Shows the current power level (seven-segment digits, coloured by how close it is to the target),
the target (white digits) and a bar gauge from 0 to 120% with ticks at the target and at 100%.
Gazebo's ogre2 renderer cannot draw text markers, so the digits are built from box markers; unlit
segments are drawn dim, like a real display.

Markers belong to the Gazebo GUI (its MarkerManager plugin, service /marker_array): they do not
affect physics and are not seen by camera sensors. The whole display is re-sent once a second, so
it reappears within a second if the GUI is restarted.

The power_display node draws the levels that reactor_monitor publishes on current_power_level and
target_power_level. It is a node of its own because in Python, a gz-transport request does not
get its reply while the same process has a gz-transport subscription, and holds every other
thread of the process while it waits: drawing from reactor_monitor, which follows the rods with
such a subscription, stalled the whole node for seconds at a time.
"""

import signal
import threading

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.marker_pb2 import Marker
from gz.msgs.marker_v_pb2 import Marker_V
from gz.transport import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

NAMESPACE = 'power_display'
GAUGE_MAX = 120.0  # percent at the right-hand end of the bar

# Seven-segment digits: segment name -> (x, z) centre in digit units, horizontal?
SEGMENTS = {
    'a': ((0.0, 0.5), True), 'b': ((0.5, 0.25), False), 'c': ((0.5, -0.25), False),
    'd': ((0.0, -0.5), True), 'e': ((-0.5, -0.25), False), 'f': ((-0.5, 0.25), False),
    'g': ((0.0, 0.0), True),
}
DIGITS = {
    '0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd', '4': 'fgbc', '5': 'afgcd',
    '6': 'afgedc', '7': 'abc', '8': 'abcdefg', '9': 'abcdfg', ' ': '',
}

BLACK = (0.03, 0.03, 0.03)
DIM = (0.16, 0.16, 0.16)
WHITE = (0.95, 0.95, 0.95)
GREY = (0.55, 0.55, 0.55)
GREEN = (0.1, 0.9, 0.2)
AMBER = (1.0, 0.65, 0.0)
RED = (1.0, 0.12, 0.08)


class PowerDisplay:
    """
    Wall display of current and target power level.

    The display faces -y, on a wall whose surface is at y = wall_y; (x, z) is its centre.
    """

    DIGIT_H = 0.9    # digit height (m)
    DIGIT_W = 0.5    # digit width (m)
    SEG_T = 0.1      # segment thickness (m)
    PITCH = 0.75     # distance between digit centres (m)
    BAR_LEN = 8.0    # bar gauge length (m)

    def __init__(self, gz_node, logger, x=0.0, z=3.6, wall_y=12.5, tolerance=5.0):
        self.gz = gz_node
        self.logger = logger
        self.x, self.z = x, z
        self.y_back = wall_y - 0.02   # background panel, just in front of the wall
        self.y_front = wall_y - 0.05  # everything drawn on it
        self.tolerance = tolerance
        self.values = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def update(self, current, target):
        with self.lock:
            self.values = (current, target)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def run(self):
        # The GUI usually replies within a few ms, but can take hundreds when busy, so allow 2 s
        # (this thread does nothing else) and only report a missing GUI after 3 failures in a row.
        drawing, failures = False, 0
        while not self.stop_event.wait(1.0):
            with self.lock:
                values = self.values
            if values is None:
                continue
            ok, _ = self.gz.request('/marker_array', self.markers(*values),
                                    Marker_V, Boolean, 2000)
            if ok:
                if not drawing:
                    self.logger.info('power display: drawing on the north wall')
                drawing, failures = True, 0
            else:
                failures += 1
                if failures == 3:
                    if drawing:
                        self.logger.warning('power display: Gazebo GUI stopped responding; '
                                            'will keep trying')
                    else:
                        self.logger.info('power display: no Gazebo GUI to draw on yet')
                    drawing = False

    # -- drawing --------------------------------------------------------------------------------

    def colour_for(self, current, target):
        error = abs(current - target)
        if error <= self.tolerance:
            return GREEN
        if error <= 2 * self.tolerance:
            return AMBER
        return RED

    def markers(self, current, target):
        msg = Marker_V()
        self.next_id = 0
        colour = self.colour_for(current, target)
        # background panel
        self.box(msg, 0.0, 0.0, 9.4, 2.6, BLACK, y=self.y_back)
        # numbers: current (left, coloured) and target (right, white)
        self.number(msg, current, -2.3, 0.45, colour)
        self.number(msg, target, 2.3, 0.45, WHITE)
        # bar gauge
        left, bar_z = -self.BAR_LEN / 2, -0.75
        self.box(msg, 0.0, bar_z, self.BAR_LEN, 0.5, DIM)
        fill = self.BAR_LEN * min(max(current, 0.0), GAUGE_MAX) / GAUGE_MAX
        self.box(msg, left + fill / 2, bar_z, max(fill, 0.001), 0.5, colour, y=self.y_front - 0.01)
        for value, tick_colour, height in ((100.0, GREY, 0.7), (target, WHITE, 0.9)):
            u = left + self.BAR_LEN * min(max(value, 0.0), GAUGE_MAX) / GAUGE_MAX
            self.box(msg, u, bar_z, 0.08, height, tick_colour, y=self.y_front - 0.02)
        return msg

    def number(self, msg, value, u, v, colour):
        """Draw value as 'ddd.d' (leading blanks) centred at (u, v) in display coordinates."""
        text = f'{min(max(value, 0.0), 999.9):5.1f}'   # e.g. ' 39.9', '100.0'
        digits = text.replace('.', '')
        left = u - 1.5 * self.PITCH - 0.1
        for i, ch in enumerate(digits):
            cu = left + i * self.PITCH + (0.2 if i == 3 else 0.0)  # room for the decimal point
            on = DIGITS[ch]
            for name, ((sx, sz), horizontal) in SEGMENTS.items():
                w, h = ((self.DIGIT_W, self.SEG_T) if horizontal
                        else (self.SEG_T, self.DIGIT_H / 2))
                self.box(msg, cu + sx * self.DIGIT_W, v + sz * self.DIGIT_H, w, h,
                         colour if name in on else DIM)
        # decimal point, between the third and fourth digit
        dp_u = left + 2.5 * self.PITCH + 0.1
        self.box(msg, dp_u, v - self.DIGIT_H / 2, 0.12, 0.12, colour)

    def box(self, msg, u, v, width, height, colour, y=None):
        m = msg.marker.add()
        m.ns, m.id = NAMESPACE, self.next_id
        self.next_id += 1
        m.action, m.type = Marker.ADD_MODIFY, Marker.BOX
        m.pose.position.x = self.x + u
        m.pose.position.y = self.y_front if y is None else y
        m.pose.position.z = self.z + v
        m.scale.x, m.scale.y, m.scale.z = width, 0.02, height
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = (*colour, 1.0)


class PowerDisplayNode(Node):
    """Draws current_power_level and target_power_level, as reactor_monitor publishes them."""

    def __init__(self):
        super().__init__('power_display')
        p = self.declare_parameter
        # centre of the display on the north wall; the default is above the rooms against it
        x = float(p('power_display_x', 0.0).value)
        z = float(p('power_display_z', 3.6).value)
        tolerance = float(p('power_display_tolerance', 5.0).value)
        self.current = self.target = None
        # Requests only: this process must not subscribe to anything on gz-transport
        self.display = PowerDisplay(GzNode(), self.get_logger(), x=x, z=z, tolerance=tolerance)
        self.create_subscription(Float64, 'current_power_level', self.on_current, 10)
        self.create_subscription(Float64, 'target_power_level', self.on_target, 10)

    def on_current(self, msg):
        self.current = msg.data
        self.show()

    def on_target(self, msg):
        self.target = msg.data
        self.show()

    def show(self):
        if self.current is not None and self.target is not None:
            self.display.update(self.current, self.target)

    def destroy_node(self):
        self.display.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = PowerDisplayNode()
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
