#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np


class MapColorOverlay(Node):
    def __init__(self):
        super().__init__('map_color_overlay')

        # --- Parameters ---
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('marker_topic', '/map_color_markers')

        # Relaxed green detection
        # Slightly relaxed defaults (compared to the tightened version)
        self.declare_parameter('min_green', 22)
        self.declare_parameter('green_ratio', 1.10)
        self.declare_parameter('green_diff', 14)

        # Height filter: accept z<0 if enabled; otherwise apply band
        self.declare_parameter('negative_z_only', True)
        self.declare_parameter('accept_negative_z', True)
        self.declare_parameter('min_z', -0.1)
        self.declare_parameter('max_z', 0.1)

        # 2D overlay marker on map
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('marker_layer_z', 0.0)
        # Smaller grid tiles for finer overlay
        self.declare_parameter('marker_size', 0.04)
        self.declare_parameter('marker_thickness', 0.01)
        self.declare_parameter('marker_alpha', 1.0)

        # TF + robustness
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('fallback_to_cloud_frame', True)
        self.declare_parameter('republish_hz', 1.0)

        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        self.min_green = self.get_parameter('min_green').get_parameter_value().integer_value
        self.green_ratio = self.get_parameter('green_ratio').get_parameter_value().double_value
        self.green_diff = self.get_parameter('green_diff').get_parameter_value().integer_value

        self.negative_z_only = self.get_parameter('negative_z_only').get_parameter_value().bool_value
        self.accept_negative_z = self.get_parameter('accept_negative_z').get_parameter_value().bool_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value

        self.marker_frame = self.get_parameter('marker_frame').get_parameter_value().string_value
        self.marker_layer_z = self.get_parameter('marker_layer_z').get_parameter_value().double_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.marker_thickness = self.get_parameter('marker_thickness').get_parameter_value().double_value
        self.marker_alpha = self.get_parameter('marker_alpha').get_parameter_value().double_value

        self.tf_timeout_sec = self.get_parameter('tf_timeout_sec').get_parameter_value().double_value
        self.fallback_to_cloud_frame = self.get_parameter('fallback_to_cloud_frame').get_parameter_value().bool_value
        self.republish_hz = self.get_parameter('republish_hz').get_parameter_value().double_value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._warned_tf = False

        # QoS: make marker "latched" like maps (Transient Local)
        self._marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos_profile=self._marker_qos)

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, qos_profile=cloud_qos)

        self._last_marker: Marker | None = None
        if self.republish_hz and self.republish_hz > 0.0:
            self.create_timer(1.0 / float(self.republish_hz), self._republish_cb)

        self.get_logger().info(f'Subscribed to {self.cloud_topic}, publishing markers to {self.marker_topic}')

    def _republish_cb(self):
        if self._last_marker is None:
            return
        self._last_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self._last_marker)

    def cloud_cb(self, cloud: PointCloud2):
        fields = [f.name for f in cloud.fields]
        if 'rgb' not in fields:
            return

        gen = pc2.read_points(cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        green_points_xyz = []

        for x, y, z, rgb_float in gen:
            # Height filter
            if self.negative_z_only:
                if z >= 0.0:
                    continue
            else:
                if z < 0.0:
                    if not self.accept_negative_z:
                        continue
                else:
                    if not (self.min_z <= z <= self.max_z):
                        continue

            # Decode RGB packed float
            try:
                rgb_int = int(np.float32(rgb_float).view(np.uint32))
            except Exception:
                continue

            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = (rgb_int) & 0xFF

            # Tightened green detection:
            # Require brightness + ratio dominance + absolute dominance simultaneously.
            is_green = (
                (g >= self.min_green)
                and (g >= r * self.green_ratio)
                and (g >= b * self.green_ratio)
                and ((g - max(r, b)) >= self.green_diff)
            )

            # Legacy dark-green heuristic (slightly relaxed)
            if not is_green and g >= self.min_green and g > r + 8 and g > b + 8:
                is_green = True

            if is_green:
                green_points_xyz.append((float(x), float(y), float(z)))

        if not green_points_xyz:
            return

        # Transform into marker_frame so it sticks on 2D map
        source_frame = (cloud.header.frame_id or '').strip()
        target_frame = (self.marker_frame or '').strip()
        publish_frame = target_frame if target_frame else source_frame

        points_xy = None
        if source_frame and target_frame and source_frame != target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=float(self.tf_timeout_sec)),
                )
                points_xy = [
                    self._apply_transform_xy(x, y, z, tf)[:2]
                    for (x, y, z) in green_points_xyz
                ]
                if self._warned_tf:
                    self.get_logger().info(f'TF recovered: {source_frame} -> {target_frame}')
                    self._warned_tf = False
            except TransformException as ex:
                if not self._warned_tf:
                    self.get_logger().warning(
                        f'No TF from {source_frame} -> {target_frame} (timeout={self.tf_timeout_sec}s). '
                        f'fallback_to_cloud_frame={self.fallback_to_cloud_frame}. Error: {ex}'
                    )
                    self._warned_tf = True
                if self.fallback_to_cloud_frame:
                    publish_frame = source_frame
                    points_xy = [(x, y) for (x, y, _z) in green_points_xyz]
                else:
                    return
        else:
            points_xy = [(x, y) for (x, y, _z) in green_points_xyz]

        # Quantize to grid + flatten Z to 2D layer
        cell_set = set()
        cells = []
        thickness = max(float(self.marker_thickness), 0.001)
        # Flat tiles sitting on a 2D layer
        z_out = float(self.marker_layer_z) + thickness * 0.5

        for x, y in points_xy:
            gx = round(x / self.marker_size) * self.marker_size
            gy = round(y / self.marker_size) * self.marker_size
            key = (int(round(gx * 1000.0)), int(round(gy * 1000.0)))
            if key in cell_set:
                continue
            cell_set.add(key)
            p = Point()
            p.x = float(gx)
            p.y = float(gy)
            p.z = float(z_out)
            cells.append(p)

        if not cells:
            return

        marker = Marker()
        marker.header.frame_id = publish_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "green_overlay"
        marker.id = 0
        # Revert to grid tile style (as before)
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0.0).to_msg()
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.marker_size)
        marker.scale.y = float(self.marker_size)
        marker.scale.z = float(thickness)

        # Bright opaque green for visibility
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = float(self.marker_alpha)
        marker.points = cells

        self.marker_pub.publish(marker)
        self._last_marker = marker

    @staticmethod
    def _apply_transform_xy(x: float, y: float, z: float, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)

        # Rotation matrix from quaternion
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rx = (1.0 - 2.0 * (yy + zz)) * x + (2.0 * (xy - wz)) * y + (2.0 * (xz + wy)) * z
        ry = (2.0 * (xy + wz)) * x + (1.0 - 2.0 * (xx + zz)) * y + (2.0 * (yz - wx)) * z
        rz = (2.0 * (xz - wy)) * x + (2.0 * (yz + wx)) * y + (1.0 - 2.0 * (xx + yy)) * z

        return (rx + float(t.x), ry + float(t.y), rz + float(t.z))


def main(args=None):
    rclpy.init(args=args)
    node = MapColorOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()