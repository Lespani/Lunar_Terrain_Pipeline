from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from .geotiff_map import GeoTiffTrafficabilityMap
from .layer_fusion import build_fused_trafficability_map, parse_layer_specs_json


class TrafficabilityNode(Node):
    def __init__(self) -> None:
        super().__init__("trafficability_node")

        self.declare_parameter("geotiff_path", "")
        self.declare_parameter("layers_json", "")
        self.declare_parameter("map_frame", "lroc_map")
        self.declare_parameter("localization_topic", "/localization/pose")
        self.declare_parameter("global_grid_topic", "/lunar/trafficability/global_grid")
        self.declare_parameter("local_patch_topic", "/lunar/trafficability/local_patch")
        self.declare_parameter("projected_point_topic", "/lunar/trafficability/rover_projected")
        self.declare_parameter("free_threshold", 0.35)
        self.declare_parameter("occupied_threshold", 0.65)
        self.declare_parameter("invert_values", False)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("patch_size_m", 20.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        geotiff_path = str(self.get_parameter("geotiff_path").value)
        layers_json = str(self.get_parameter("layers_json").value)
        free_threshold = float(self.get_parameter("free_threshold").value)
        occupied_threshold = float(self.get_parameter("occupied_threshold").value)
        unknown_value = int(self.get_parameter("unknown_value").value)

        if layers_json:
            layer_specs = parse_layer_specs_json(layers_json)
            self.traffic_map = build_fused_trafficability_map(
                specs=layer_specs,
                free_threshold=free_threshold,
                occupied_threshold=occupied_threshold,
                unknown_value=unknown_value,
            )
            self.get_logger().info(f"Loaded fused trafficability from {len(layer_specs)} layers.")
        else:
            if not geotiff_path:
                raise RuntimeError("Set geotiff_path (single-layer) or layers_json (multi-layer).")

            self.traffic_map = GeoTiffTrafficabilityMap.from_file(
                path=geotiff_path,
                free_threshold=free_threshold,
                occupied_threshold=occupied_threshold,
                invert=bool(self.get_parameter("invert_values").value),
                unknown_value=unknown_value,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        default_qos = QoSProfile(depth=10)

        self.global_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("global_grid_topic").value),
            latched_qos,
        )
        self.patch_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("local_patch_topic").value),
            default_qos,
        )
        self.point_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("projected_point_topic").value),
            default_qos,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("localization_topic").value),
            self.pose_callback,
            20,
        )

        self.publish_global_grid()
        self.get_logger().info("Trafficability node ready.")

    def _to_grid_message(
        self,
        array,
        origin_x: float,
        origin_y: float,
        frame_id: str,
    ) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.info.width = int(array.shape[1])
        msg.info.height = int(array.shape[0])
        msg.info.resolution = float(self.traffic_map.resolution_x)

        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = array.flatten(order="C").tolist()
        return msg

    def publish_global_grid(self) -> None:
        origin_x, origin_y = self.traffic_map.pixel_to_world(0, 0)
        global_msg = self._to_grid_message(
            self.traffic_map.data,
            origin_x,
            origin_y,
            self.map_frame,
        )
        self.global_pub.publish(global_msg)

    def _try_transform_pose(self, msg: PoseStamped) -> Optional[PoseStamped]:
        if msg.header.frame_id == self.map_frame:
            return msg

        try:
            return self.tf_buffer.transform(msg, self.map_frame)
        except TransformException as exc:
            self.get_logger().warning(
                f"Pose transform failed from {msg.header.frame_id} to {self.map_frame}: {exc}"
            )
            return None

    def _extract_xy(self, pose_msg: PoseStamped) -> Tuple[float, float]:
        return float(pose_msg.pose.position.x), float(pose_msg.pose.position.y)

    def pose_callback(self, msg: PoseStamped) -> None:
        map_pose = self._try_transform_pose(msg)
        if map_pose is None:
            return

        x, y = self._extract_xy(map_pose)
        if not self.traffic_map.contains_world_point(x, y):
            self.get_logger().warning("Projected rover pose is outside loaded GeoTIFF extent.")
            return

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.map_frame
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0
        self.point_pub.publish(point_msg)

        patch, origin_x, origin_y = self.traffic_map.extract_patch(
            center_x=x,
            center_y=y,
            size_m=float(self.get_parameter("patch_size_m").value),
        )

        patch_msg = self._to_grid_message(
            patch,
            origin_x,
            origin_y,
            self.map_frame,
        )
        patch_msg.header.stamp = point_msg.header.stamp
        self.patch_pub.publish(patch_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TrafficabilityNode] = None
    try:
        node = TrafficabilityNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        if node is None:
            temp_node = rclpy.create_node("trafficability_node_startup_error")
            temp_node.get_logger().error(str(exc))
            temp_node.destroy_node()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
