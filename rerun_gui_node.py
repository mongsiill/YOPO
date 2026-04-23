"""RerunGuiNode: ROS 2 → Rerun 브리지 노드 구현 (rerun_gui.py 진입점에서 사용)."""
from __future__ import annotations

import math
import sys
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except ImportError:
    Buffer = None  # type: ignore[misc, assignment]
    TransformException = Exception  # type: ignore[misc, assignment]
    TransformListener = None  # type: ignore[misc, assignment]

try:
    import rerun as rr
except ImportError as e:  # pragma: no cover
    print("rerun-sdk가 필요합니다: pip install rerun-sdk", file=sys.stderr)
    raise e

try:
    import rerun.blueprint as rrb
except ImportError:
    rrb = None

try:
    from cv_bridge import CvBridge

    _HAS_CV_BRIDGE = True
except (ImportError, AttributeError):
    # NumPy 2.x + ROS Humble cv_bridge(1.x ABI) → AttributeError: _ARRAY_API not found
    CvBridge = None  # type: ignore
    _HAS_CV_BRIDGE = False

try:
    import cv2

    _HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore
    _HAS_CV2 = False


def quat_to_rotmat(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _image_msg_to_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "rgb8":
        return raw.reshape((h, w, 3)).copy()
    if enc == "bgr8":
        arr = raw.reshape((h, w, 3))
        return arr[..., ::-1].copy()
    if enc == "rgba8":
        arr = raw.reshape((h, w, 4))
        return arr[..., :3].copy()
    if enc == "bgra8":
        arr = raw.reshape((h, w, 4))
        return arr[..., 2::-1].copy()
    if enc == "mono8":
        g = raw.reshape((h, w))
        return np.stack([g, g, g], axis=-1)
    if _HAS_CV_BRIDGE and CvBridge is not None:
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        return np.asarray(cv_img)
    raise ValueError(f"지원하지 않는 image encoding: {msg.encoding}")


def _compressed_to_rgb(msg: CompressedImage) -> np.ndarray:
    if not _HAS_CV2 or cv2 is None:
        raise RuntimeError("CompressedImage 처리를 위해 opencv-python 또는 python3-opencv가 필요합니다.")
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("압축 이미지 디코딩 실패")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb


def _pose_to_transform(msg: PoseStamped) -> rr.Transform3D:
    p = msg.pose.position
    o = msg.pose.orientation
    return rr.Transform3D(
        translation=[p.x, p.y, p.z],
        quaternion=rr.Quaternion(xyzw=[o.x, o.y, o.z, o.w]),
    )


class RerunGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("rerun_gui")

        self.declare_parameter("application_id", "yopo_pipeline")
        self.declare_parameter("rerun_spawn", True)
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("compressed_image_topic", "")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("markers_topic", "")
        self.declare_parameter("marker_topic", "")
        self.declare_parameter("camera_entity_path", "world/camera")
        self.declare_parameter("markers_entity_path", "world/detections")
        self.declare_parameter("max_fps", 20.0)
        self.declare_parameter("log_camera_pose", True)

        # rerun_bridge_node.py 호환 (토마토 스택)
        self.declare_parameter("tomato_stack_defaults", False)
        self.declare_parameter("mask_overlay_topic", "")
        self.declare_parameter("box2d_overlay_topic", "")
        self.declare_parameter("depth_map_topic", "")
        self.declare_parameter("pointcloud_topic", "")
        self.declare_parameter("tomato_bbox3d_topic", "")
        self.declare_parameter("vlm_annotated_topic", "")
        self.declare_parameter("reasoning_topic", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("lock_to_world", False)
        self.declare_parameter("log_tomato_camera_from_tf", True)
        self.declare_parameter("log_tomato_camera_axes", True)
        self.declare_parameter("camera_axis_length", 0.08)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("apply_tomato_blueprint", True)

        app_id = self.get_parameter("application_id").get_parameter_value().string_value
        spawn = self.get_parameter("rerun_spawn").get_parameter_value().bool_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        compressed_topic = self.get_parameter("compressed_image_topic").get_parameter_value().string_value
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        markers_topic = self.get_parameter("markers_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self._cam_path = self.get_parameter("camera_entity_path").get_parameter_value().string_value
        self._markers_path = self.get_parameter("markers_entity_path").get_parameter_value().string_value
        self._max_fps = max(0.1, self.get_parameter("max_fps").get_parameter_value().double_value)
        self._log_pose = self.get_parameter("log_camera_pose").get_parameter_value().bool_value

        tomato_defaults = self.get_parameter("tomato_stack_defaults").get_parameter_value().bool_value
        mask_overlay_topic = self.get_parameter("mask_overlay_topic").get_parameter_value().string_value
        box2d_overlay_topic = self.get_parameter("box2d_overlay_topic").get_parameter_value().string_value
        depth_map_topic = self.get_parameter("depth_map_topic").get_parameter_value().string_value
        pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        tomato_bbox3d_topic = self.get_parameter("tomato_bbox3d_topic").get_parameter_value().string_value
        vlm_annotated_topic = self.get_parameter("vlm_annotated_topic").get_parameter_value().string_value
        reasoning_topic = self.get_parameter("reasoning_topic").get_parameter_value().string_value

        if tomato_defaults:
            if not mask_overlay_topic:
                mask_overlay_topic = "/tomato/mask_overlay"
            if not box2d_overlay_topic:
                box2d_overlay_topic = "/tomato/box2d_overlay"
            if not depth_map_topic:
                depth_map_topic = "/tomato/depth_vis"
            if not pointcloud_topic:
                pointcloud_topic = "/tomato/pointcloud"
            if not tomato_bbox3d_topic:
                tomato_bbox3d_topic = "/tomato/box3d"
            if not vlm_annotated_topic:
                vlm_annotated_topic = "/tomato/vlm/annotated"
            if not reasoning_topic:
                reasoning_topic = "/tomato/vlm/reasoning"

        self._world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self._lock_to_world = self.get_parameter("lock_to_world").get_parameter_value().bool_value
        self._log_tomato_cam_tf = self.get_parameter("log_tomato_camera_from_tf").get_parameter_value().bool_value
        self._log_tomato_axes = self.get_parameter("log_tomato_camera_axes").get_parameter_value().bool_value
        self._axis_len = float(self.get_parameter("camera_axis_length").get_parameter_value().double_value)
        self._max_points = int(self.get_parameter("max_points").get_parameter_value().integer_value)
        self._tomato_cam_entity = self._cam_path.lstrip("/")
        apply_bp = self.get_parameter("apply_tomato_blueprint").get_parameter_value().bool_value

        self._warned_tf = False
        self._bridge = CvBridge() if _HAS_CV_BRIDGE and CvBridge is not None else None
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener = None
        need_tf = self._lock_to_world and (
            bool(mask_overlay_topic)
            or bool(box2d_overlay_topic)
            or bool(depth_map_topic)
            or bool(pointcloud_topic)
            or bool(tomato_bbox3d_topic)
        )
        if need_tf:
            if Buffer is None or TransformListener is None:
                self.get_logger().error("tf2_ros 가 없어 lock_to_world 를 사용할 수 없습니다.")
                self._lock_to_world = False
            else:
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)

        rr.init(app_id, spawn=spawn)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        if apply_bp and rrb is not None:
            self._apply_tomato_blueprint()

        self._last_image_log = 0.0
        self._last_marker_log = 0.0

        qos = qos_profile_sensor_data
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if compressed_topic:
            self.create_subscription(CompressedImage, compressed_topic, self._on_compressed, qos)
            self.get_logger().info(f"CompressedImage 구독: {compressed_topic}")
        elif image_topic:
            self.create_subscription(Image, image_topic, self._on_image, qos)
            self.get_logger().info(f"Image 구독: {image_topic}")
        else:
            self.get_logger().warn("image_topic 과 compressed_image_topic 이 모두 비어 있습니다.")

        if pose_topic and self._log_pose:
            self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)
            self.get_logger().info(f"PoseStamped 구독: {pose_topic}")

        if markers_topic:
            self.create_subscription(MarkerArray, markers_topic, self._on_markers, qos)
            self.get_logger().info(f"MarkerArray 구독: {markers_topic}")

        if marker_topic:
            self.create_subscription(Marker, marker_topic, self._on_marker, qos)
            self.get_logger().info(f"Marker 구독 (일반 경로): {marker_topic}")

        if mask_overlay_topic:
            self.create_subscription(Image, mask_overlay_topic, self._on_mask_overlay, image_qos)
            self.get_logger().info(f"mask_overlay 구독: {mask_overlay_topic}")
        if box2d_overlay_topic:
            self.create_subscription(Image, box2d_overlay_topic, self._on_box2d_overlay, image_qos)
            self.get_logger().info(f"box2d_overlay 구독: {box2d_overlay_topic}")
        if depth_map_topic:
            self.create_subscription(Image, depth_map_topic, self._on_depth_map, 10)
            self.get_logger().info(f"depth_vis 구독: {depth_map_topic}")
        if pointcloud_topic:
            self.create_subscription(PointCloud2, pointcloud_topic, self._on_pointcloud, 10)
            self.get_logger().info(f"pointcloud 구독: {pointcloud_topic}")
        if tomato_bbox3d_topic:
            self.create_subscription(Marker, tomato_bbox3d_topic, self._on_tomato_bbox3d, 10)
            self.get_logger().info(f"tomato bbox3d (Marker) 구독: {tomato_bbox3d_topic}")
        if vlm_annotated_topic:
            self.create_subscription(Image, vlm_annotated_topic, self._on_vlm_annotated, image_qos)
            self.get_logger().info(f"vlm annotated 구독: {vlm_annotated_topic}")
        if reasoning_topic:
            self.create_subscription(String, reasoning_topic, self._on_reasoning, 10)
            self.get_logger().info(f"reasoning(String) 구독: {reasoning_topic}")

    def _apply_tomato_blueprint(self) -> None:
        try:
            blueprint = rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(origin="/", name="3D View"),
                    rrb.Vertical(
                        rrb.Vertical(
                            rrb.Spatial2DView(origin="tomato/vlm/annotated", name="VLM Annotated"),
                            rrb.TextLogView(origin="tomato/vlm/reasoning", name="VLM Reasoning Log"),
                            row_shares=[0.85, 0.15],
                        ),
                        rrb.Spatial2DView(origin="tomato/box2d_overlay", name="Box2D overlay (detector)"),
                        rrb.Spatial2DView(origin="tomato/mask_overlay", name="Mask overlay (filtered)"),
                        rrb.Spatial2DView(origin="tomato/depth_vis", name="Depth Map"),
                        row_shares=[0.42, 0.19, 0.19, 0.20],
                    ),
                    column_shares=[0.58, 0.42],
                ),
                collapse_panels=True,
            )
            rr.send_blueprint(blueprint, make_active=True)
            self.get_logger().info("Rerun tomato 기본 blueprint 적용")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"blueprint 적용 생략: {e}")

    def _lookup_world_transform(self, from_frame: str, stamp) -> Optional[object]:
        if not self._lock_to_world or self._tf_buffer is None:
            return None
        try:
            return self._tf_buffer.lookup_transform(self._world_frame, from_frame, stamp)
        except TransformException:
            try:
                return self._tf_buffer.lookup_transform(self._world_frame, from_frame, Time())
            except TransformException as error:
                if not self._warned_tf:
                    self.get_logger().warning(
                        f"TF lookup 실패 ({from_frame}->{self._world_frame}): {error}. "
                        "map→camera TF 를 확인하거나 lock_to_world:=false 로 시도하세요."
                    )
                    self._warned_tf = True
                return None

    def _apply_transform_points(self, points: np.ndarray, transform) -> np.ndarray:
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = quat_to_rotmat(q.x, q.y, q.z, q.w)
        translated = (rot @ points.T).T + np.array([t.x, t.y, t.z], dtype=np.float64)
        return translated.astype(np.float32)

    def _apply_transform_point(self, p: Point, transform) -> np.ndarray:
        points = np.array([[p.x, p.y, p.z]], dtype=np.float64)
        return self._apply_transform_points(points, transform)[0]

    def _log_camera_pose_from_tf(self, tf_msg) -> None:
        if not self._log_tomato_cam_tf:
            return
        try:
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            rot = quat_to_rotmat(q.x, q.y, q.z, q.w)
            rr.log(
                self._tomato_cam_entity,
                rr.Transform3D(translation=[float(t.x), float(t.y), float(t.z)], mat3x3=rot),
            )
            if self._log_tomato_axes and self._axis_len > 0.0:
                L = float(self._axis_len)
                rr.log(
                    f"{self._tomato_cam_entity}/axes",
                    rr.Arrows3D(
                        vectors=[[L, 0.0, 0.0], [0.0, L, 0.0], [0.0, 0.0, L]],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                    ),
                )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"camera pose (TF) 로깅 실패: {e}")

    def _tomato_bgr_to_rgb_log(self, msg: Image, rerun_path: str, log_tf: bool) -> None:
        if self._bridge is None:
            self.get_logger().error("CvBridge 가 필요합니다 (토마토 Image 토픽). ros-humble-cv-bridge 설치.")
            return
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = bgr[:, :, ::-1]
        rr.log(rerun_path, rr.Image(rgb))
        if log_tf and self._lock_to_world and msg.header.frame_id:
            tf = self._lookup_world_transform(msg.header.frame_id, msg.header.stamp)
            if tf is not None:
                self._log_camera_pose_from_tf(tf)

    def _throttle(self, last_attr: str) -> bool:
        now = time.monotonic()
        min_dt = 1.0 / self._max_fps
        prev = getattr(self, last_attr)
        if now - prev < min_dt:
            return True
        setattr(self, last_attr, now)
        return False

    def _on_image(self, msg: Image) -> None:
        if self._throttle("_last_image_log"):
            return
        try:
            rgb = _image_msg_to_rgb(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"이미지 변환 실패: {e}")
            return
        rr.log(f"{self._cam_path}/image", rr.Image(rgb))

    def _on_compressed(self, msg: CompressedImage) -> None:
        if self._throttle("_last_image_log"):
            return
        try:
            rgb = _compressed_to_rgb(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"압축 이미지 처리 실패: {e}")
            return
        rr.log(f"{self._cam_path}/image", rr.Image(rgb))

    def _on_pose(self, msg: PoseStamped) -> None:
        try:
            tf = _pose_to_transform(msg)
            rr.log(self._cam_path, tf)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"pose 로깅 실패: {e}")

    def _on_mask_overlay(self, msg: Image) -> None:
        try:
            self._tomato_bgr_to_rgb_log(msg, "tomato/mask_overlay", True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"mask_overlay 로깅 실패: {e}")

    def _on_box2d_overlay(self, msg: Image) -> None:
        try:
            self._tomato_bgr_to_rgb_log(msg, "tomato/box2d_overlay", True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"box2d_overlay 로깅 실패: {e}")

    def _on_depth_map(self, msg: Image) -> None:
        try:
            self._tomato_bgr_to_rgb_log(msg, "tomato/depth_vis", True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"depth_vis 로깅 실패: {e}")

    def _on_vlm_annotated(self, msg: Image) -> None:
        try:
            self._tomato_bgr_to_rgb_log(msg, "tomato/vlm/annotated", True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"vlm annotated 로깅 실패: {e}")

    def _on_reasoning(self, msg: String) -> None:
        try:
            text = (msg.data or "").strip()
            if not text:
                return
            rr.log("tomato/vlm/reasoning", rr.TextLog(text))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"reasoning 로깅 실패: {e}")

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        try:
            field_names = [f.name for f in msg.fields]
            color_field = None
            if "rgb" in field_names:
                color_field = "rgb"
            elif "rgba" in field_names:
                color_field = "rgba"

            read_names = ("x", "y", "z", color_field) if color_field is not None else ("x", "y", "z")
            raw = point_cloud2.read_points(msg, field_names=read_names, skip_nans=True)
            arr = np.asarray(raw)
            if arr.size == 0:
                return

            colors = None
            if arr.dtype.fields is not None:
                points = np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32, copy=False)
                if color_field is not None and color_field in arr.dtype.fields:
                    packed = np.asarray(arr[color_field])
                    if np.issubdtype(packed.dtype, np.floating):
                        packed_u32 = packed.astype(np.float32, copy=False).view(np.uint32)
                    else:
                        packed_u32 = packed.astype(np.uint32, copy=False)
                    r = ((packed_u32 >> 16) & 0xFF).astype(np.uint8)
                    g = ((packed_u32 >> 8) & 0xFF).astype(np.uint8)
                    b = (packed_u32 & 0xFF).astype(np.uint8)
                    colors = np.column_stack((r, g, b))
            else:
                points = np.asarray(arr[..., :3], dtype=np.float32)
                if points.ndim == 1:
                    points = points.reshape(-1, 3)

            if self._max_points > 0 and points.shape[0] > self._max_points:
                step = int(math.ceil(points.shape[0] / float(self._max_points)))
                points = points[::step]
                if colors is not None:
                    colors = colors[::step]

            frame_name = msg.header.frame_id
            if self._lock_to_world:
                tf = self._lookup_world_transform(frame_name, msg.header.stamp)
                if tf is None:
                    return
                self._log_camera_pose_from_tf(tf)
                points = self._apply_transform_points(points, tf)

            if colors is not None:
                rr.log("tomato/pointcloud", rr.Points3D(points, colors=colors))
            else:
                rr.log("tomato/pointcloud", rr.Points3D(points))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"pointcloud 로깅 실패: {e}")

    def _on_tomato_bbox3d(self, msg: Marker) -> None:
        if msg.type not in (Marker.CUBE, Marker.SPHERE):
            return
        if msg.action != Marker.ADD:
            return
        try:
            center = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)
            size = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)

            if self._lock_to_world:
                tf = self._lookup_world_transform(msg.header.frame_id, msg.header.stamp)
                if tf is None:
                    return
                self._log_camera_pose_from_tf(tf)
                cp = Point()
                cp.x, cp.y, cp.z = float(center[0]), float(center[1]), float(center[2])
                center = self._apply_transform_point(cp, tf)

            half = (size * 0.5).tolist()
            c = [center.tolist()]
            colors = [[255, 0, 0]]
            box = rr.Boxes3D(centers=c, half_sizes=[half], colors=colors)
            if msg.type == Marker.SPHERE and hasattr(rr, "Ellipsoids3D"):
                try:
                    rr.log("tomato/bbox3d", rr.Ellipsoids3D(centers=c, half_sizes=[half], colors=colors))
                except Exception:
                    rr.log("tomato/bbox3d", box)
            else:
                rr.log("tomato/bbox3d", box)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"tomato bbox3d 로깅 실패: {e}")

    def _on_markers(self, msg: MarkerArray) -> None:
        if self._throttle("_last_marker_log"):
            return
        for i, m in enumerate(msg.markers):
            if m.action != Marker.ADD:
                continue
            base = f"{self._markers_path}/marker_{m.id or i}"
            self._log_single_marker(base, m)

    def _on_marker(self, m: Marker) -> None:
        if self._throttle("_last_marker_log"):
            return
        if m.action != Marker.ADD:
            return
        base = f"{self._markers_path}/marker_{m.id}"
        self._log_single_marker(base, m)

    def _log_single_marker(self, entity: str, m: Marker) -> None:
        if m.type == Marker.LINE_STRIP or m.type == Marker.LINE_LIST:
            pts = [(p.x, p.y, p.z) for p in m.points]
            if len(pts) < 2:
                return
            strips = np.array([pts], dtype=np.float32)
            rr.log(entity, rr.LineStrips3D(strips, radii=0.005, colors=[[255, 80, 80]]))
        elif m.type == Marker.CUBE:
            t = m.pose.position
            s = m.scale
            hx, hy, hz = abs(s.x) * 0.5, abs(s.y) * 0.5, abs(s.z) * 0.5
            if hx < 1e-6 or hy < 1e-6 or hz < 1e-6:
                return
            q = m.pose.orientation
            rr.log(
                entity,
                rr.Boxes3D(
                    centers=[[t.x, t.y, t.z]],
                    half_sizes=[[hx, hy, hz]],
                    quaternions=[rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])],
                    colors=[(80, 200, 255)],
                ),
            )
        elif m.type == Marker.SPHERE:
            t = m.pose.position
            r = max(m.scale.x, m.scale.y, m.scale.z) * 0.5
            if r < 1e-6:
                return
            rr.log(entity, rr.Points3D([[t.x, t.y, t.z]], radii=r, colors=[[255, 200, 80]]))
