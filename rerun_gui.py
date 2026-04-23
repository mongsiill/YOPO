#!/usr/bin/env python3
# Copyright: minimal host bridge for YOPO pipeline + optional EdgeTAM_bbox rerun_bridge parity.
"""
호스트에서 ROS 2 토픽을 구독해 Rerun에 기록합니다.

구현은 rerun_gui_node.py 의 RerunGuiNode 를 참고하세요.

  source /opt/ros/humble/setup.bash
  pip install --user rerun-sdk  # 또는 requirements-rerun.txt

  YOPO 단독:
  python3 rerun_gui.py --ros-args -p image_topic:=/your/camera/image

  EdgeTAM_bbox / rerun_bridge_node 와 동일 레이아웃·토픽:
  python3 rerun_gui.py --ros-args -p tomato_stack_defaults:=true -p lock_to_world:=true

의존: rclpy, sensor_msgs, geometry_msgs, visualization_msgs, tf2_ros, sensor_msgs_py,
      cv_bridge(토마토 Image 토픽 사용 시). perception/desktop 메타패키지에 포함되는 경우가 많습니다.
"""
from __future__ import annotations

from typing import Optional

import rclpy

from rerun_gui_node import RerunGuiNode


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[RerunGuiNode] = None
    try:
        node = RerunGuiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
