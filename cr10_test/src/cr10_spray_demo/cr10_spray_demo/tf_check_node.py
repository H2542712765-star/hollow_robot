#!/usr/bin/env python3
"""One-shot TF diagnostic for the CR10 spray demo."""

from __future__ import annotations

import copy
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfCheckNode(Node):
    def __init__(self) -> None:
        super().__init__('cr10_tf_check')
        self.declare_parameter('planning_frame', 'dummy_link')
        self.declare_parameter('tool_link', 'Link6')
        self.declare_parameter('timeout', 15.0)
        self.planning_frame = str(self.get_parameter('planning_frame').value).lstrip('/')
        self.tool_link = str(self.get_parameter('tool_link').value).lstrip('/')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def run(self, executor: SingleThreadedExecutor) -> bool:
        deadline = time.monotonic() + float(self.get_parameter('timeout').value)
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.planning_frame,
                    self.tool_link,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
                pose = PoseStamped()
                pose.header = copy.deepcopy(transform.header)
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
                self.get_logger().info(
                    f'TF 正常：{self.planning_frame} -> {self.tool_link}，'
                    f'xyz=({pose.pose.position.x:.6f}, '
                    f'{pose.pose.position.y:.6f}, '
                    f'{pose.pose.position.z:.6f})'
                )
                self.get_logger().info(
                    '当前 TF 缓冲区：\n' + self.tf_buffer.all_frames_as_yaml()
                )
                return True
            except TransformException:
                pass

        self.get_logger().error(
            f'TF 检查失败：{self.planning_frame} -> {self.tool_link}'
        )
        self.get_logger().error(
            '当前 TF 缓冲区：\n' + self.tf_buffer.all_frames_as_yaml()
        )
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfCheckNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ok = False
    try:
        ok = node.run(executor)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
