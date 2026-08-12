#!/usr/bin/env python3
"""CR10 autonomous raster spray demonstration for ROS 2 Humble + MoveIt 2.

This node talks directly to MoveIt's ``/move_action`` action server and does
not depend on ``moveit_py``.  It validates the live TF tree before planning,
previews the raster in RViz, approaches the first point with Pilz PTP, and
executes each spray stroke/transition with Pilz LIN.

The defaults in this package match the verified TF chain:

    world -> dummy_link -> base_link -> Link1 -> ... -> Link6

The spray output remains disabled by default; the robot can move in Gazebo,
but no hardware spray command is published until explicitly enabled.
"""

from __future__ import annotations

import copy
import difflib
import math
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class Cr10SprayDemo(Node):
    """Validate TF, generate a raster path, and execute it through MoveIt."""

    def __init__(self) -> None:
        super().__init__('cr10_spray_demo')

        # Robot and MoveIt configuration.
        self.declare_parameter('planning_group', 'cr10_group')
        self.declare_parameter('planning_frame', 'dummy_link')
        self.declare_parameter('tool_link', 'Link6')
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter(
            'pilz_pipeline_id', 'pilz_industrial_motion_planner'
        )
        self.declare_parameter('approach_planner_id', 'PTP')
        self.declare_parameter('linear_planner_id', 'LIN')
        self.declare_parameter('move_to_ready_state', True)
        self.declare_parameter(
            'ready_joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        )
        self.declare_parameter(
            'ready_joint_positions',
            [0.0, 0.3296, -1.3436, -0.3643, 1.6134, 0.0],
        )
        self.declare_parameter('ready_joint_tolerance', 0.01)

        # Startup and TF validation.
        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('move_group_timeout', 30.0)
        self.declare_parameter('require_single_move_action_server', True)
        self.declare_parameter('tf_timeout', 30.0)
        self.declare_parameter('log_tf_tree_on_start', True)

        # Raster geometry.  Default plane is Y-Z, with constant X.
        self.declare_parameter('horizontal_axis', 'y')
        self.declare_parameter('vertical_axis', 'z')
        self.declare_parameter('center_offset_x', 0.0)
        self.declare_parameter('center_offset_y', 0.0)
        self.declare_parameter('center_offset_z', 0.0)
        self.declare_parameter('raster_width', 0.04)
        self.declare_parameter('raster_height', 0.03)
        self.declare_parameter('row_spacing', 0.015)
        self.declare_parameter('max_pattern_extent', 0.20)
        self.declare_parameter('max_row_count', 20)

        # Motion settings.  These are deliberately conservative.
        self.declare_parameter('ptp_velocity_scaling', 0.05)
        self.declare_parameter('ptp_acceleration_scaling', 0.05)
        self.declare_parameter('lin_velocity_scaling', 0.03)
        self.declare_parameter('lin_acceleration_scaling', 0.03)
        self.declare_parameter('allowed_planning_time', 10.0)
        self.declare_parameter('planning_attempts', 3)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.05)
        self.declare_parameter('goal_send_timeout', 10.0)
        self.declare_parameter('motion_result_timeout', 120.0)
        self.declare_parameter('settle_time', 0.50)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('return_to_start_on_failure', False)

        # RViz trajectory preview.
        self.declare_parameter('publish_path_preview', True)
        self.declare_parameter('preview_topic', '/spray_path_preview')
        self.declare_parameter('preview_line_width', 0.006)

        # Spray output.  False means robot motion is allowed, but no hardware
        # command is published.
        self.declare_parameter('publish_spray_command', False)
        self.declare_parameter('spray_topic', '/spray_enable')
        self.declare_parameter('spray_on_delay', 0.10)
        self.declare_parameter('spray_off_delay', 0.10)

        self.group_name = self._parameter_string('planning_group')
        self.planning_frame = self._normalise_frame(
            self._parameter_string('planning_frame')
        )
        self.tool_link = self._normalise_frame(
            self._parameter_string('tool_link')
        )
        action_name = self._parameter_string('move_action_name')
        spray_topic = self._parameter_string('spray_topic')
        preview_topic = self._parameter_string('preview_topic')

        self.move_client = ActionClient(self, MoveGroup, action_name)
        self.spray_pub = self.create_publisher(Bool, spray_topic, 10)

        preview_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.preview_pub = self.create_publisher(
            Marker, preview_topic, preview_qos
        )

        # The listener uses this node's executor; all wait loops below spin the
        # executor so /tf and /tf_static callbacks can fill the buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

    def _parameter_string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _parameter_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    @staticmethod
    def _normalise_frame(frame: str) -> str:
        # tf2 frame IDs should not start with '/'.  Do not change case because
        # TF frame names are case-sensitive (Link6 != link_6).
        return frame.strip().lstrip('/')

    @staticmethod
    def _wait_for_future(
        executor: SingleThreadedExecutor,
        future,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            executor.spin_once(timeout_sec=0.05)
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                return False
        return future.done()

    def _spin_for(
        self, executor: SingleThreadedExecutor, duration_sec: float
    ) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.1, deadline - time.monotonic()))

    def validate_parameters(self) -> bool:
        errors: List[str] = []

        if not self.group_name:
            errors.append('planning_group 不能为空')
        if not self.planning_frame:
            errors.append('planning_frame 不能为空')
        if not self.tool_link:
            errors.append('tool_link 不能为空')

        ready_names = list(self.get_parameter('ready_joint_names').value)
        ready_positions = list(
            self.get_parameter('ready_joint_positions').value
        )
        if self._parameter_bool('move_to_ready_state'):
            if not ready_names:
                errors.append('ready_joint_names 不能为空')
            elif len(ready_names) != len(ready_positions):
                errors.append(
                    'ready_joint_names 与 ready_joint_positions 数量必须一致'
                )
            elif len(set(ready_names)) != len(ready_names):
                errors.append('ready_joint_names 不能包含重复关节')
            if self._parameter_float('ready_joint_tolerance') <= 0.0:
                errors.append('ready_joint_tolerance 必须大于 0')

        horizontal = self._parameter_string('horizontal_axis').lower()
        vertical = self._parameter_string('vertical_axis').lower()
        if horizontal not in {'x', 'y', 'z'}:
            errors.append('horizontal_axis 必须是 x、y 或 z')
        if vertical not in {'x', 'y', 'z'}:
            errors.append('vertical_axis 必须是 x、y 或 z')
        if horizontal == vertical:
            errors.append('horizontal_axis 与 vertical_axis 不能相同')

        width = self._parameter_float('raster_width')
        height = self._parameter_float('raster_height')
        spacing = self._parameter_float('row_spacing')
        max_extent = self._parameter_float('max_pattern_extent')
        if width <= 0.0 or height <= 0.0 or spacing <= 0.0:
            errors.append('raster_width、raster_height、row_spacing 必须大于 0')
        if max_extent <= 0.0:
            errors.append('max_pattern_extent 必须大于 0')
        elif width > max_extent or height > max_extent:
            errors.append(
                f'喷涂尺寸超过 max_pattern_extent={max_extent:.3f} m'
            )

        for name in (
            'ptp_velocity_scaling',
            'ptp_acceleration_scaling',
            'lin_velocity_scaling',
            'lin_acceleration_scaling',
        ):
            value = self._parameter_float(name)
            if value <= 0.0 or value > 1.0:
                errors.append(f'{name} 必须位于 (0, 1]')

        if self._parameter_int('max_row_count') < 2:
            errors.append('max_row_count 必须至少为 2')

        if errors:
            for error in errors:
                self.get_logger().error(f'参数错误：{error}。')
            return False
        return True

    def wait_for_move_group(self, executor: SingleThreadedExecutor) -> bool:
        timeout_sec = self._parameter_float('move_group_timeout')
        action_name = self._parameter_string('move_action_name')
        self.get_logger().info(f'等待 MoveIt 动作服务器 {action_name}……')
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.move_client.wait_for_server(timeout_sec=0.2):
                status_topic = action_name.rstrip('/') + '/_action/status'
                publishers = self.get_publishers_info_by_topic(status_topic)
                discovery_deadline = time.monotonic() + 1.0
                while (
                    not publishers
                    and rclpy.ok()
                    and time.monotonic() < discovery_deadline
                ):
                    executor.spin_once(timeout_sec=0.05)
                    publishers = self.get_publishers_info_by_topic(status_topic)
                servers = sorted({
                    (
                        info.node_namespace.rstrip('/') + '/' + info.node_name
                    ).replace('//', '/')
                    for info in publishers
                })
                if (
                    self._parameter_bool('require_single_move_action_server')
                    and len(publishers) != 1
                ):
                    server_list = ', '.join(servers) if servers else '未发现'
                    self.get_logger().error(
                        f'{action_name} 应当只有一个动作服务器，当前发现 '
                        f'{len(publishers)} 个：{server_list}。请关闭重复的 '
                        'move_group 启动文件。'
                    )
                    return False
                self.get_logger().info(
                    'MoveIt 动作服务器已连接：'
                    f'{", ".join(servers) if servers else action_name}。'
                )
                return True
            executor.spin_once(timeout_sec=0.05)
        self.get_logger().error(
            f'在 {timeout_sec:.1f} 秒内未连接到 {action_name}。'
        )
        return False

    def _tf_tree(self) -> Tuple[str, Set[str], Dict[str, str]]:
        """Return raw TF YAML, all known frames, and child->parent mapping."""
        raw = self.tf_buffer.all_frames_as_yaml()
        frames: Set[str] = set()
        parents: Dict[str, str] = {}
        try:
            data = yaml.safe_load(raw) or {}
            if isinstance(data, dict):
                for child, details in data.items():
                    child_name = self._normalise_frame(str(child))
                    frames.add(child_name)
                    if isinstance(details, dict):
                        parent = details.get('parent')
                        if parent:
                            parent_name = self._normalise_frame(str(parent))
                            frames.add(parent_name)
                            parents[child_name] = parent_name
        except yaml.YAMLError:
            pass
        return raw, frames, parents

    def _frame_suggestion(self, requested: str, frames: Sequence[str]) -> str:
        if not frames:
            return '当前节点尚未收到任何 TF 帧'

        case_matches = [frame for frame in frames if frame.lower() == requested.lower()]
        if case_matches:
            return f'注意大小写，可能应为：{case_matches[0]}'

        close = difflib.get_close_matches(requested, list(frames), n=3, cutoff=0.35)
        if close:
            return '相近的 TF 帧：' + '、'.join(close)
        return '已知 TF 帧：' + '、'.join(sorted(frames))

    def _describe_chain(self, parents: Dict[str, str]) -> str:
        chain = [self.tool_link]
        visited = {self.tool_link}
        current = self.tool_link
        while current in parents:
            current = parents[current]
            if current in visited:
                return 'TF 链存在循环，无法解析'
            chain.append(current)
            visited.add(current)
            if current == self.planning_frame:
                return ' -> '.join(reversed(chain))
        return f'未从 {self.tool_link} 回溯到 {self.planning_frame}'

    def wait_for_tool_transform(
        self, executor: SingleThreadedExecutor
    ) -> Optional[PoseStamped]:
        timeout_sec = self._parameter_float('tf_timeout')
        deadline = time.monotonic() + timeout_sec
        next_log = 0.0
        last_error = 'TF 缓冲区尚未收到目标变换'

        self.get_logger().info(
            f'等待 TF：{self.planning_frame} -> {self.tool_link}……'
        )

        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            try:
                if self.tf_buffer.can_transform(
                    self.planning_frame,
                    self.tool_link,
                    Time(),
                    timeout=Duration(seconds=0.0),
                ):
                    transform = self.tf_buffer.lookup_transform(
                        self.planning_frame,
                        self.tool_link,
                        Time(),
                        timeout=Duration(seconds=0.0),
                    )
                    pose = PoseStamped()
                    pose.header.frame_id = self.planning_frame
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = copy.deepcopy(
                        transform.transform.rotation
                    )

                    raw, _, parents = self._tf_tree()
                    self.get_logger().info(
                        'TF 已连接：' + self._describe_chain(parents)
                    )
                    if self._parameter_bool('log_tf_tree_on_start'):
                        self.get_logger().info(f'当前 TF 缓冲区：\n{raw}')
                    return pose

                last_error = (
                    f'当前 TF 树不能连接 {self.planning_frame} 与 '
                    f'{self.tool_link}'
                )
            except TransformException as exc:
                last_error = str(exc)

            now = time.monotonic()
            if now >= next_log:
                _, frames, _ = self._tf_tree()
                self.get_logger().info(
                    f'仍在等待 TF（已知 {len(frames)} 个帧）……'
                )
                next_log = now + 2.0

        raw, frames, _ = self._tf_tree()
        self.get_logger().error(
            f'无法获得 TF：{self.planning_frame} -> {self.tool_link}。'
        )
        self.get_logger().error(f'TF 详细原因：{last_error}')
        if self.planning_frame not in frames:
            self.get_logger().error(
                'planning_frame 不存在。'
                + self._frame_suggestion(self.planning_frame, sorted(frames))
            )
        if self.tool_link not in frames:
            self.get_logger().error(
                'tool_link 不存在。'
                + self._frame_suggestion(self.tool_link, sorted(frames))
            )
        self.get_logger().error(f'当前节点收到的 TF 帧：\n{raw}')
        return None

    @staticmethod
    def _set_axis(point, axis: str, value: float) -> None:
        setattr(point, axis, value)

    def generate_raster(self, reference: PoseStamped) -> List[List[PoseStamped]]:
        width = self._parameter_float('raster_width')
        height = self._parameter_float('raster_height')
        spacing = self._parameter_float('row_spacing')
        horizontal_axis = self._parameter_string('horizontal_axis').lower()
        vertical_axis = self._parameter_string('vertical_axis').lower()

        center = {
            'x': reference.pose.position.x + self._parameter_float('center_offset_x'),
            'y': reference.pose.position.y + self._parameter_float('center_offset_y'),
            'z': reference.pose.position.z + self._parameter_float('center_offset_z'),
        }

        row_count = max(2, int(math.ceil(height / spacing)) + 1)
        max_rows = self._parameter_int('max_row_count')
        if row_count > max_rows:
            raise ValueError(
                f'轨迹需要 {row_count} 行，超过 max_row_count={max_rows}'
            )
        actual_spacing = height / float(row_count - 1)

        rows: List[List[PoseStamped]] = []
        for index in range(row_count):
            vertical_value = center[vertical_axis] - height / 2.0 + (
                index * actual_spacing
            )
            low = center[horizontal_axis] - width / 2.0
            high = center[horizontal_axis] + width / 2.0
            horizontal_start, horizontal_end = (
                (low, high) if index % 2 == 0 else (high, low)
            )

            start = copy.deepcopy(reference)
            end = copy.deepcopy(reference)
            for axis in ('x', 'y', 'z'):
                self._set_axis(start.pose.position, axis, center[axis])
                self._set_axis(end.pose.position, axis, center[axis])

            self._set_axis(start.pose.position, vertical_axis, vertical_value)
            self._set_axis(end.pose.position, vertical_axis, vertical_value)
            self._set_axis(
                start.pose.position, horizontal_axis, horizontal_start
            )
            self._set_axis(end.pose.position, horizontal_axis, horizontal_end)
            rows.append([start, end])

        self.get_logger().info(
            f'已生成蛇形轨迹：参考帧={self.planning_frame}，'
            f'平面={horizontal_axis.upper()}-{vertical_axis.upper()}，'
            f'{row_count} 行，宽={width:.3f} m，高={height:.3f} m，'
            f'实际行距={actual_spacing:.3f} m。'
        )
        return rows

    def publish_path_preview(self, rows: List[List[PoseStamped]]) -> None:
        if not self._parameter_bool('publish_path_preview'):
            return

        marker = Marker()
        marker.header.frame_id = self.planning_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'cr10_spray_demo'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._parameter_float('preview_line_width')
        marker.color.r = 0.10
        marker.color.g = 0.90
        marker.color.b = 0.20
        marker.color.a = 1.0

        for row_index, (start, end) in enumerate(rows):
            if row_index == 0:
                marker.points.append(
                    Point(
                        x=start.pose.position.x,
                        y=start.pose.position.y,
                        z=start.pose.position.z,
                    )
                )
            marker.points.append(
                Point(
                    x=end.pose.position.x,
                    y=end.pose.position.y,
                    z=end.pose.position.z,
                )
            )
            if row_index + 1 < len(rows):
                next_start = rows[row_index + 1][0]
                marker.points.append(
                    Point(
                        x=next_start.pose.position.x,
                        y=next_start.pose.position.y,
                        z=next_start.pose.position.z,
                    )
                )

        self.preview_pub.publish(marker)
        self.get_logger().info(
            f'已发布 RViz 轨迹预览：{self._parameter_string("preview_topic")}。'
        )

    def make_pose_goal(self, pose: PoseStamped) -> Constraints:
        pos_tol = self._parameter_float('position_tolerance')
        ori_tol = self._parameter_float('orientation_tolerance')

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose.header.frame_id
        position_constraint.header.stamp = pose.header.stamp
        position_constraint.link_name = self.tool_link
        position_constraint.weight = 1.0

        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [2.0 * pos_tol, 2.0 * pos_tol, 2.0 * pos_tol]

        region_pose = Pose()
        region_pose.position = copy.deepcopy(pose.pose.position)
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = pose.header.frame_id
        orientation_constraint.header.stamp = pose.header.stamp
        orientation_constraint.link_name = self.tool_link
        orientation_constraint.orientation = copy.deepcopy(pose.pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = ori_tol
        orientation_constraint.absolute_y_axis_tolerance = ori_tol
        orientation_constraint.absolute_z_axis_tolerance = ori_tol
        orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = 'spray_pose_goal'
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def build_motion_goal(
        self,
        target: PoseStamped,
        planner_id: str,
        velocity_scaling: float,
        acceleration_scaling: float,
    ) -> MoveGroup.Goal:
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.pipeline_id = self._parameter_string('pilz_pipeline_id')
        request.planner_id = planner_id
        request.num_planning_attempts = self._parameter_int('planning_attempts')
        request.allowed_planning_time = self._parameter_float(
            'allowed_planning_time'
        )
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.goal_constraints.append(self.make_pose_goal(target))

        options = PlanningOptions()
        options.plan_only = False
        options.look_around = False
        options.replan = True
        options.replan_attempts = 2
        options.replan_delay = 0.2
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def build_joint_goal(self) -> MoveGroup.Goal:
        names = list(self.get_parameter('ready_joint_names').value)
        positions = list(self.get_parameter('ready_joint_positions').value)
        tolerance = self._parameter_float('ready_joint_tolerance')

        constraints = Constraints()
        constraints.name = 'ready_joint_state'
        for name, position in zip(names, positions):
            joint = JointConstraint()
            joint.joint_name = str(name)
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.pipeline_id = self._parameter_string('pilz_pipeline_id')
        request.planner_id = self._parameter_string('approach_planner_id')
        request.num_planning_attempts = self._parameter_int('planning_attempts')
        request.allowed_planning_time = self._parameter_float(
            'allowed_planning_time'
        )
        request.max_velocity_scaling_factor = self._parameter_float(
            'ptp_velocity_scaling'
        )
        request.max_acceleration_scaling_factor = self._parameter_float(
            'ptp_acceleration_scaling'
        )
        request.goal_constraints.append(constraints)

        options = PlanningOptions()
        options.plan_only = False
        options.look_around = False
        options.replan = True
        options.replan_attempts = 2
        options.replan_delay = 0.2
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def move_to_ready_state(
        self, executor: SingleThreadedExecutor
    ) -> bool:
        if not self._parameter_bool('move_to_ready_state'):
            return True

        positions = list(self.get_parameter('ready_joint_positions').value)
        self.get_logger().info(
            '从当前关节状态移动到喷涂准备位：'
            + ', '.join(f'{value:.4f}' for value in positions)
        )
        send_future = self.move_client.send_goal_async(self.build_joint_goal())
        if not self._wait_for_future(
            executor,
            send_future,
            timeout_sec=self._parameter_float('goal_send_timeout'),
        ):
            self.get_logger().error('移动到喷涂准备位: 发送动作目标超时。')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('移动到喷涂准备位: MoveIt 拒绝了目标。')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            executor,
            result_future,
            timeout_sec=self._parameter_float('motion_result_timeout'),
        ):
            self.get_logger().error('移动到喷涂准备位: 执行超时，正在取消。')
            goal_handle.cancel_goal_async()
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error('移动到喷涂准备位: 未收到动作结果。')
            return False

        error_code = wrapped_result.result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                '移动到喷涂准备位: 失败，MoveItErrorCode='
                f'{error_code.val} ({self._moveit_error_name(error_code.val)})。'
            )
            return False

        self.get_logger().info('移动到喷涂准备位: 完成。')
        self._spin_for(executor, self._parameter_float('settle_time'))
        return True

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        for name in dir(MoveItErrorCodes):
            if name.startswith('_'):
                continue
            try:
                if isinstance(getattr(MoveItErrorCodes, name), int) and (
                    getattr(MoveItErrorCodes, name) == code
                ):
                    return name
            except (AttributeError, TypeError):
                continue
        return 'UNKNOWN'

    def execute_pose(
        self,
        executor: SingleThreadedExecutor,
        target: PoseStamped,
        planner_id: str,
        velocity_scaling: float,
        acceleration_scaling: float,
        label: str,
    ) -> bool:
        target.header.frame_id = self.planning_frame
        target.header.stamp = self.get_clock().now().to_msg()
        goal = self.build_motion_goal(
            target,
            planner_id,
            velocity_scaling,
            acceleration_scaling,
        )

        self.get_logger().info(
            f'{label}: pipeline={self._parameter_string("pilz_pipeline_id")}'
            f'，planner={planner_id}，xyz=('
            f'{target.pose.position.x:.3f}, '
            f'{target.pose.position.y:.3f}, '
            f'{target.pose.position.z:.3f})'
        )

        send_future = self.move_client.send_goal_async(goal)
        if not self._wait_for_future(
            executor,
            send_future,
            timeout_sec=self._parameter_float('goal_send_timeout'),
        ):
            self.get_logger().error(f'{label}: 发送动作目标超时。')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label}: MoveIt 拒绝了目标。')
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            executor,
            result_future,
            timeout_sec=self._parameter_float('motion_result_timeout'),
        ):
            self.get_logger().error(f'{label}: 规划或执行超时，正在取消。')
            goal_handle.cancel_goal_async()
            return False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(f'{label}: 未收到动作结果。')
            return False

        error_code = wrapped_result.result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            code_name = self._moveit_error_name(error_code.val)
            detail = f"MoveIt错误码: {error_code.val}"
            self.get_logger().error(
                f'{label}: 失败，MoveItErrorCode={error_code.val} '
                f'({code_name})，{detail}'
            )
            if planner_id in {'PTP', 'LIN', 'CIRC'}:
                self.get_logger().error(
                    '请确认 move_group 已加载 Pilz planning pipeline，且 '
                    'pilz_cartesian_limits.yaml 与 joint_limits.yaml 有效。'
                )
            return False

        self.get_logger().info(f'{label}: 完成。')
        self._spin_for(executor, self._parameter_float('settle_time'))
        return True

    def set_spray(self, enabled: bool) -> None:
        if self._parameter_bool('publish_spray_command'):
            msg = Bool()
            msg.data = enabled
            self.spray_pub.publish(msg)
            self.get_logger().info('喷涂输出：ON' if enabled else '喷涂输出：OFF')
        else:
            self.get_logger().info(
                '干运行：请求喷涂 ON（未发布硬件命令）'
                if enabled
                else '干运行：请求喷涂 OFF（未发布硬件命令）'
            )

        delay_name = 'spray_on_delay' if enabled else 'spray_off_delay'
        time.sleep(self._parameter_float(delay_name))

    def _attempt_return(
        self,
        executor: SingleThreadedExecutor,
        original_pose: PoseStamped,
        failure_return: bool,
    ) -> bool:
        should_return = self._parameter_bool('return_to_start')
        if failure_return:
            should_return = self._parameter_bool('return_to_start_on_failure')
        if not should_return:
            return True

        label = '失败后返回初始位姿' if failure_return else '返回初始位姿'
        return self.execute_pose(
            executor,
            original_pose,
            self._parameter_string('approach_planner_id'),
            self._parameter_float('ptp_velocity_scaling'),
            self._parameter_float('ptp_acceleration_scaling'),
            label,
        )

    def run(self, executor: SingleThreadedExecutor) -> bool:
        if not self.validate_parameters():
            return False

        self.get_logger().info(
            '有效配置：'
            f'group={self.group_name}，frame={self.planning_frame}，'
            f'tool={self.tool_link}，use_sim_time='
            f'{self.get_parameter("use_sim_time").value}'
        )

        self._spin_for(executor, self._parameter_float('startup_delay'))

        if not self.wait_for_move_group(executor):
            return False

        if not self.move_to_ready_state(executor):
            return False

        original_pose = self.wait_for_tool_transform(executor)
        if original_pose is None:
            return False

        self.get_logger().info(
            '初始工具位姿：'
            f'xyz=({original_pose.pose.position.x:.3f}, '
            f'{original_pose.pose.position.y:.3f}, '
            f'{original_pose.pose.position.z:.3f})，'
            f'frame={original_pose.header.frame_id}。'
        )

        try:
            rows = self.generate_raster(original_pose)
        except ValueError as exc:
            self.get_logger().error(f'轨迹生成失败：{exc}。')
            return False

        self.publish_path_preview(rows)
        self.set_spray(False)

        success = False
        approach_planner = self._parameter_string('approach_planner_id')
        linear_planner = self._parameter_string('linear_planner_id')
        ptp_v = self._parameter_float('ptp_velocity_scaling')
        ptp_a = self._parameter_float('ptp_acceleration_scaling')
        lin_v = self._parameter_float('lin_velocity_scaling')
        lin_a = self._parameter_float('lin_acceleration_scaling')

        try:
            if not self.execute_pose(
                executor,
                rows[0][0],
                approach_planner,
                ptp_v,
                ptp_a,
                '移动到喷涂起点',
            ):
                return False

            for row_index, (row_start, row_end) in enumerate(rows):
                if row_index > 0:
                    if not self.execute_pose(
                        executor,
                        row_start,
                        linear_planner,
                        lin_v,
                        lin_a,
                        f'第 {row_index + 1} 行过渡',
                    ):
                        return False

                self.set_spray(True)
                stroke_ok = self.execute_pose(
                    executor,
                    row_end,
                    linear_planner,
                    lin_v,
                    lin_a,
                    f'第 {row_index + 1} 行喷涂',
                )
                self.set_spray(False)
                if not stroke_ok:
                    return False

            success = True
            return_ok = self._attempt_return(
                executor, original_pose, failure_return=False
            )
            if not return_ok:
                success = False
                return False

            self.get_logger().info('CR10 自主喷涂演示完成。')
            return True
        finally:
            self.set_spray(False)
            if not success and self._parameter_bool('return_to_start_on_failure'):
                self.get_logger().warning('演示未完成，尝试返回初始位姿。')
                self._attempt_return(executor, original_pose, failure_return=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Cr10SprayDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    success = False
    try:
        success = node.run(executor)
    except KeyboardInterrupt:
        node.get_logger().warning('用户中断喷涂演示。')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(
    f'喷涂演示异常终止：{exc}'
        )
    finally:
        try:
            node.set_spray(False)
        except Exception:  # noqa: BLE001
            pass
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
