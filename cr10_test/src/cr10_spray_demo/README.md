# cr10_spray_demo v0.3.0

适用于 ROS 2 Humble、Gazebo Classic 和 MoveIt 2 的 CR10 蛇形喷涂演示包。

## 已按当前 TF 树修正

当前实际 TF 链：

```text
world
└── dummy_link
    └── base_link
        └── Link1
            └── Link2
                └── Link3
                    └── Link4
                        └── Link5
                            └── Link6
```

MoveIt 模型的根帧是 `dummy_link`，因此规划、IK、RViz 和喷涂轨迹统一使用：

```yaml
planning_frame: dummy_link
tool_link: Link6
```

TF 名称严格区分大小写，`Link6` 不能写成 `link_6`。

## 主要改进

- 默认规划 TF 使用 `dummy_link -> Link6`。
- `use_sim_time` 默认设为 `true`，与 Gazebo 和当前 MoveIt 启动保持一致。
- launch 不再用旧默认值强制覆盖 YAML；只有显式传参时才覆盖。
- 启动时检查 TF 树并输出缺失帧、大小写建议和已知帧。
- 支持 Y-Z、X-Z、X-Y 等可配置喷涂平面。
- 增加喷涂尺寸、行数和速度参数检查。
- 在 RViz 发布 `/spray_path_preview` 轨迹预览。
- 运行失败时保持喷涂 OFF，并可尝试返回初始位姿。
- 增加独立 TF 检查节点。

## 编译

```bash
cd ~/hollow_robot/cr10_test
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  cra_description dobot_gazebo cr10_moveit cr10_spray_demo
source install/setup.bash
```

## 先检查 TF

保持 Gazebo、robot_state_publisher、控制器和 MoveIt 正在运行：

```bash
ros2 launch cr10_spray_demo tf_check.launch.py
```

预期看到：

```text
TF 正常：dummy_link -> Link6
```

也可以直接验证：

```bash
ros2 run tf2_ros tf2_echo dummy_link Link6
```

## 运行喷涂演示

只启动一套 Gazebo/控制器和一个 `move_group`。不要同时运行多个
`demo.launch.py`、`moveit_gazebo.launch.py` 或 `dobot_moveit.launch.py`。
完整系统启动后，在新终端执行：

```bash
cd ~/hollow_robot/cr10_test
source install/setup.bash
ros2 launch cr10_spray_demo spray_demo.launch.py
```

启动日志必须显示：

```text
有效配置：group=cr10_group，frame=dummy_link，tool=Link6
TF 已连接：dummy_link -> base_link -> Link1 -> ... -> Link6
```

如需临时覆盖参数：

```bash
ros2 launch cr10_spray_demo spray_demo.launch.py \
  planning_frame:=dummy_link \
  tool_link:=Link6
```

空的 launch 参数不会覆盖 `config/spray_demo.yaml`。

## RViz 显示规划喷涂路径

在 RViz 中：

1. `Add`。
2. 选择 `Marker`。
3. Topic 选择 `/spray_path_preview`。
4. Fixed Frame 使用 `dummy_link`。

## 默认演示轨迹

轨迹以启动时 `Link6` 位姿为中心，在 Y-Z 平面生成：

```yaml
horizontal_axis: y
vertical_axis: z
raster_width: 0.04
raster_height: 0.03
row_spacing: 0.015
```

末端姿态保持为启动时的姿态。首次测试不要增大轨迹尺寸。

## 喷涂输出

默认：

```yaml
publish_spray_command: false
```

此时机械臂会在 Gazebo 中运动，但仅记录喷涂 ON/OFF，不向实际喷枪发布命令。只有独立 IO/PLC 节点、安全区域、急停和速度限制全部验证后，才能改为 `true`。

## Pilz 检查

程序使用：

```text
PTP：到达喷涂起点、返回初始位姿
LIN：喷涂直线和行间过渡
```

如果 TF 通过但规划失败，检查 MoveIt 是否加载 Pilz pipeline，以及 `joint_limits.yaml` 和 `pilz_cartesian_limits.yaml` 是否有效。
