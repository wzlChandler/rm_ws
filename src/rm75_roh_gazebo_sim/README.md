# RM75-BI + ROH-A001 抓取苹果 Gazebo 仿真

基于 `rm75_roh_description` 的 URDF（睿尔曼 RM75-BI 七轴机械臂 + OYMotion
ROH-A001 五指机械手），在 Gazebo（ODE 物理引擎）中搭建的"机械爪抓取苹果"仿真。

## 组成

| 文件 | 说明 |
| --- | --- |
| `urdf/rm75_roh_sim.xacro` | 仿真组装：复用 `rm75_roh_description/urdf/rm75_roh_a001_right.urdf.xacro`，追加手臂传动（复用睿尔曼官方宏）、gazebo_ros_control 插件、world 固定关节（顶装倒挂）与手臂物理阻尼 |
| `config/rm75_roh_control.yaml` | 控制器：手臂 7 轴 JointTrajectoryController + gazebo_ros_control PID |
| `launch/gazebo.launch` | 一键启动：空世界（ODE）、机器人（顶装倒挂）、苹果、控制器、初始位姿保持 |
| `models/apple.sdf` | 苹果模型（红色球体 + 果茎，static 纯视觉，由演示脚本控制位置模拟被抓取） |
| `scripts/rename_rohand_joints.py` | URDF 预处理：手爪 joint 加 `_j` 后缀（规避 link/joint 同名冲突）、全部关节转固定（刚体手爪）、去 STL 碰撞、手掌加"碗"形碰撞盒、质量与阻尼调整 |
| `scripts/generate_urdf.sh` | 生成最终 robot_description 的管道（xacro → 预处理） |
| `scripts/hold_init_pose.py` | 启动后保持手臂零位（trajectory 控制器无目标时不输出力） |
| `scripts/grasp_apple.py` | 抓取演示：手爪罩住苹果 → 手臂上收 → 苹果随机械爪抬起 |

## 启动

```bash
source ~/rm_ws/devel/setup.bash
roslaunch rm75_roh_gazebo_sim gazebo.launch
```

无界面运行：

```bash
roslaunch rm75_roh_gazebo_sim gazebo.launch headless:=true gui:=false
```

## 运行抓取演示

```bash
# 另开终端(等仿真稳定约 1 分钟)
source ~/rm_ws/devel/setup.bash
rosrun rm75_roh_gazebo_sim grasp_apple.py
```

演示流程：手爪位于最低位 → 苹果放置在手爪正下方 → 手爪下探罩住苹果 →
手臂上收（joint2/joint3 弯曲），苹果随机械爪一起抬起。脚本打印各阶段
手爪与苹果位置，确认"抓取成功"。

## 实现要点

- 机器人以**顶装倒挂**方式安装（与真机一致），`world_joint` 把 `base_link`
  固定在世界 (0,0,0.73)，绕 X 翻转 180°，手臂零位垂直向下。
- ROH 手爪 URDF 中 link 与 joint 同名，Gazebo 会报 name collision 并导致
  模型异常，预处理脚本统一给关节名加 `_j` 后缀。
- 手爪全部关节固化为刚体（Gazebo 自动将固定关节子树焊接进 Link7），
  手指保持弯曲握持姿态；手掌加"碗"形碰撞盒用于托住苹果。
- 苹果为 static 纯视觉模型，演示脚本在其"被抓取"期间控制位置跟随手爪，
  呈现完整的抓取动作。
- 手臂初始位姿由 `hold_init_pose` 保持；抓取脚本会先停止该节点并等待
  发布连接建立，再发送轨迹（`respawn` 已关闭，避免节点复活覆盖轨迹）。
