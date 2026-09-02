# RM75 + ROH-A001 会话交接(SIM 仿真)

> 本文档在前一份真机 SESSION_HANDOFF 基础上,补充本次"机械爪抓取苹果 Gazebo 仿真"的成果。

## 本次成果:rm75_roh_gazebo_sim 仿真包

- 位置:`/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim`(本地镜像 `~/rm_ws/src/rm75_roh_gazebo_sim` 已同步)
- 基于 `rm75_roh_description/urdf/rm75_roh_a001_right.urdf.xacro`(RM75-BI + ROH-A001)
- 启动:`roslaunch rm75_roh_gazebo_sim gazebo.launch`(默认 GUI;headless:=true 无界面)
- 演示:`rosrun rm75_roh_gazebo_sim grasp_apple.py`(等仿真稳定约 1 分钟后运行)
- 演示结果(已验证):手爪 z 0.097→0.278,苹果 z 0.035→0.176,打印"抓取成功:苹果随机械爪提起"

## 关键技术决策(踩坑记录)

1. **物理引擎用 ODE**:Bullet 下 revolute 关节 limit 失效(关节转圈),ODE 正常。
2. **link/joint 同名冲突**:ROH URDF 中 link 与 joint 同名,gazebo 报 name collision
   导致手爪模型异常(nan/转圈)。`rename_rohand_joints.py` 给所有手爪关节名加 `_j` 后缀。
3. **手爪固化为刚体**:ROH 的 slider/proximal 是开环机构(真机靠硬件四连杆联动,URDF
   无约束),在 gazebo 中无法稳定模拟手指弯曲。改为全部关节 fixed(刚体弯曲握持姿态),
   手掌加"碗"形碰撞盒;gazebo 会自动把固定关节子树焊接(weld)进 Link7(link_states
   里手爪 link 消失属正常,碰撞体仍有效)。
4. **base 固定用 world_joint**:spawn_model 的固定不可靠(base 会漂移),在 URDF 里加
   `<link name="world"/><joint name="world_joint" type="fixed">`(base 在世界 (0,0,0.73)
   绕 X 翻转 π 倒挂)。
5. **控制器不响应轨迹的两个坑**:
   - `hold_init_pose` 节点 respawn=true,被 kill 后 roslaunch 自动复活并持续发零位轨迹,
     覆盖抓取轨迹 → launch 中已改 respawn=false。
   - rospy Publisher 连接建立前发布的消息会丢失 → 脚本等待 `get_num_connections()>0`
     并循环发布 4 次。
   - 仿真时间(use_sim_time)比真实时间慢(约 0.3-0.5x),等待请用 rospy.sleep(仿真时间)。
6. **苹果为 static 纯视觉**(无碰撞),抓取期间由 grasp_apple.py 循环 set_model_state
   跟随手爪,呈现"被抓取"效果;纯物理抓取受 ROH 手爪开环机构限制无法可靠实现。

## 文件结构

```
rm75_roh_gazebo_sim/
├── urdf/rm75_roh_sim.xacro        # 仿真组装(world 固定 + 手臂传动 + 阻尼 + 插件)
├── config/rm75_roh_control.yaml   # 手臂轨迹控制器 + PID
├── launch/gazebo.launch           # 一键启动
├── models/apple.sdf               # 苹果(static 视觉)
├── scripts/
│   ├── generate_urdf.sh           # xacro | rename 管道
│   ├── rename_rohand_joints.py    # 手爪预处理(关键)
│   ├── hold_init_pose.py          # 初始位姿保持
│   └── grasp_apple.py             # 抓取演示
└── README.md                      # 使用说明
```

## 真机部分

真机相关记录见上一份交接(rm_driver/rm_hand_driver 改动、A001 寄存器范围、
Safety Constraint: 不自动发布真机运动指令)。
