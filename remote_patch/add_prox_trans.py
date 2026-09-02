#!/usr/bin/env python3
# 给 5 个 proximal 弯曲关节添加 transmission(否则 gazebo_ros_control 无法驱动)
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
add = """
  <!-- 手指弯曲关节(proximal)传动 -->
  <transmission name="if_proximal_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="if_proximal_link_j"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
    <actuator name="if_proximal_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="mf_proximal_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="mf_proximal_link_j"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
    <actuator name="mf_proximal_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="rf_proximal_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="rf_proximal_link_j"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
    <actuator name="rf_proximal_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="lf_proximal_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="lf_proximal_link_j"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
    <actuator name="lf_proximal_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="th_proximal_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="th_proximal_link_j"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
    <actuator name="th_proximal_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
"""
txt = txt.replace("</robot>", add + "</robot>")
open(path, "w").write(txt)
print("proximal transmissions added:", txt.count("proximal_trans"))
