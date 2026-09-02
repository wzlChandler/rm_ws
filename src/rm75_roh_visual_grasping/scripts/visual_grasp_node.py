#!/usr/bin/env python3

import copy
import math
import os
import sys
import threading
import time

import moveit_commander
import rospkg
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped TF conversions
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rm_msgs.msg import Read_TCPandRTU, Register_Data, Set_Modbus_Mode
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool, String, UInt16MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

from rm75_roh_visual_grasping.core import (
    StableTargetFilter,
    cartesian_fraction_ok,
    decode_register_values,
    hand_joint_positions,
    hand_state_allows_lift,
    link_position_for_tcp,
    point_before_target_along_tool_z,
    point_in_workspace,
    poses_are_close,
    surface_point_to_sphere_center,
)

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path("rohand_urdf_ros1"), "scripts"))
from FingerMathURDF import HAND_FingerPosToAngle


class PlanningError(RuntimeError):
    pass


class VisualGraspNode:
    def __init__(self):
        rospy.init_node("visual_grasp")

        self.move_group_name = rospy.get_param("~move_group", "arm")
        self.ee_link = rospy.get_param("~ee_link", "Link7")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.camera_target_topic = rospy.get_param("~camera_target_topic", "/yolo/target_point_camera")
        self.execute_enabled = bool(rospy.get_param("~execute", False))
        self.execution_stage = rospy.get_param("~execution_stage", "full")
        self.prompt_enabled = bool(rospy.get_param("~prompt_enabled", True))
        self.robot_model_name = rospy.get_param("~robot_model_name", "rm75_roh_a001_right")
        if self.execution_stage not in ("approach", "full"):
            raise ValueError("execution_stage must be 'approach' or 'full'")

        self.ball_radius = float(rospy.get_param("~ball_radius", 0.020))
        self.workspace = (
            tuple(rospy.get_param("~workspace_x", [0.12, 0.42])),
            tuple(rospy.get_param("~workspace_y", [-0.18, 0.18])),
            tuple(rospy.get_param("~workspace_z", [-0.005, 0.10])),
        )
        self.tcp_offset = tuple(rospy.get_param("~tcp_xyz", [0.034, 0.023, 0.106]))
        self.down_rpy = tuple(rospy.get_param("~down_rpy", [-3.141592653589793, 0.0, 0.0]))
        self.grasp_tilt_degrees = list(rospy.get_param("~grasp_tilt_degrees", [25.0, 15.0, 30.0]))
        self.grasp_yaw_degrees = list(
            rospy.get_param("~grasp_yaw_degrees", [0.0, 30.0, -30.0, 60.0, -60.0, 90.0])
        )
        self.pregrasp_height = float(rospy.get_param("~pregrasp_height", 0.15))
        self.lift_height = float(rospy.get_param("~lift_height", 0.15))
        self.observation_xyz = tuple(
            rospy.get_param("~observation_xyz", [0.231402, 0.000371, 0.442864])
        )
        self.observation_quaternion = tuple(
            rospy.get_param(
                "~observation_quaternion", [-0.000719, 0.999790, 0.000810, 0.020498]
            )
        )
        self.observation_settle_time = float(rospy.get_param("~observation_settle_time", 0.5))
        self.observation_target_timeout = float(
            rospy.get_param("~observation_target_timeout", 8.0)
        )
        if len(self.observation_xyz) != 3 or len(self.observation_quaternion) != 4:
            raise ValueError("observation_xyz and observation_quaternion must contain 3 and 4 values")

        self.table_enabled = bool(rospy.get_param("~table_enabled", True))
        self.table_id = rospy.get_param("~table_id", "visual_grasp_table")
        self.table_top_z = float(rospy.get_param("~table_top_z", -0.017))
        self.table_center_xy = tuple(rospy.get_param("~table_center_xy", [0.30, 0.0]))
        self.table_size = tuple(rospy.get_param("~table_size", [0.80, 0.80, 0.04]))

        self.velocity_scaling = float(rospy.get_param("~velocity_scaling", 0.1))
        self.acceleration_scaling = float(rospy.get_param("~acceleration_scaling", 0.1))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.005))
        self.orientation_tolerance = float(rospy.get_param("~orientation_tolerance", 0.02))
        self.planning_time = float(rospy.get_param("~planning_time", 5.0))
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 10))
        self.eef_step = float(rospy.get_param("~cartesian_eef_step", 0.005))
        self.min_cartesian_fraction = float(rospy.get_param("~cartesian_min_fraction", 0.98))
        self.target_wait_timeout = float(rospy.get_param("~target_wait_timeout", 1.0))

        self.hand_open = list(rospy.get_param("~hand_open", [0, 0, 0, 0, 0, 0]))
        self.hand_preshape = list(rospy.get_param("~hand_preshape", [250, 250, 250, 0, 0, 0]))
        self.hand_close = list(rospy.get_param("~hand_close", [800, 800, 800, 0, 0, 0]))
        self.hand_ack_timeout = float(rospy.get_param("~hand_ack_timeout", 2.0))
        self.hand_motion_timeout = float(rospy.get_param("~hand_motion_timeout", 5.0))
        self.hand_poll_period = float(rospy.get_param("~hand_poll_period", 0.2))
        self.hand_read_retry_period = float(rospy.get_param("~hand_read_retry_period", 0.1))
        self.hand_stable_samples = int(rospy.get_param("~hand_stable_samples", 3))
        self.hand_stable_delta = int(rospy.get_param("~hand_stable_delta", 1000))
        self.hand_open_max_position = int(rospy.get_param("~hand_open_max_position", 5000))
        self.hand_min_close_position = int(rospy.get_param("~hand_min_close_position", 20000))
        self.hand_modbus_port = int(rospy.get_param("~hand_modbus_port", 1))
        self.hand_baudrate = int(rospy.get_param("~hand_baudrate", 115200))
        self.hand_modbus_timeout = int(rospy.get_param("~hand_modbus_timeout", 2))
        self.hand_mode_ack_timeout = float(rospy.get_param("~hand_mode_ack_timeout", 3.0))
        self.hand_device = int(rospy.get_param("~hand_device", 2))
        self.reset_vision_service = rospy.get_param("~reset_vision_service", "/yolo/reset_target")

        self.target_filter = StableTargetFilter(
            int(rospy.get_param("~stable_count", 5)),
            float(rospy.get_param("~stable_threshold", 0.010)),
            float(rospy.get_param("~target_max_age", 0.5)),
        )

        self.lock = threading.Lock()
        self.busy = False
        self.state = None
        self.ack_condition = threading.Condition()
        self.ack_sequence = 0
        self.last_ack = False
        self.read_condition = threading.Condition()
        self.read_sequence = 0
        self.last_read = None
        self.modbus_condition = threading.Condition()
        self.modbus_sequence = 0
        self.modbus_ready = False

        self.status_pub = rospy.Publisher("/visual_grasp/status", String, queue_size=1, latch=True)
        self.display_pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=1, latch=True
        )
        self.hand_pub = rospy.Publisher("/hand/target", UInt16MultiArray, queue_size=1)
        self.read_pub = rospy.Publisher(
            "/rm_driver/Read_Multiple_Holding_Registers", Read_TCPandRTU, queue_size=1
        )
        self.modbus_pub = rospy.Publisher(
            "/rm_driver/Set_Modbus_Mode", Set_Modbus_Mode, queue_size=1
        )
        rospy.Subscriber(
            "/rm_driver/Write_Registers_Result", Bool, self._write_result_callback, queue_size=10
        )
        rospy.Subscriber(
            "/rm_driver/Read_Multiple_Holding_Registers_Result",
            Register_Data,
            self._read_result_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/rm_driver/Set_Modbus_Mode_Result",
            Bool,
            self._modbus_result_callback,
            queue_size=10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.group = moveit_commander.MoveGroupCommander(self.move_group_name)
        self.group.set_pose_reference_frame(self.base_frame)
        self.group.set_end_effector_link(self.ee_link)
        self.group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.group.set_goal_position_tolerance(self.position_tolerance)
        self.group.set_goal_orientation_tolerance(self.orientation_tolerance)
        self.group.set_planning_time(self.planning_time)
        self.group.set_num_planning_attempts(self.planning_attempts)
        self.group.allow_replanning(True)
        if self.table_enabled:
            self._add_table_collision()

        rospy.Subscriber(self.camera_target_topic, PointStamped, self._target_callback, queue_size=10)
        self.preview_service = rospy.Service("/visual_grasp/preview", Trigger, self._preview_callback)
        self.reset_service = rospy.Service("/visual_grasp/reset", Trigger, self._reset_callback)
        self.vision_reset_client = rospy.ServiceProxy(self.reset_vision_service, Trigger)

        self._set_state("WAITING_TARGET")
        rospy.loginfo(
            "visual grasp ready: execute=%s stage=%s tcp=(%.3f, %.3f, %.3f)",
            self.execute_enabled,
            self.execution_stage,
            *self.tcp_offset
        )
        if self.prompt_enabled:
            prompt_thread = threading.Thread(target=self._prompt_loop, name="visual_grasp_prompt")
            prompt_thread.daemon = True
            prompt_thread.start()

    def _set_state(self, state, detail=""):
        with self.lock:
            changed = self.state != state
            self.state = state
        if changed:
            self.status_pub.publish(String(data=state))
        if detail and changed:
            rospy.loginfo("visual grasp state=%s: %s", state, detail)
        elif changed:
            rospy.loginfo("visual grasp state=%s", state)

    def _target_callback(self, message):
        with self.lock:
            allowed_states = (
                ("OBSERVING",)
                if self.execute_enabled
                else ("WAITING_TARGET", "READY")
            )
            if self.state not in allowed_states:
                return
        try:
            camera_center = surface_point_to_sphere_center(
                (message.point.x, message.point.y, message.point.z), self.ball_radius
            )
            corrected = PointStamped()
            corrected.header = message.header
            corrected.point.x, corrected.point.y, corrected.point.z = camera_center
            try:
                base_point = self.tf_buffer.transform(
                    corrected, self.base_frame, timeout=rospy.Duration(0.3)
                )
            except tf2_ros.ExtrapolationException:
                with self.lock:
                    observing = self.execute_enabled and self.state == "OBSERVING"
                if not observing:
                    raise
                corrected.header.stamp = rospy.Time(0)
                base_point = self.tf_buffer.transform(
                    corrected, self.base_frame, timeout=rospy.Duration(0.3)
                )
                rospy.logwarn_throttle(
                    2.0,
                    "image-time TF unavailable; using latest TF while stationary at observation pose",
                )
            point = (base_point.point.x, base_point.point.y, base_point.point.z)
            if not point_in_workspace(point, self.workspace):
                rospy.logwarn_throttle(1.0, "sphere center outside configured workspace: %s", point)
                return
            with self.lock:
                allowed_states = (
                    ("OBSERVING",)
                    if self.execute_enabled
                    else ("WAITING_TARGET", "READY")
                )
                if self.state not in allowed_states:
                    return
                if not self.target_filter.add(message.header.stamp.to_sec(), point):
                    return
                target = self.target_filter.stable_target(rospy.Time.now().to_sec())
            if target is not None:
                rospy.loginfo_throttle(
                    1.0, "stable sphere center=(%.3f, %.3f, %.3f)" % target
                )
                with self.lock:
                    observing = self.state == "OBSERVING"
                if not observing:
                    self._set_state("READY")
        except (
            ValueError,
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn_throttle(1.0, "target conversion failed: %s", error)

    def _write_result_callback(self, message):
        with self.ack_condition:
            self.ack_sequence += 1
            self.last_ack = bool(message.data)
            self.ack_condition.notify_all()

    def _read_result_callback(self, message):
        rospy.loginfo("hand register response: state=%s data=%s", message.state, list(message.data))
        with self.read_condition:
            self.read_sequence += 1
            self.last_read = message
            self.read_condition.notify_all()

    def _modbus_result_callback(self, message):
        with self.modbus_condition:
            self.modbus_sequence += 1
            self.modbus_ready = bool(message.data)
            self.modbus_condition.notify_all()

    def _take_target(self, timeout=None, waiting_state="WAITING_TARGET"):
        wait_timeout = self.target_wait_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait_timeout
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with self.lock:
                target = self.target_filter.stable_target(now)
            if target is not None:
                return target
            self._set_state(waiting_state, "waiting for a fresh stable target")
            if time.monotonic() >= deadline:
                break
            rospy.sleep(0.02)
        rospy.logwarn(
            "start rejected: no fresh stable target after %.2fs; "
            "wait for /visual_grasp/status=READY",
            wait_timeout,
        )
        return None

    def _begin_request(self):
        with self.lock:
            if self.busy:
                return False
            self.busy = True
            return True

    def _end_request(self):
        with self.lock:
            self.busy = False

    def _preview_callback(self, _request):
        return self._handle_cycle(dry_run=True, stage="full")

    def _prompt_loop(self):
        try:
            terminal_input = open("/dev/tty", "r")
            terminal_output = open("/dev/tty", "w", buffering=1)
        except OSError as error:
            rospy.logerr("cannot open /dev/tty for grasp confirmation: %s", error)
            return
        try:
            while not rospy.is_shutdown():
                action = "execute one grasp" if self.execute_enabled else "preview one grasp"
                terminal_output.write("\n[visual_grasp] Enter y to %s: " % action)
                answer = terminal_input.readline()
                if not answer:
                    return
                if answer.strip().lower() != "y":
                    terminal_output.write("[visual_grasp] Not started.\n")
                    continue
                response = self._handle_cycle(
                    dry_run=not self.execute_enabled,
                    stage=self.execution_stage if self.execute_enabled else "full",
                )
                terminal_output.write(
                    "[visual_grasp] %s: %s\n"
                    % ("SUCCESS" if response.success else "FAILED", response.message)
                )
        finally:
            terminal_input.close()
            terminal_output.close()

    def _handle_cycle(self, dry_run, stage):
        if not self._begin_request():
            return TriggerResponse(success=False, message="visual grasp is busy")
        try:
            rospy.loginfo("grasp service requested: dry_run=%s stage=%s", dry_run, stage)
            if dry_run:
                target = self._take_target()
                if target is None:
                    return TriggerResponse(success=False, message="no fresh stable target")
                cycle_start = self.robot.get_current_state()
                observation = self._plan_observation(cycle_start)
                grasp_start = self._state_after(cycle_start, observation) if observation else cycle_start
                self._set_state("PLANNING")
                plan = self._build_plan(target, include_full=(stage == "full"), start_state=grasp_start)
                plan["display_start_state"] = cycle_start
                plan["observation_trajectory"] = observation
                self._publish_display(plan)
                self._set_state("READY", "dry-run plan complete; no commands were sent")
                return TriggerResponse(success=True, message="dry-run plan complete; no motion executed")

            self._set_state("MOVING_TO_OBSERVE")
            if self._at_observation_pose():
                rospy.loginfo("already at observation pose; skipping observation motion")
            else:
                observation = self._plan_observation(self.robot.get_current_state())
                self._execute_trajectory(observation, "observation")

            self._set_state("OPENING")
            if not self._command_hand(self.hand_open, verification="open"):
                raise PlanningError("hand did not open or its status could not be verified")

            self._set_state("OBSERVING")
            if self.observation_settle_time > 0.0:
                rospy.sleep(self.observation_settle_time)
            with self.lock:
                self.target_filter.clear()
            target = self._take_target(
                timeout=self.observation_target_timeout, waiting_state="OBSERVING"
            )
            if target is None:
                raise PlanningError("no fresh stable target at the observation pose")
            with self.lock:
                self.target_filter.clear()

            self._set_state("PLANNING")
            plan = self._build_plan(
                target,
                include_full=(stage == "full"),
                start_state=self.robot.get_current_state(),
            )
            self._publish_display(plan)
            return self._execute_cycle(plan, stage)
        except PlanningError as error:
            self._set_state("FAILED", str(error))
            return TriggerResponse(success=False, message=str(error))
        except Exception as error:
            rospy.logerr("visual grasp failed: %s", error)
            self._set_state("FAILED", str(error))
            return TriggerResponse(success=False, message=str(error))
        finally:
            self._end_request()

    def _reset_callback(self, _request):
        with self.lock:
            if self.busy:
                return TriggerResponse(success=False, message="cannot reset while visual grasp is busy")
            self.target_filter.clear()
        vision_message = ""
        try:
            rospy.wait_for_service(self.reset_vision_service, timeout=1.0)
            response = self.vision_reset_client()
            if not response.success:
                vision_message = "; detector reset failed: " + response.message
        except (rospy.ROSException, rospy.ServiceException) as error:
            vision_message = "; detector reset unavailable: " + str(error)
        self._set_state("WAITING_TARGET")
        return TriggerResponse(success=True, message="target buffer reset" + vision_message)

    def _add_table_collision(self):
        if len(self.table_center_xy) != 2 or len(self.table_size) != 3:
            raise ValueError("table_center_xy and table_size must contain 2 and 3 values")
        table = PoseStamped()
        table.header.frame_id = self.base_frame
        table.pose.orientation.w = 1.0
        table.pose.position.x, table.pose.position.y = self.table_center_xy
        table.pose.position.z = self.table_top_z - self.table_size[2] / 2.0
        self.scene.add_box(self.table_id, table, size=self.table_size)
        deadline = time.monotonic() + 2.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.table_id in self.scene.get_known_object_names():
                rospy.loginfo("planning scene table added: top_z=%.3f", self.table_top_z)
                return
            rospy.sleep(0.05)
        raise RuntimeError("table collision object was not accepted by MoveIt")

    def _pose_for_tcp(self, tcp_position, quaternion):
        link_position = link_position_for_tcp(tcp_position, quaternion, self.tcp_offset)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = link_position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        return pose

    def _target_poses(self, target, quaternion):
        grasp = self._pose_for_tcp(target, quaternion)
        pregrasp_tcp = point_before_target_along_tool_z(
            target, quaternion, self.pregrasp_height
        )
        lift_tcp = point_before_target_along_tool_z(target, quaternion, self.lift_height)
        pregrasp = self._pose_for_tcp(pregrasp_tcp, quaternion)
        lift = self._pose_for_tcp(lift_tcp, quaternion)
        return pregrasp, grasp, lift

    def _orientation_candidates(self):
        for tilt_degrees in self.grasp_tilt_degrees:
            for yaw_degrees in self.grasp_yaw_degrees:
                rpy = (
                    self.down_rpy[0] + math.radians(tilt_degrees),
                    self.down_rpy[1],
                    self.down_rpy[2] + math.radians(yaw_degrees),
                )
                yield tilt_degrees, yaw_degrees, tuple(quaternion_from_euler(*rpy))

    @staticmethod
    def _trajectory_has_points(trajectory):
        return bool(trajectory and trajectory.joint_trajectory.points)

    def _plan_pose(self, pose, start_state, description="pregrasp"):
        self.group.set_start_state(start_state)
        self.group.set_pose_target(pose, self.ee_link)
        raw_plan = self.group.plan()
        self.group.clear_pose_targets()
        if isinstance(raw_plan, tuple):
            success, trajectory = bool(raw_plan[0]), raw_plan[1]
        else:
            trajectory = raw_plan
            success = self._trajectory_has_points(trajectory)
        if not success or not self._trajectory_has_points(trajectory):
            raise PlanningError("OMPL could not plan the %s pose" % description)
        return trajectory

    def _plan_observation(self, start_state):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.observation_xyz
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.observation_quaternion
        return self._plan_pose(pose, start_state, description="observation")

    def _at_observation_pose(self):
        current = self.group.get_current_pose(self.ee_link).pose
        return poses_are_close(
            (current.position.x, current.position.y, current.position.z),
            (current.orientation.x, current.orientation.y, current.orientation.z, current.orientation.w),
            self.observation_xyz,
            self.observation_quaternion,
            self.position_tolerance,
            self.orientation_tolerance,
        )

    def _plan_cartesian(self, pose, start_state, description):
        self.group.set_start_state(start_state)
        trajectory, fraction = self.group.compute_cartesian_path(
            [pose], self.eef_step, avoid_collisions=True
        )
        if not cartesian_fraction_ok(fraction, self.min_cartesian_fraction) or not self._trajectory_has_points(trajectory):
            raise PlanningError(
                "%s Cartesian fraction %.3f is below %.3f"
                % (description, fraction, self.min_cartesian_fraction)
            )
        try:
            trajectory = self.group.retime_trajectory(
                start_state,
                trajectory,
                self.velocity_scaling,
                self.acceleration_scaling,
                algorithm="iterative_time_parameterization",
            )
        except TypeError:
            trajectory = self.group.retime_trajectory(
                start_state, trajectory, self.velocity_scaling, self.acceleration_scaling
            )
        if not self._trajectory_has_points(trajectory):
            raise PlanningError(description + " trajectory timing failed")
        return trajectory

    @staticmethod
    def _state_after(start_state, trajectory):
        state = copy.deepcopy(start_state)
        final_point = trajectory.joint_trajectory.points[-1]
        positions = dict(zip(trajectory.joint_trajectory.joint_names, final_point.positions))
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        current.update(positions)
        state.joint_state = JointState()
        state.joint_state.name = list(current.keys())
        state.joint_state.position = [current[name] for name in state.joint_state.name]
        return state

    def _build_plan_for_orientation(self, target, include_full, tilt, yaw, quaternion, start_state):
        pregrasp_pose, grasp_pose, lift_pose = self._target_poses(target, quaternion)
        pregrasp_plan = self._plan_pose(pregrasp_pose, start_state)
        result = {
            "start_state": start_state,
            "pregrasp_pose": pregrasp_pose,
            "grasp_pose": grasp_pose,
            "lift_pose": lift_pose,
            "quaternion": quaternion,
            "tilt_degrees": tilt,
            "yaw_degrees": yaw,
            "trajectories": [pregrasp_plan],
        }
        if include_full:
            pregrasp_state = self._state_after(start_state, pregrasp_plan)
            descent_plan = self._plan_cartesian(grasp_pose, pregrasp_state, "descent")
            grasp_state = self._state_after(pregrasp_state, descent_plan)
            lift_plan = self._plan_cartesian(lift_pose, grasp_state, "lift")
            result["trajectories"].extend((descent_plan, lift_plan))
            lift_state = self._state_after(grasp_state, lift_plan)
            result["return_trajectory"] = self._plan_observation(lift_state)
        return result

    def _build_plan(self, target, include_full, start_state):
        failures = []
        for tilt, yaw, quaternion in self._orientation_candidates():
            try:
                plan = self._build_plan_for_orientation(
                    target, include_full, tilt, yaw, quaternion, start_state
                )
                rospy.loginfo("selected grasp orientation: tilt=%.1f yaw=%.1f", tilt, yaw)
                return plan
            except PlanningError as error:
                failures.append("tilt=%.1f yaw=%.1f: %s" % (tilt, yaw, error))
        raise PlanningError("no grasp orientation produced a safe path; " + "; ".join(failures))

    def _publish_display(self, plan):
        display = DisplayTrajectory()
        display.model_id = self.robot_model_name
        display.trajectory_start = plan.get("display_start_state", plan["start_state"])
        arm_trajectories = plan["trajectories"]
        observation = plan.get("observation_trajectory")
        if observation:
            display.trajectory.append(observation)
        display.trajectory.append(self._hand_display_trajectory(self.hand_open))
        display.trajectory.append(arm_trajectories[0])
        if len(arm_trajectories) == 3:
            display.trajectory.append(self._hand_display_trajectory(self.hand_preshape))
            display.trajectory.append(arm_trajectories[1])
            display.trajectory.append(self._hand_display_trajectory(self.hand_close))
            display.trajectory.append(arm_trajectories[2])
            return_trajectory = plan.get("return_trajectory")
            if return_trajectory:
                display.trajectory.append(return_trajectory)
        self.display_pub.publish(display)

    @staticmethod
    def _hand_display_trajectory(actuator_values):
        names, positions = hand_joint_positions(actuator_values, HAND_FingerPosToAngle)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.5)
        trajectory.joint_trajectory.points.append(point)
        return trajectory

    def _execute_trajectory(self, trajectory, description):
        if not self.group.execute(trajectory, wait=True):
            self.group.stop()
            raise PlanningError(description + " execution failed")
        self.group.stop()

    def _execute_cycle(self, plan, stage):
        self._set_state("APPROACHING")
        self._execute_trajectory(plan["trajectories"][0], "pregrasp")
        if stage == "approach":
            self._set_state("APPROACHED")
            return TriggerResponse(success=True, message="pregrasp pose reached; full grasp was not executed")

        self._set_state("PRESHAPING")
        if not self._command_hand(self.hand_preshape, verification="stable"):
            raise PlanningError("hand preshape command or register verification failed")

        grasp_pose = plan["grasp_pose"]
        lift_pose = plan["lift_pose"]

        self._set_state("DESCENDING")
        current_state = self.robot.get_current_state()
        descent = self._plan_cartesian(grasp_pose, current_state, "descent")
        self._execute_trajectory(descent, "descent")

        self._set_state("CLOSING")
        if not self._command_hand(self.hand_close, verification="close"):
            raise PlanningError("hand close command or register verification failed; lift inhibited")

        self._set_state("LIFTING")
        current_state = self.robot.get_current_state()
        lift = self._plan_cartesian(lift_pose, current_state, "lift")
        self._execute_trajectory(lift, "lift")
        self._set_state("LIFTED", "hand remains closed; object presence is not independently sensed")

        self._set_state("RETURNING")
        return_trajectory = self._plan_observation(self.robot.get_current_state())
        if return_trajectory:
            self._execute_trajectory(return_trajectory, "return to observation")
        self._set_state("RETURNED", "grasp complete; hand remains closed at the observation pose")
        return TriggerResponse(
            success=True, message="grasp sequence complete; returned to observation with hand closed"
        )

    def _wait_for_ack(self, previous_sequence):
        deadline = time.monotonic() + self.hand_ack_timeout
        with self.ack_condition:
            while not rospy.is_shutdown() and self.ack_sequence <= previous_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.ack_condition.wait(remaining)
            return self.ack_sequence > previous_sequence and self.last_ack

    def _read_registers(self, address, count, timeout=1.0):
        deadline = time.monotonic() + timeout
        request = Read_TCPandRTU()
        request.type = 1
        request.port = self.hand_modbus_port
        request.address = address
        request.num = count
        request.device = self.hand_device

        while not rospy.is_shutdown():
            with self.read_condition:
                previous_sequence = self.read_sequence
            self.read_pub.publish(request)
            with self.read_condition:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                while self.read_sequence <= previous_sequence and remaining > 0.0:
                    self.read_condition.wait(remaining)
                    remaining = deadline - time.monotonic()
                if self.read_sequence <= previous_sequence:
                    return None
                previous_sequence = self.read_sequence
                message = self.last_read
                if message is not None and message.state:
                    try:
                        return decode_register_values(message.data, count)
                    except ValueError as error:
                        rospy.logwarn("invalid hand register response: %s", error)
                elif message is not None:
                    rospy.logwarn_throttle(1.0, "hand register read busy or rejected; retrying")
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            rospy.sleep(min(self.hand_read_retry_period, remaining))

    def _ensure_hand_modbus(self):
        with self.modbus_condition:
            if self.modbus_ready:
                return True
            previous_sequence = self.modbus_sequence
        if self.modbus_pub.get_num_connections() == 0:
            rospy.logerr("/rm_driver/Set_Modbus_Mode has no subscribers")
            return False

        request = Set_Modbus_Mode()
        request.port = self.hand_modbus_port
        request.baudrate = self.hand_baudrate
        request.timeout = self.hand_modbus_timeout
        self.modbus_pub.publish(request)

        deadline = time.monotonic() + self.hand_mode_ack_timeout
        with self.modbus_condition:
            while not rospy.is_shutdown() and self.modbus_sequence <= previous_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    rospy.logerr("tool Modbus initialization timed out")
                    return False
                self.modbus_condition.wait(remaining)
            if not self.modbus_ready:
                rospy.logerr("tool Modbus initialization was rejected")
                return False
        rospy.loginfo("tool Modbus mode confirmed before hand command")
        return True

    def _verify_hand(self, verification):
        error = self._read_registers(1006, 1)
        if error is None or error[0] != 0:
            rospy.logerr("hand error register invalid: %s", error)
            return False

        deadline = time.monotonic() + self.hand_motion_timeout
        stable_count = 0
        previous = None
        latest = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            latest = self._read_registers(1145, 6)
            if latest is None:
                rospy.sleep(self.hand_poll_period)
                continue
            active = latest[:3]
            if previous is not None and max(abs(a - b) for a, b in zip(active, previous)) <= self.hand_stable_delta:
                stable_count += 1
            else:
                stable_count = 1
            previous = active
            if stable_count >= self.hand_stable_samples:
                if verification == "close":
                    if hand_state_allows_lift(True, error[0], latest, self.hand_min_close_position):
                        return True
                elif verification == "open" and all(
                    position <= self.hand_open_max_position for position in active
                ):
                    return True
                elif verification == "stable":
                    return True
            rospy.sleep(self.hand_poll_period)
        rospy.logerr("hand positions did not settle before timeout; latest=%s", latest)
        return False

    def _command_hand(self, values, verification):
        if not self._ensure_hand_modbus():
            return False
        if self.hand_pub.get_num_connections() == 0:
            rospy.logerr("/hand/target has no subscribers")
            return False
        with self.ack_condition:
            previous_sequence = self.ack_sequence
        message = UInt16MultiArray(data=values)
        self.hand_pub.publish(message)
        if not self._wait_for_ack(previous_sequence):
            rospy.logerr("hand register write was rejected or timed out")
            return False
        return self._verify_hand(verification)


def main():
    VisualGraspNode()
    rospy.spin()


if __name__ == "__main__":
    main()
