#!/usr/bin/env python3
import os
import sys
import time
import math
import re
from enum import Enum, auto
from dataclasses import dataclass

pymycobot_path = os.environ.get("PYMYCOBOT_PATH", os.path.expanduser("~/arm_ws/src/pymycobot"))
if os.path.isdir(pymycobot_path):
    sys.path.insert(0, pymycobot_path)

from pymycobot import MyCobot280  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from moveit_msgs.srv import GetMotionPlan  # noqa: E402
from moveit_msgs.msg import (  # noqa: E402
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive  # noqa: E402
from geometry_msgs.msg import PoseStamped, Quaternion  # noqa: E402

from control_msgs.action import FollowJointTrajectory, GripperCommand  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402


def quat_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def moveit_error_to_str(val: int) -> str:
    m = {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.FAILURE: "FAILURE",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
        MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
        MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    }
    return m.get(val, f"UNKNOWN({val})")


def _extract_joint_index(name: str):
    nums = re.findall(r"\d+", name)
    if not nums:
        return None


    # 对于 link1_to_link2 这类命名，第一段数字才是关节序号。
    # 取最后一段会把 link1_to_link2 误判成 joint2，导致真机关节映射错位。
    return int(nums[0])

class State(Enum):
    IDLE = auto()
    PLAN_EXEC = auto()
    WAIT_PRE = auto()
    HOMING = auto()


@dataclass
class Task:
    mode: str
    x: float
    y: float
    z: float
    retries_left: int = 1  # 抓取失败：从 home 重试一次（默认 1 次）


class ArmWorker(Node):
    def __init__(self):
        super().__init__("arm_worker_node")
        # ---------------- HW ----------------
        self.port = "/dev/ttyAMA0"
        self.baud = 1000000
        self.mc = None

        # ---------------- MoveIt (保持第一版命名) ----------------
        self.group_name = "arm"
        self.ee_link = "tcp"
        self.base_frame = "base_link"  # REP-103: x前 y左 z上
        self.allowed_planning_time = 8.0
        self.num_planning_attempts = 20
        self.max_vel_scale = 0.15
        self.max_acc_scale = 0.15

        # OMPL vs LIN position box
        # Position constraint box
        self.pos_box_m = 0.03
        self.pos_box_m_lin = 0.001
        # pick 任务为了避免横向漂移，收紧 pregrasp 与下抓目标容差
        self.pick_pregrasp_box_m = 0.001
        self.pick_down_box_m = 0.0002

        # ---------------- 末端约束：tcp +Y 对齐 base -Z（保持第一版） ----------------
        # 选择 R = RotX(-90deg)：
        #   tcp +Y -> base -Z
        #   tcp +X -> base +X
        #   tcp +Z -> base +Y
        self.ee_roll_deg = -90.0
        self.ee_pitch_deg = 0.0
        self.ee_yaw_deg = 0.0

        # 允许绕“竖直轴”(tcp 的 Y) 自由转动：放开 about Y
        self.tilt_tol_rad = 0.15
        self.free_about_y_rad = math.pi
        self.ori_weight = 1.0

        # ---------------- Pilz LIN（保证垂直直线下抓） ----------------
        self.pilz_pipeline_id = "pilz_industrial_motion_planner"
        self.pilz_lin_planner_id = "LIN"

        # ---------------- Behavior ----------------
        self.pregrasp_offset_mm = 30.0
        self.pregrasp_wait_sec = 3.0

        # ---------------- Home ----------------
        self.home_angles_deg = [0, 0, 0, 0, 0, 0]
        self.home_speed = 25
        self.home_timeout_sec = 10.0
        self.home_open_gripper = True
        self.sim_home_time_sec = 2.0

        # ---------------- Smooth HW exec ----------------
        self.hw_rate_hz = 190.0
        self.hw_time_scale = 0.2
        self.hw_send_speed = 12

        # ---------------- Gripper ----------------
        self.gripper_cmd_speed = 50
        self.gripper_open_value = 100
        self.gripper_close_value = 5
        self.gripper_verify_tol = 20
        self.gripper_retries = 3

        # 仿真 GripperActionController 的关节位置（需落在 URDF 关节限位内）
        self.sim_gripper_open_pos = 0.15
        self.sim_gripper_close_pos = -0.65

        # 抓取即时判定 + 重试节奏
        self.grasp_check_delay_sec = 0.35
        self.grasp_success_min_value = 35
        self.grasp_retry_pause_sec = 0.4

        # ---------------- 掉落检测（保留第二版逻辑） ----------------
        self.gripper_monitor_period = 0.5
        self.gripper_drop_threshold = 30
        self._gripper_monitor_timer = None

        # ---------------- ROS ----------------
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.exec_ac = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper_ac = ActionClient(self, GripperCommand, "/gripper_action_controller/gripper_cmd")
        self.gripper_traj_ac = ActionClient(self, FollowJointTrajectory, "/gripper_action_controller/follow_joint_trajectory")
        self.gripper_traj_ac_legacy = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.sub = self.create_subscription(String, "arm_command", self.command_callback, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        # joint mapping (MoveIt -> HW)
        self.moveit_joint_names = None
        self._moveit_name_to_hw_idx = None
        self._last_joint_rad_by_moveit = None

        # sim joint names（保持第一版）
        self.sim_joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]

        # ---------------- FSM ----------------
        self.state = State.IDLE
        self.has_object = False
        self._queue = []
        self._task: Task | None = None
        self._steps = []
        self._step_idx = 0
        self._token = 0
        self._wait_timer = None

        # ---------------- Connect HW ----------------
        try:
            self.mc = MyCobot280(self.port, self.baud)
            self.mc.power_on()
            time.sleep(0.5)
            self.get_logger().info(f"机械臂连接成功: {self.port} -> 真机模式")
            self._init_gripper()
        except Exception as e:
            self.mc = None
            self.get_logger().warn(f"未连接真机（{e}）-> 仿真模式")

        if self.mc:
            self.timer_hw = self.create_timer(0.05, self._poll_hw_and_publish_joint_states)
            self._gripper_monitor_timer = self.create_timer(self.gripper_monitor_period, self._monitor_gripper)

        self.get_logger().info("Waiting for /plan_kinematic_path ...")
        ok = self.plan_cli.wait_for_service(timeout_sec=30.0)
        if not ok:
            self.get_logger().error("❌ /plan_kinematic_path not available. Is move_group running?")


        if not self.mc:
            arm_ready = self.exec_ac.wait_for_server(timeout_sec=10.0)
            grip_cmd_ready = self.gripper_ac.wait_for_server(timeout_sec=2.0)

            # 兼容旧安装包/历史版本：若属性不存在则懒创建，避免直接崩溃。
            gripper_traj_ac = getattr(self, "gripper_traj_ac", None)
            if gripper_traj_ac is None:
                self.get_logger().warn("gripper_traj_ac missing, creating fallback trajectory client")
                gripper_traj_ac = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
                self.gripper_traj_ac = gripper_traj_ac
            grip_traj_ready = gripper_traj_ac.wait_for_server(timeout_sec=3.0)
            grip_traj_legacy_ready = self.gripper_traj_ac_legacy.wait_for_server(timeout_sec=1.0)

            if not arm_ready:
                self.get_logger().error("❌ /arm_controller/follow_joint_trajectory not available")
            if not grip_cmd_ready and not grip_traj_ready and not grip_traj_legacy_ready:
                self.get_logger().warn("⚠️ 未发现夹爪 action server（gripper_cmd / gripper_action_controller/follow_joint_trajectory），仿真夹爪将不可见")

        self.timer_tick = self.create_timer(0.02, self._tick)
        self.get_logger().info("ArmWorker ready.")

    # -----------------------------
    # busy 判断（掉落检测用）
    # -----------------------------
    def _is_busy(self) -> bool:
        return self.state in (State.PLAN_EXEC, State.WAIT_PRE, State.HOMING)

    # =========================================================
    # 掉落检测（后台）
    # =========================================================
    def _monitor_gripper(self):
        if (not self.mc) or (not self.has_object):
            return
        if self._is_busy():
            return
        try:
            v = self.mc.get_gripper_value()
            if v is None:
                return
            if v < self.gripper_drop_threshold:
                self.get_logger().warn(f"🚨 警报：疑似掉落！夹爪值={v} < {self.gripper_drop_threshold}")
                self.has_object = False
                self.get_logger().warn("🔄 has_object=False，等待下一条指令（或重新 pick）")
        except Exception as e:
            self.get_logger().error(f"掉落监测读取失败: {e}")

    # =========================================================
    # 夹爪：设置并验证 + 重试
    # =========================================================
    def _set_gripper_and_verify(
        self,
        target_value: int,
        speed: int,
        *,
        expect_open: bool,
        retries: int = 3,
        wait_each: float = 0.9,
        tol: int = 20,
    ) -> bool:
        if not self.mc:
            return True

        for k in range(max(1, int(retries))):
            try:
                try:
                    self.mc.set_gripper_value(int(target_value), int(speed))
                except AttributeError:
                    state = 0 if expect_open else 1
                    self.mc.set_gripper_state(int(state), int(speed))

                time.sleep(float(wait_each))

                try:
                    v = self.mc.get_gripper_value()
                except Exception:
                    v = None

                if v is None:
                    self.get_logger().warn(f"gripper verify: read None (try {k+1}/{retries})")
                    time.sleep(0.2)
                    continue

                if expect_open:
                    if v >= (int(target_value) - int(tol)):
                        return True
                else:
                    if v <= (int(target_value) + int(tol)):
                        return True

                self.get_logger().warn(f"gripper verify failed: v={v} (try {k+1}/{retries})")

            except Exception as e:
                self.get_logger().warn(f"gripper cmd exception (try {k+1}/{retries}): {e}")

            time.sleep(0.2)

        return False

    def _init_gripper(self):
        if not self.mc:
            return
        ok = self._set_gripper_and_verify(
            self.gripper_open_value,
            self.gripper_cmd_speed,
            expect_open=True,
            retries=2,
            wait_each=1.0,
            tol=self.gripper_verify_tol,
        )
        if not ok:
            self.get_logger().warn("⚠️ 初始化夹爪张开未确认成功（建议检查通信/固件）")
        self.has_object = False
    
    def _gripper_open(self) -> bool:       
        if not self.mc:
            return self._gripper_sim(open_gripper=True)

        return self._set_gripper_and_verify(
            self.gripper_open_value,
            self.gripper_cmd_speed,
            expect_open=True,
            retries=self.gripper_retries,
            wait_each=0.9,
            tol=self.gripper_verify_tol,
        )

    def _gripper_close(self) -> bool:
        if not self.mc:
            return self._gripper_sim(open_gripper=False)


        return self._set_gripper_and_verify(
            self.gripper_close_value,
            self.gripper_cmd_speed,
            expect_open=False,
            retries=self.gripper_retries,
            wait_each=1.0,
            tol=self.gripper_verify_tol,
        )

    # =========================================================
    # 抓取即时判定：夹爪闭合后立即读值判断
    #=====================================================
    def _verify_grasp_now(self) -> bool:
        if not self.mc:
            return True  # 仿真：无法读夹爪，避免流程卡住

        time.sleep(float(self.grasp_check_delay_sec))

        try:
            v = self.mc.get_gripper_value()
        except Exception as e:
            self.get_logger().warn(f"抓取判定：读取夹爪失败({e}) -> 按失败处理")
            return False

        if v is None:
            self.get_logger().warn("抓取判定：夹爪值 None -> 按失败处理")
            return False

        if v <= int(self.grasp_success_min_value):
            self.get_logger().warn(f"❌ 抓取失败：夹爪值={v} <= {self.grasp_success_min_value}（疑似空夹）")
            return False

        self.get_logger().info(f"✅ 抓取成功：夹爪值={v} > {self.grasp_success_min_value}")
        return True
    def _gripper_sim(self, *, open_gripper: bool) -> bool:
        target = self.sim_gripper_open_pos if open_gripper else self.sim_gripper_close_pos
        action = "open" if open_gripper else "close"

        if self.gripper_ac.server_is_ready():
            cmd = GripperCommand.Goal()
            cmd.command.max_effort = 100.0
            cmd.command.position = target
            self.get_logger().info(f"SIM gripper {action} (GripperCommand): position={target:.3f}")
            fut = self.gripper_ac.send_goal_async(cmd)
            fut.add_done_callback(lambda f: self._on_sim_gripper_goal_sent(f, open_gripper))
            return True

        traj_clients = [
            getattr(self, "gripper_traj_ac", None),
            getattr(self, "gripper_traj_ac_legacy", None),
        ]
        for traj_ac in traj_clients:
            if traj_ac is None or (not traj_ac.server_is_ready()):
                continue

            traj = JointTrajectory()
            traj.joint_names = ["gripper_controller"]
            pt = JointTrajectoryPoint()
            pt.positions = [float(target)]
            pt.time_from_start.sec = 0
            pt.time_from_start.nanosec = int(0.5 * 1e9)
            traj.points = [pt]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            self.get_logger().info(f"SIM gripper {action} (JointTrajectory): position={target:.3f}")
            traj_ac.send_goal_async(goal)
            return True

        self.get_logger().warn("SIM gripper command skipped: no available gripper action server")
        return False

    def _on_sim_gripper_goal_sent(self, fut, open_gripper: bool):
        try:
            gh = fut.result()
        except Exception as e:
            self.get_logger().error(f"SIM gripper send failed: {e}")
            return

        if gh is None or not gh.accepted:
            action = "open" if open_gripper else "close"
            self.get_logger().warn(f"SIM gripper goal rejected ({action})")
            return

        gh.get_result_async().add_done_callback(self._on_sim_gripper_done)

    def _on_sim_gripper_done(self, fut):
        try:
            result = fut.result().result
        except Exception as e:
            self.get_logger().warn(f"SIM gripper result error: {e}")
            return

        if hasattr(result, "stalled") and result.stalled:
            self.get_logger().info("SIM gripper reached stall condition")

    # =========================================================
    # HW -> joint_states
    # =========================================================
    def _poll_hw_and_publish_joint_states(self):
        try:
            deg = self.mc.get_angles()
            if not deg or len(deg) < 6:
                return
        except Exception:
            return

        rad_hw = [math.radians(x) for x in deg[:6]]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        if self.moveit_joint_names is None:
            js.name = [f"joint{i}" for i in range(1, 7)]
            js.position = rad_hw
            self.js_pub.publish(js)
            return

        js.name = self.moveit_joint_names
        pos = [0.0] * 6
        for mi, hw_i in enumerate(self._moveit_name_to_hw_idx):
            pos[mi] = rad_hw[hw_i]
        js.position = pos
        self.js_pub.publish(js)
        self._last_joint_rad_by_moveit = pos

    def _learn_moveit_joint_mapping(self, traj_joint_names):
        names = list(traj_joint_names[:6])
        indices = []
        for n in names:
            idx = _extract_joint_index(n)
            if idx is None or idx < 1 or idx > 6:
                return False
            indices.append(idx - 1)
        self.moveit_joint_names = names
        self._moveit_name_to_hw_idx = indices
        self.get_logger().info(f"✅ MoveIt joint_names: {self.moveit_joint_names}")
        self.get_logger().info(f"✅ MoveIt->HW map: {self._moveit_name_to_hw_idx}")
        return True

    # =========================================================
    # Command
    # =========================================================
    def command_callback(self, msg: String):
        s = msg.data.strip()
        cmd = s.lower()
        self.get_logger().info(f"收到指令: {s}")

        if cmd == "home":
            self._queue.clear()
            self._queue.append(("home",))
            self._enter_home()
            return

        parts = cmd.split()
        if len(parts) != 4:
            self.get_logger().error("格式: pick x y z (mm) | place x y z (mm) | home")
            return

        mode = parts[0]
        if mode not in ("pick", "place"):
            self.get_logger().error("模式必须是 pick 或 place")
            return

        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            self.get_logger().error("x y z 必须是数字(mm)")
            return

        # 每条新指令：默认允许 1 次抓取失败重试（只对 pick 有意义）
        self._queue.append(Task(mode, x, y, z, retries_left=1))

    # =========================================================
    # Tick / FSM
    # =========================================================
    def _tick(self):
        if self.state != State.IDLE:
            return
        if not self._queue:
            return

        item = self._queue.pop(0)
        if isinstance(item, tuple) and item[0] == "home":
            self._enter_home()
            return

        assert isinstance(item, Task)
        t = item

        if t.mode == "pick" and self.has_object:
            self.get_logger().warn("手里已有物体，忽略 pick")
            return
        if t.mode == "place" and (not self.has_object):
            self.get_logger().warn("手里没东西，忽略 place")
            return

        pre_z = t.z + self.pregrasp_offset_mm
        self._task = t
        self._steps = [(t.x, t.y, pre_z), (t.x, t.y, t.z)]
        self._step_idx = 0
        self._start_step()

    def _start_step(self):
        if self._task is None:
            self.state = State.IDLE
            return

        if self._step_idx >= len(self._steps):
            self._enter_home()
            return

        self.state = State.PLAN_EXEC
        self._token += 1
        token = self._token

        x, y, z = self._steps[self._step_idx]
        self.get_logger().info(f"Planning step[{self._step_idx}] -> ({x:.1f},{y:.1f},{z:.1f}) mm")

        # ✅ 保证直线下抓：pick 的 step1 使用 Pilz LIN（保持第一版）
        pipeline_id = None
        planner_id = None
        goal_box_m = None
        if self._task.mode == "pick":
            # step0 先把 x/y 收紧到目标点正上方，减少 step1 横向修正
            if self._step_idx == 0:
                goal_box_m = self.pick_pregrasp_box_m
            # step1 使用 Pilz LIN 下抓，并进一步收紧目标容差，尽量保证纯竖直下移
            elif self._step_idx == 1:
                pipeline_id = self.pilz_pipeline_id
                planner_id = self.pilz_lin_planner_id
                goal_box_m = self.pick_down_box_m

        req = self._build_plan_request_with_start_state(
            x, y, z,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            goal_box_m=goal_box_m,
        )
        self.plan_cli.call_async(req).add_done_callback(lambda f: self._on_plan_done(f, token))

    def _on_plan_done(self, fut, token: int):
        if token != self._token:
            return
        try:
            result = fut.result()
        except Exception as e:
            self.get_logger().error(f"planning exception: {e}")
            self._reset()
            return

        res = result.motion_plan_response
        code = res.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"planning failed: {moveit_error_to_str(code)} ({code})")
            self._reset()
            return

        traj = res.trajectory.joint_trajectory
        if (not traj.joint_names) or (len(traj.points) == 0):
            self.get_logger().error("planning returned empty trajectory")
            self._reset()
            return

        if self.mc and self.moveit_joint_names is None:
            if not self._learn_moveit_joint_mapping(traj.joint_names):
                self.get_logger().error(f"无法解析 joint_names: {traj.joint_names}")
                self._reset()
                return

        if self.mc:
            ok = self._exec_hw_smooth_interpolated(traj)
            if not ok:
                self._reset()
                return
            self._on_step_finished()
        else:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            self.exec_ac.send_goal_async(goal).add_done_callback(lambda f: self._on_sim_goal_sent(f, token))

    def _on_sim_goal_sent(self, fut, token: int):
        if token != self._token:
            return
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("controller rejected goal")
            self._reset()
            return
        gh.get_result_async().add_done_callback(lambda f: self._on_sim_done(f, token))

    def _on_sim_done(self, fut, token: int):
        if token != self._token:
            return
        try:
            result = fut.result().result
            if hasattr(result, "error_code") and int(result.error_code) != 0:
                self.get_logger().error(f"sim controller error_code={int(result.error_code)}")
                self._reset()
                return
        except Exception as e:
            self.get_logger().error(f"sim result exception: {e}")
            self._reset()
            return
        self._on_step_finished()

    def _on_step_finished(self):
        idx = self._step_idx
        self.get_logger().info(f"✅ step[{idx}] finished")

        # pick：到预抓取点后等待，再下抓
        if self._task and self._task.mode == "pick" and idx == 0:
            self.state = State.WAIT_PRE
            self._token += 1
            token = self._token
            self._cancel_wait_timer()
            self.get_logger().info(f"到达预抓取，停顿 {self.pregrasp_wait_sec:.1f}s 后下抓")
            self._wait_timer = self.create_timer(self.pregrasp_wait_sec, lambda: self._on_wait_done(token))
            return

        # step1 到目标点：pick / place 的末端动作
        if self._task and idx == 1:
            if self._task.mode == "pick":
                self.get_logger().info("夹爪闭合：抓取")
                self._gripper_close()

                ok = self._verify_grasp_now()
                self.has_object = bool(ok)

                if ok:
                    self.get_logger().info("✅ 抓取成功，回 Home")
                    self._enter_home()
                    return

                # 失败：从 home 重试一次（保留第二版 E）
                self.get_logger().warn("🔁 抓取失败：将从 Home 重新尝试同一抓取点（1 次）")

                # 失败时务必开爪（保证下一轮从 home 开始夹爪张开）
                self._gripper_open()
                self.has_object = False

                if self._task.retries_left > 0:
                    retry_task = Task(
                        mode="pick",
                        x=self._task.x,
                        y=self._task.y,
                        z=self._task.z,
                        retries_left=self._task.retries_left - 1,
                    )

                    time.sleep(float(self.grasp_retry_pause_sec))
                    self._enter_home()
                    time.sleep(float(self.grasp_retry_pause_sec))

                    # 双保险：回 home 后再开一次
                    try:
                        self._gripper_open()
                    except Exception:
                        pass

                    # 插队重试：从 home 开始再次跑 step0/step1（同一点）
                    self._queue.insert(0, retry_task)
                    self.get_logger().warn("🟡 已重新排队：同一点 pick 重试（从 Home 开始）")
                    return

                self.get_logger().warn("❌ 抓取失败且无重试次数，回 Home 等待下一条指令")
                self._enter_home()
                return

            else:
                self.get_logger().info("夹爪张开：放置")
                ok = self._gripper_open()
                if self.mc:
                    try:
                        v = self.mc.get_gripper_value()
                        self.get_logger().info(f"place: gripper value after open={v} (ok={ok})")
                    except Exception:
                        self.get_logger().info(f"place: gripper verify read failed (ok={ok})")
                self.has_object = False
                self._enter_home()
                return

        # 其它情况：推进下一 step
        self._step_idx += 1
        self.state = State.IDLE
        self._start_step()

    def _on_wait_done(self, token: int):
        self._cancel_wait_timer()
        if token != self._token:
            return
        if self.state != State.WAIT_PRE:
            return
        self.get_logger().info("⏱️ 等待结束，开始 step1 下抓（Pilz LIN）")
        self.state = State.IDLE
        self._step_idx = 1
        self._start_step()

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            try:
                self._wait_timer.cancel()
            except Exception:
                pass
            self._wait_timer = None

    # =========================================================
    # HOME（真机/仿真都实现，避免第二版“仿真 home 不动”）
    # =========================================================
    def _enter_home(self):
        self.state = State.HOMING
        self._token += 1
        self._cancel_wait_timer()

        # 清理当前任务/步骤（保持第一版风格）
        self._task = None
        self._steps = []
        self._step_idx = 0

        if not self.mc:
            self._send_home_sim()
            return

        self.get_logger().info("🏠 (HW) 回到 Home ...")
        try:
            self.mc.send_angles(self.home_angles_deg, self.home_speed)
        except Exception as e:
            self.get_logger().error(f"home send failed: {e}")

        t0 = time.time()
        while time.time() - t0 < self.home_timeout_sec:
            try:
                if hasattr(self.mc, "is_moving") and (not self.mc.is_moving()):
                    break
            except Exception:
                pass
            time.sleep(0.1)

        if self.home_open_gripper and (not self.has_object):
            try:
                self._gripper_open()
            except Exception:
                pass

        self.state = State.IDLE

    def _send_home_sim(self):
        traj = JointTrajectory()
        traj.joint_names = list(self.sim_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(a) for a in self.home_angles_deg]
        pt.time_from_start.sec = int(self.sim_home_time_sec)
        pt.time_from_start.nanosec = int((self.sim_home_time_sec - int(self.sim_home_time_sec)) * 1e9)
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._token += 1
        token = self._token
        self.exec_ac.send_goal_async(goal).add_done_callback(lambda f: self._on_sim_home_sent(f, token))

    def _on_sim_home_sent(self, fut, token: int):
        if token != self._token:
            return
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("SIM home goal rejected")
            self.state = State.IDLE
            return
        gh.get_result_async().add_done_callback(lambda f: self._on_sim_home_done(f, token))

    def _on_sim_home_done(self, fut, token: int):
        if token != self._token:
            return
        self.state = State.IDLE

    def _reset(self):
        self._cancel_wait_timer()
        self._token += 1
        self._task = None
        self._steps = []
        self._step_idx = 0
        self.state = State.IDLE

    # =========================================================
    # Build plan request（带 start_state + 末端约束 + LIN 时更小 box）
    # =========================================================
    def _build_plan_request_with_start_state(
        self,
        x_mm,
        y_mm,
        z_mm,
        *,
        pipeline_id=None,
        planner_id=None,
        goal_box_m=None,
    ) -> GetMotionPlan.Request:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = x_mm / 1000.0
        pose.pose.position.y = y_mm / 1000.0
        pose.pose.position.z = z_mm / 1000.0

        roll = math.radians(self.ee_roll_deg)
        pitch = math.radians(self.ee_pitch_deg)
        yaw = math.radians(self.ee_yaw_deg)
        pose.pose.orientation = quat_from_rpy(roll, pitch, yaw)

        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = self.group_name
        mpr.allowed_planning_time = self.allowed_planning_time
        mpr.num_planning_attempts = self.num_planning_attempts
        mpr.max_velocity_scaling_factor = self.max_vel_scale
        mpr.max_acceleration_scaling_factor = self.max_acc_scale

        if pipeline_id:
            mpr.pipeline_id = str(pipeline_id)
        if planner_id:
            mpr.planner_id = str(planner_id)

        # start_state：真机且已掌握 MoveIt joint 顺序时，使用当前关节角作为规划起点
        if self.mc and self.moveit_joint_names is not None and self._last_joint_rad_by_moveit is not None:
            rs = RobotState()
            js = JointState()
            js.name = self.moveit_joint_names
            js.position = self._last_joint_rad_by_moveit
            js.header.stamp = self.get_clock().now().to_msg()
            rs.joint_state = js
            mpr.start_state = rs

        c = Constraints()

        # Position constraint (box)
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        if goal_box_m is None:
            box_size = self.pos_box_m_lin if planner_id == "LIN" else self.pos_box_m
        else:
            box_size = float(goal_box_m)

        box.dimensions = [box_size, box_size, box_size]

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        # Orientation constraint（tcp +Y 向下；允许绕 Y 自由转）
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation

        oc.absolute_x_axis_tolerance = float(self.tilt_tol_rad)
        oc.absolute_y_axis_tolerance = float(self.free_about_y_rad)
        oc.absolute_z_axis_tolerance = float(self.tilt_tol_rad)
        oc.weight = float(self.ori_weight)

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        mpr.goal_constraints.append(c)

        req.motion_plan_request = mpr
        return req

    # =========================================================
    # HW exec：插值发送更丝滑
    # =========================================================
    def _exec_hw_smooth_interpolated(self, traj: JointTrajectory) -> bool:
        if self.moveit_joint_names is None:
            self.get_logger().error("moveit_joint_names 未初始化，无法执行")
            return False

        name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        for n in self.moveit_joint_names:
            if n not in name_to_idx:
                self.get_logger().error(f"trajectory missing joint: {n}")
                return False

        waypoints = []
        for pt in traj.points:
            t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            moveit_rad = [pt.positions[name_to_idx[n]] for n in self.moveit_joint_names]
            hw_rad = [0.0] * 6
            for mi, hw_i in enumerate(self._moveit_name_to_hw_idx):
                hw_rad[hw_i] = moveit_rad[mi]
            waypoints.append((t, hw_rad))

        if len(waypoints) < 2:
            return False

        rate = max(10.0, float(self.hw_rate_hz))
        dt = 1.0 / rate
        total_t = waypoints[-1][0] * float(self.hw_time_scale)

        start_real = time.time()
        k = 0
        try:
            while True:
                elapsed = time.time() - start_real
                if elapsed >= total_t:
                    break

                t_ref = elapsed / float(self.hw_time_scale)
                while k < len(waypoints) - 2 and t_ref > waypoints[k + 1][0]:
                    k += 1

                t0, q0 = waypoints[k]
                t1, q1 = waypoints[k + 1]
                alpha = 1.0 if (t1 <= t0) else max(0.0, min(1.0, (t_ref - t0) / (t1 - t0)))
                q = [q0[i] + alpha * (q1[i] - q0[i]) for i in range(6)]
                deg = [math.degrees(x) for x in q]
                self.mc.send_angles(deg, self.hw_send_speed)
                time.sleep(dt)

            last = waypoints[-1][1]
            deg = [math.degrees(x) for x in last]
            self.mc.send_angles(deg, self.hw_send_speed)

            # 排空/等待：降低“紧接着夹爪指令被吞”的概率
            time.sleep(0.6)
            try:
                if hasattr(self.mc, "is_moving"):
                    t0 = time.time()
                    while time.time() - t0 < 1.5:
                        if not self.mc.is_moving():
                            break
                        time.sleep(0.05)
            except Exception:
                pass

            return True
        except Exception as e:
            self.get_logger().error(f"hardware smooth exec failed: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ArmWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()