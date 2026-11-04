"""Arm-only ROS action bridge. Read-only unless an exact trajectory is armed.

The operator approves the plan before starting with --execute and its digest.
Collision planning and measured finger geometry belong to the caller. This
bridge never enables the arm, controls grip width, or claims grip contact.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes

from .sdk_executor import (NAMES, Mapping, CliTransport, Cancelled,
                           prepare, execute, healthy)
from .path_timing import PROFILE


def trajectory_plan(trajectory, mapping, timing_profile=PROFILE, *, rate=10., speed=.5,
                    acceleration=1., respect_source_timing=True):
    if trajectory.multi_dof_joint_trajectory.points:
        raise ValueError('Floating-base motion is unsupported')
    jt = trajectory.joint_trajectory
    return prepare(list(jt.joint_names), [list(p.positions) for p in jt.points],
                   [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points], mapping,
                   timing_profile=timing_profile, rate=rate, speed=speed,
                   acceleration=acceleration, respect_source_timing=respect_source_timing)


def retime_trajectory(trajectory, mapping, timing_profile=PROFILE, **timing_options):
    """Give clients the actual bounded duration before they set action deadlines."""
    result = copy.deepcopy(trajectory)
    plan = trajectory_plan(result, mapping, timing_profile, **timing_options)
    apply_plan_timing(result, plan, mapping)
    return result


def apply_plan_timing(result, plan, mapping):
    """Expose continuous waypoint derivatives in model units, never fake stops."""
    times = plan.get('waypoint_times_s')
    if times is None:
        times = [0.]
        for ticks in plan['ticks']:
            times.append(times[-1]+ticks/plan['rate'])
    for i, point in enumerate(result.joint_trajectory.points):
        ns = round(times[i] * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
        for field, key in [('velocities', 'waypoint_velocities_deg_s'),
                           ('accelerations', 'waypoint_accelerations_deg_s2')]:
            sdk_values = plan.get(key, [[0.]*6]*len(times))[i]
            model_values = dict(zip(NAMES, mapping.signs*np.deg2rad(sdk_values)))
            setattr(point, field, [float(model_values[n]) for n in result.joint_trajectory.joint_names])
        point.effort = []


def trajectory_digest(trajectory):
    # CDR alignment bytes are not message fields and may be uninitialized.
    # Hash the canonical field values so equivalent ROS round trips are stable.
    fields = message_to_ordereddict(trajectory)
    payload = json.dumps(fields, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest()


def timed_execution_binding(trajectory, plan):
    """Bind the exact ROS path, retimed plan and actual timed execution options.

    Import only for the opt-in mode. The executor owns the parameter definition
    so a changed buffer, lateness limit or endpoint rule cannot silently differ
    between goal admission and execution.
    """
    from .sdk_timed_executor import execution_parameters
    binding = dict(schema='d1_timed_execution_binding_v1',
        trajectory_sha256=trajectory_digest(trajectory),
        scheduling_policy='timed_group_v1', sdk_plan=plan,
        execution_parameters=execution_parameters(plan),
        gripper_mode='preserve', endpoint_policy=None)
    payload = json.dumps(binding,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()
    return dict(binding=binding,sha256=hashlib.sha256(payload).hexdigest())


class Bridge(Node):
    def __init__(self, args):
        super().__init__('d1_hardware_bridge')
        self.args = args
        self.mapping = Mapping.from_candidate(args.candidate)
        self.lock = threading.Lock()
        self.busy = False
        self.used = False
        self.stopping = threading.Event()
        self.camera = None
        self.last_seq = -1
        self.scheduling_policy = getattr(args,'scheduling_policy','ack_then_period_v1')
        self.execution_binding = None
        args.output.mkdir(parents=True, exist_ok=False)
        if self.scheduling_policy == 'timed_group_v1':
            from .timed_group_transport import TimedGroupTransport
            timed_binary = getattr(args,'timed_binary',None) or args.control_root/'host_build/d1_timed_bridge'
            self.transport = TimedGroupTransport(timed_binary,args.nic,args.output/'transport.jsonl')
        else:
            self.transport = CliTransport(args.binary, args.nic, args.output / 'transport.jsonl')
        group = ReentrantCallbackGroup()
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.status = self.create_publisher(String, '/d1/hardware_status', 10)
        self.timer = self.create_timer(.05, self.publish_state, callback_group=group)
        self.action = ActionServer(self, ExecuteTrajectory, '/d1/execute_trajectory',
            self.run_goal, goal_callback=self.accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.get_logger().info('READ ONLY' if not args.execute else 'One approved arm trajectory armed')

    def publish_state(self):
        s = self.transport.snapshot()
        now = self.transport.now()
        self.status.publish(String(data=json.dumps(dict(
            sdk_j6_raw=None if s['q'] is None else float(s['q'][6]),
            status=s['status'], angle_age_s=None if s['q'] is None else now-s['q_time'],
            fault=s['fault'], armed=self.args.execute and not self.used,
            gripper_width_measured=False))))
        # Republished old data must not look like new angle feedback.
        if s['q'] is not None and s['q_seq'] != self.last_seq and now-s['q_time'] <= .6:
            self.last_seq = s['q_seq']
            msg = JointState(name=NAMES, position=self.mapping.to_model(s['q']).tolist())
            stamp = self.get_clock().now().nanoseconds - int((now-s['q_time'])*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            self.joints.publish(msg)

    def accept_goal(self, goal):
        with self.lock:
            if not self.args.execute or self.busy or self.used:
                return GoalResponse.REJECT
            try:
                if trajectory_digest(goal.trajectory) != self.args.trajectory_sha256:
                    raise ValueError('Trajectory differs from the approved artifact')
                plan = trajectory_plan(goal.trajectory, self.mapping)
                if self.scheduling_policy == 'timed_group_v1':
                    # Native group execution needs powered, error-free, fresh
                    # actual feedback; software enable=0 is a valid start.
                    healthy(self.transport.snapshot(),self.transport.now(),allow_unstarted_position=True)
                    binding = timed_execution_binding(goal.trajectory,plan)
                else:
                    healthy(self.transport.snapshot(), self.transport.now())
                    binding = None
                if goal.controller_names:
                    raise ValueError('Controller overrides are unsupported')
                if binding is not None:
                    (self.args.output/'execution_binding.json').write_text(json.dumps(binding,indent=2))
                    self.execution_binding = binding
            except Exception as error:
                self.get_logger().error(str(error))
                return GoalResponse.REJECT
            self.busy = self.used = True  # One-shot: never replay after failure/cancellation.
            return GoalResponse.ACCEPT

    def run_goal(self, handle):
        result = ExecuteTrajectory.Result()
        report = dict(completed=False, grasp_executed=False,
                      trajectory_sha256=trajectory_digest(handle.request.trajectory),
                      scheduling_policy=self.scheduling_policy)
        try:
            sys.path.insert(0, str(self.args.control_root / 'tools'))
            from handeye_joint_step import Cameras
            self.camera = Cameras(self.args.output / 'frames')
            deadline = time.monotonic() + 5
            while not self.camera.ready:
                if self.camera.error or time.monotonic() > deadline:
                    self.camera.check()
                if handle.is_cancel_requested or self.stopping.is_set():
                    raise Cancelled('Cancelled during read-only camera preflight')
                time.sleep(.05)
            plan = trajectory_plan(handle.request.trajectory, self.mapping)
            if self.scheduling_policy == 'timed_group_v1':
                binding = timed_execution_binding(handle.request.trajectory,plan)
                if self.execution_binding is None or binding != self.execution_binding:
                    raise ValueError('Timed trajectory or execution parameters changed after goal admission')
                report['execution_binding'] = binding
                report['gripper_mode'] = 'preserve'
                report['endpoint_retries_enabled'] = False
            report['retimed_duration_s'] = plan['duration_s']
            (self.args.output / 'arm_plan.json').write_text(json.dumps(plan, indent=2))
            handle.publish_feedback(ExecuteTrajectory.Feedback(state='EXECUTING'))
            report.update(execute(plan, self.transport,
                cancel=lambda: handle.is_cancel_requested or self.stopping.is_set(),
                external_check=self.camera.check,scheduling_policy=self.scheduling_policy))
            result.error_code.val = MoveItErrorCodes.SUCCESS
            handle.succeed()
        except Cancelled as error:
            report.update(error=str(error), hold_command_received=False,
                          future_commands_stopped=True, physical_stop_verified=False)
            result.error_code.val = MoveItErrorCodes.PREEMPTED
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        except Exception as error:
            report['error'] = str(error)
            result.error_code.val = MoveItErrorCodes.CONTROL_FAILED
            handle.abort()
            self.get_logger().error(str(error))
        finally:
            if self.camera:
                self.camera.close()
                self.camera = None
            (self.args.output / 'result.json').write_text(json.dumps(report, indent=2))
            with self.lock:
                self.busy = False
        return result

    def close(self):
        self.stopping.set()
        self.action.destroy()
        if self.camera:
            self.camera.close()
        self.transport.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--control-root', type=Path, default=Path('/home/aup/D1/Control'))
    parser.add_argument('--binary', type=Path, default=Path('/home/aup/D1/Control/build/arm_control'))
    parser.add_argument('--scheduling-policy',choices=['ack_then_period_v1','timed_group_v1'],
                        default='ack_then_period_v1',help='Timed group execution is opt-in; legacy default is unchanged')
    parser.add_argument('--timed-binary',type=Path,
                        help='Default: <control-root>/host_build/d1_timed_bridge')
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--trajectory-sha256')
    args = parser.parse_args(argv)
    if args.timed_binary is None:
        args.timed_binary=args.control_root/'host_build/d1_timed_bridge'
    if args.execute and (not args.trajectory_sha256 or len(args.trajectory_sha256) != 64
                         or any(x not in '0123456789abcdef' for x in args.trajectory_sha256)):
        parser.error('--execute requires the SHA256 of the explicitly approved trajectory')
    rclpy.init()
    node = Bridge(args)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stopping.set()
        executor.shutdown(timeout_sec=8.)
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
