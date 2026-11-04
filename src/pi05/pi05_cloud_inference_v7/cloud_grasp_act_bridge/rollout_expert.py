"""Official LeRobot base/sync rollout with official PI05 FIFO with user runtime20 override.

Policy selection/reset are inherited official methods. The hardware timing
adapter binds each cached action to its original source-derived time slot;
startup, arming and cleanup use the existing independent PiPER executor.
"""
import logging
from nativequeuepolicy import track_native_queue
from control_logging import with_asynchronous_control_logging

LOG = logging.getLogger(__name__)


@with_asynchronous_control_logging
def execute_official(cfg, checked_policy, readiness, *, startup_argv=None,
                     startup_result=None, existing_mode_evidence=None,
                     expected_motion_ctrl2=(1, 1, 5, 0, 0, 0, 0, 0)):
    import lerobot.rollout.context as context_module
    from lerobot.rollout import build_rollout_context, create_strategy
    from lerobot.utils.process import ProcessSignalHandler
    from rollout_act_piper import official_build_injection, run_startup_once
    from camera_recovery import vla_camera_backend_injection
    from visual_gripper_runtime import visual_gripper_control

    if startup_argv is None and existing_mode_evidence is None:
        raise RuntimeError("Execution requires explicit startup or existing receiver mode evidence.")

    signal_handler = ProcessSignalHandler(use_threads=True, display_pid=False)
    shutdown = signal_handler.shutdown_event
    created_robots = []
    ctx = None
    strategy = create_strategy(cfg.strategy)
    failed = False
    cleanup_errors = []
    try:
        with official_build_injection(context_module, checked_policy, created_robots), vla_camera_backend_injection(context_module):
            ctx = build_rollout_context(cfg, shutdown)
        robot = ctx.hardware.robot_wrapper.inner
        if robot.guard is not None and robot.guard.armed:
            raise RuntimeError("Robot unexpectedly armed during context construction.")
        # Official base setup only prepares/resets the inference engine. Keep
        # the adapter unarmed throughout setup so it cannot consume lease TTL.
        strategy.setup(ctx)
        if shutdown.is_set():
            raise RuntimeError("Shutdown requested before arming.")
        if startup_argv is not None:
            if startup_result is None:
                raise RuntimeError("An explicit startup result path is required.")
            # The RX bridge is already running and can observe this preflight's
            # single 0x151. No receiver timestamp or mode evidence is fabricated.
            run_startup_once(startup_argv, startup_result,
                             expected_motion_ctrl2=expected_motion_ctrl2)
            if shutdown.wait(1.05):
                raise RuntimeError("Shutdown requested during post-startup control quiet.")
        # This is the only arm site. It anchors to actual current feedback and
        # emits no auto-home/enable/reset command. All future targets are still
        # checked by the independent execution adapter and receiver.
        robot.backend.refresh_unarmed_clock()
        robot.arm_from_current_state(readiness)
        with track_native_queue(checked_policy, robot, fps=cfg.fps), visual_gripper_control(
                robot, robot.backend.config['vla_visual_gripper_gate'],
                closure_config=robot.backend.config['vla_closure_sync'], policy=checked_policy):
            strategy.run(ctx)
    except BaseException:
        failed = True
        raise
    finally:
        shutdown.set()
        for robot in created_robots:
            try:
                if robot.is_connected:
                    robot.disarm("PI05 expert new cloud rollout finished or aborted")
            except BaseException as exc:
                cleanup_errors.append(f"disarm: {exc}")
                LOG.exception("Could not invalidate execution lease")
        if ctx is not None:
            try:
                strategy.teardown(ctx)
            except BaseException as exc:
                cleanup_errors.append(f"official teardown: {exc}")
                LOG.exception("Official teardown failed")
        for robot in created_robots:
            try:
                if robot.is_connected:
                    robot.disconnect()
            except BaseException as exc:
                cleanup_errors.append(f"disconnect: {exc}")
                LOG.exception("Could not complete adapter disconnect")
        if cleanup_errors and not failed:
            raise RuntimeError("Rollout cleanup failed: " + "; ".join(cleanup_errors))

