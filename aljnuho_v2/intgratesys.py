import numpy as np
import select
import sys
import termios
import tty
import atexit
import signal
from multiprocessing import shared_memory
from tracker_class import UKFRiskTracker
from ur5e_ik import RobotUR5eKin, RobotController
from msd import MSDRobotEE
from intergrate_system import IntegrateSystem
import time
from enum import Enum

robot_real = RobotController()
robot_kin = RobotUR5eKin()
tracker = UKFRiskTracker()
eemodel = MSDRobotEE()
intsyst = IntegrateSystem()


# live control parameters
vel = 0.1
acc = 0.1
dt = 1.0 / 500  # 2ms
lokat = 0.1
gain = 200

Hhome = robot_kin.solve_fk(intsyst.qhomeflip)
qcurrent = robot_real.get_actual_q()
Hcurrent = robot_real.get_actual_tcp_pose()
speed = np.zeros(3)
speed6d = np.zeros(6)
Rfixed = Hcurrent.copy()
Rfixed = Rfixed[:3, :3]

Hpreplace = Hhome.copy()
Hpreplace[0, 3] += 0.0
Hpreplace[1, 3] -= 0.35
Hpreplace[2, 3] += 0.0

Hplace = Hpreplace.copy()
Hplace[2, 3] = 0.07
print(f"==>> Hplace in Base Frame: \n{Hplace}")

# Hretract = Hplace.copy()
# Hretract[1, 3] += 0.2

# initial measurement
obj_z_meas = np.zeros(6, dtype=np.float64)
obj_z_meas_filtered = np.zeros(6, dtype=np.float64)
prev_xyz_c = None


class MoveMode(Enum):
    Servo_Cart = 0
    Servo_Joint = 1
    Move_L = 2


class GripperMode(Enum):
    chase_object = 0
    preplace_object = 1
    place_object = 2
    move_home = 3


real_action = {
    "move": False,
    "grip_delta": 0,
    "rot_x_delta": 0.0,
    "grip_pos": 0,
    "move_to_place": False,
    "obj_grasped": False,
    "move_chase_object": False,
    "move_to_home": False,
    "cancel_now": False,
}
reach_mode = {"follow_ellipse": False}
move_mode_now = MoveMode.Servo_Cart
gripper_mode_now = GripperMode.chase_object
move_preplace_done = False
move_place_done = False
move_retract_done = False
gripper_trigger = False
stop_update_desired = False
Hdesiredsave = None
contact_det_started = False
is_contact = False


def toggle_action(key):
    global intsyst

    def set_mode(mode_key):
        next_state = not real_action[mode_key]
        real_action["move_to_place"] = False
        real_action["move_chase_object"] = False
        real_action["move_to_home"] = False
        real_action[mode_key] = next_state

    if key == "b":
        real_action["move"] = not real_action["move"]
        if not real_action["move"]:
            real_action["cancel_now"] = True
    if key == "p":
        set_mode("move_to_place")
    if key == "c":
        set_mode("move_chase_object")
    if key == "h":
        set_mode("move_to_home")
    if key == "x":
        real_action["cancel_now"] = True


def setup_nonblocking_keyboard():
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return fd, old_term


def restore_keyboard(fd, old_term):
    if fd is None or old_term is None:
        return
    termios.tcsetattr(fd, termios.TCSADRAIN, old_term)


def consume_keypresses(fd):
    if fd is None:
        return
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            break
        key = sys.stdin.read(1)
        if not key:
            break
        toggle_action(key)


def cleanup_terminal():
    global kb_fd, kb_old_term
    restore_keyboard(kb_fd, kb_old_term)


def _signal_cleanup_then_exit(signum, _frame):
    cleanup_terminal()
    signal.signal(signum, signal.SIG_DFL)
    raise SystemExit(128 + signum)


def move_gripper_hold(position):
    robot_real.move_gripper(position)
    time.sleep(1)


# set real to initial state
robot_real.move_gripper(real_action["grip_pos"])
robot_real.move_joints(intsyst.qhomeflip)
robot_real.move_gripper(real_action["grip_pos"])


kb_fd = None
kb_old_term = None
kb_state = setup_nonblocking_keyboard()
if kb_state is not None:
    kb_fd, kb_old_term = kb_state

atexit.register(cleanup_terminal)
for _sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
    signal.signal(_sig, _signal_cleanup_then_exit)

shm_found = False
try:
    shm = shared_memory.SharedMemory(name="camera_zed_centroid", create=False)
    centroids = np.ndarray((4,), dtype=np.float64, buffer=shm.buf)
    shm_found = True
except FileNotFoundError:
    print("Shared memory 'camera_zed_centroid' not found.")

print("Starting control loop")

is_start_time_recorded = False
is_contact_time_record = False
is_elapsed_time_record = False

try:
    while True:
        consume_keypresses(kb_fd)

        if real_action["cancel_now"]:
            real_action["cancel_now"] = False
            real_action["move"] = False
            real_action["move_to_place"] = False
            real_action["move_chase_object"] = False
            real_action["move_to_home"] = False
            real_action["grip_delta"] = 0
            speed[:] = 0.0
            # Stop active servo stream immediately, then wait for next user command.
            robot_real.rtde_c.servoStop()

        t_start = robot_real.rtde_c.initPeriod()

        if shm_found:
            xyz_c = centroids[:3].copy()
            if prev_xyz_c is None:
                vxyz_c = np.zeros(3, dtype=np.float64)
            else:
                vxyz_c = (xyz_c - prev_xyz_c) / dt

            obj_z_meas[:3] = xyz_c
            obj_z_meas[3:] = vxyz_c
            prev_xyz_c = xyz_c
        else:
            obj_z_meas = np.array([-0.017, -0.7338, 0.2861, 0.0, 0.0, 0.0])

        # UKF update
        tracker.ukf.predict(dt=dt)
        tracker.ukf.update(obj_z_meas)

        # get filtered state estimate
        # xobj = tracker.ukf.x[0]
        # yobj = tracker.ukf.x[1]
        # zobj = tracker.ukf.x[2]
        xobj = obj_z_meas[0]
        yobj = obj_z_meas[1]
        zobj = obj_z_meas[2]

        # update covariance ellipse
        cov_e_w, cov_e_h, cov_e_d, cov_e_angle_xy, cov_e_angle_xz = (
            tracker.pose_covariance_ellipse_3d(tracker.ukf.P)
        )

        # update risk ellipse
        speed_xyz = np.linalg.norm(tracker.ukf.x[3:6])
        # print(f"==>> obj_z_meas: \n{obj_z_meas}")
        # print(f"==>> centroid measurement: \n{centroids[:3]}")
        # print(f"==>> speed_xyz: \n{speed_xyz}")
        rk_e_w, rk_e_h, rk_e_d, rk_e_angle_xy, rk_e_angle_xz = (
            tracker.risk_covariance_ellipse_3d(tracker.ukf.P, speed=speed_xyz)
        )

        # detect if object is out of workspace boundary
        if not intsyst.is_obj_in_boundary((xobj, yobj, zobj), intsyst.rworkspace):
            radius_bound = intsyst.rreach
        else:
            radius_bound = intsyst.rworkspace

        # grasp point projection and reach projection
        xproj, yproj, zproj = intsyst.project_point_on_rotated_ellipse3d(
            center=(xobj, yobj, zobj),
            width=rk_e_w,
            height=rk_e_h,
            depth=rk_e_d,
            angle_xy_deg=rk_e_angle_xy,
            angle_xz_deg=rk_e_angle_xz,
            target_point=(Hhome[0, 3], Hhome[1, 3], Hhome[2, 3]),
        )
        xreach, yreach, zreach, reach_mode["follow_ellipse"] = (
            intsyst.select_reach_target_model_3d(
                ee_point=(Hhome[0, 3], Hhome[1, 3], Hhome[2, 3]),
                ellipse_proj_point=(xproj, yproj, zproj),
                radius=radius_bound,
                follow_ellipse=reach_mode["follow_ellipse"],
            )
        )

        if gripper_trigger:
            real_action["move"] = True
            gripper_trigger = False

        elif real_action["move"]:
            if not is_start_time_recorded:
                start_time = time.time()
                is_start_time_recorded = True

            # Default to holding current pose so servoL always receives a valid target.
            Hcurrent = robot_real.get_actual_tcp_pose()
            qcurrent = robot_real.get_actual_q()
            pose_cl = robot_real.make_pose_from_H(Hcurrent)
            q_cl = robot_real.get_actual_q()

            if real_action["move_to_home"]:
                if not is_elapsed_time_record:
                    final_time = time.time()
                    full_elapsed = final_time - start_time
                    print(f"Total elapsed time: {full_elapsed:.2f} seconds")
                    is_elapsed_time_record = True

                real_action["move_chase_object"] = False
                real_action["move_to_place"] = False
                if gripper_mode_now != GripperMode.move_home:
                    move_gripper_hold(intsyst.gripper_open)
                    gripper_mode_now = GripperMode.move_home
                move_mode_now = MoveMode.Servo_Joint
                q_cl = intsyst.interp_q(qcurrent, intsyst.qhomeflip, alpha=0.1)

            elif real_action["move_to_place"]:
                real_action["move_chase_object"] = False
                real_action["move_to_home"] = False
                move_mode_now = MoveMode.Servo_Cart
                if not is_contact_time_record:
                    t_human_last_contact = time.time()
                    thlc = t_human_last_contact - start_time
                    print(f"==>> thlc: \n{thlc}")
                    is_contact_time_record = True

                # move to preplace
                if not move_preplace_done:
                    if gripper_mode_now != GripperMode.preplace_object:
                        move_gripper_hold(intsyst.gripper_close)
                        gripper_mode_now = GripperMode.preplace_object

                    Hctrl, speed = eemodel.step2(
                        T_ee=Hcurrent,
                        vel3d=speed,
                        T_obj=Hpreplace,
                    )
                    pose_cl = robot_real.make_pose_from_H(Hctrl)
                    Dist = np.linalg.norm(Hpreplace[:3, 3] - Hcurrent[:3, 3])
                    if Dist < 0.009:
                        move_preplace_done = True

                # move to place
                elif not move_place_done:
                    if not contact_det_started:
                        robot_real.start_contact_detection()
                        contact_det_started = True
                    if contact_det_started:
                        is_contact = robot_real.read_contact()
                        if is_contact:
                            move_place_done = True
                            robot_real.stop_contact_detection()
                            if gripper_mode_now != GripperMode.place_object:
                                move_gripper_hold(intsyst.gripper_open)
                                gripper_mode_now = GripperMode.place_object
                            Hcontract = robot_real.get_actual_tcp_pose()

                    Hctrl, speed = eemodel.step2(
                        T_ee=Hcurrent,
                        vel3d=speed,
                        T_obj=Hplace,
                    )
                    pose_cl = robot_real.make_pose_from_H(Hctrl)
                    Dist = np.linalg.norm(Hplace[:3, 3] - Hcurrent[:3, 3])
                    if Dist < 0.009:
                        move_place_done = True
                        if gripper_mode_now != GripperMode.place_object:
                            move_gripper_hold(intsyst.gripper_open)
                            gripper_mode_now = GripperMode.place_object
                    Hcontract = None

                # move to retract
                else:
                    if Hcontract is None:
                        Hcontract = Hplace.copy()
                    Hretract = intsyst.make_retract(Hcontract, retract_dist=0.1)
                    Hctrl, speed = eemodel.step2(
                        T_ee=Hcurrent,
                        vel3d=speed,
                        T_obj=Hretract,
                    )
                    pose_cl = robot_real.make_pose_from_H(Hctrl)
                    Dist = np.linalg.norm(Hretract[:3, 3] - Hcurrent[:3, 3])
                    if Dist < 0.01:
                        move_retract_done = True
                        real_action["move_to_place"] = False
                        real_action["move_to_home"] = True

            elif real_action["move_chase_object"]:
                real_action["move_to_place"] = False
                real_action["move_to_home"] = False
                move_mode_now = MoveMode.Servo_Cart
                obj_pos = np.array([xobj, yobj, zobj])
                reach_pos = np.array([xreach, yreach, zreach])

                # Hdesired = intsyst.make_grasp_pose(
                #     obj_P=obj_pos,
                #     reach_P=reach_pos,
                #     ee_pose=Hcurrent,
                # )

                # Hdesired = intsyst.make_grasp_pose_fixedR(
                #     obj_P=obj_pos,
                #     reach_P=reach_pos,
                #     ee_pose=Hcurrent,
                #     Rfixed=Rfixed,
                # )

                if not stop_update_desired:
                    Hdesired = intsyst.make_grasp_variable_z(
                        obj_P=obj_pos,
                        reach_P=reach_pos,
                        ee_pose=Hcurrent,
                        Rfixed=Rfixed,
                    )
                    Hdesiredsave = Hdesired.copy()
                else:
                    Hdesired = Hdesiredsave.copy()

                Dist_stop_update = np.linalg.norm(
                    Hdesired[:3, 3] - Hcurrent[:3, 3]
                )
                if Dist_stop_update < 0.3:
                    stop_update_desired = True

                Hctrl, speed = eemodel.step2(
                    T_ee=Hcurrent,
                    vel3d=speed,
                    T_obj=Hdesired,
                )
                Dist = np.linalg.norm(Hdesired[:3, 3] - Hcurrent[:3, 3])
                if Dist >= 0.005:
                    pose_cl = robot_real.make_pose_from_H(Hctrl)
                else:
                    pose_cl = robot_real.make_pose_from_H(Hdesired)
                    speed[:] = 0.0
                    real_action["move_chase_object"] = False
                    real_action["move_to_place"] = True

            # control mode selection
            if move_mode_now == MoveMode.Servo_Cart:
                robot_real.rtde_c.servoL(pose_cl, vel, acc, dt, lokat, gain)
                robot_real.rtde_c.waitPeriod(t_start)
            elif move_mode_now == MoveMode.Servo_Joint:
                robot_real.rtde_c.servoJ(q_cl, vel, acc, dt, lokat, gain)
                robot_real.rtde_c.waitPeriod(t_start)
            elif move_mode_now == MoveMode.Move_L:
                robot_real.move_tcp(
                    pose_cl, vel=0.05, acc=0.05, asynchronous=False
                )

except KeyboardInterrupt:
    robot_real.rtde_c.servoStop()
    robot_real.rtde_c.stopScript()
    shm_obj = globals().get("camera_zed_centroid")
    if shm_obj is not None:
        shm_obj.close()
    cleanup_terminal()
    print("Program terminated.")

finally:
    print("Program terminated.")
