import numpy as np
import matplotlib.pyplot as plt
import rtde_control
import rtde_receive
import rtde_io
import robotiq_gripper
import cv2
import usb
import can
import time
import threading

np.set_printoptions(precision=4, suppress=True, linewidth=200)


class RobotController:

    def __init__(self, ft_mode=None):
        self.hostip = "192.168.0.39"
        self.toolioport = 54321
        self.rtde_frequency = 500.0
        self.rtde_c = rtde_control.RTDEControlInterface(self.hostip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.hostip)
        self.rtde_i = rtde_io.RTDEIOInterface(self.hostip)

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(self.hostip, 63352)
        self.activate_gripper()

        self.gripper_min = 0
        self.gripper_max = 255
        self.ft_sensor = AFT20D15(mode=ft_mode) if ft_mode is not None else None

    def activate_gripper(self):
        if self.gripper.is_active():
            print("Gripper is active")
        else:
            self.gripper.activate()

    def get_actual_tcp_pose(self):
        tcp_pose = np.array(self.rtde_r.getActualTCPPose())
        H = np.eye(4)
        H[:3, :3] = cv2.Rodrigues(tcp_pose[3:6])[0]
        H[:3, 3] = tcp_pose[0:3]
        return H

    def get_actual_tcp_speed(self):
        return self.rtde_r.getActualTCPSpeed()

    def make_pose_from_H(self, H):
        pose = np.zeros(6)
        pose[0:3] = H[:3, 3]
        pose[3:6] = cv2.Rodrigues(H[:3, :3])[0].flatten()
        return pose

    def make_H_from_pose(self, pose):
        H = np.eye(4)
        H[:3, :3] = cv2.Rodrigues(pose[3:6])[0]
        H[:3, 3] = pose[0:3]
        return H

    def get_actual_q(self):
        return self.rtde_r.getActualQ()

    def get_jacobian(self):
        return self.rtde_c.getJacobian()

    def get_joint_torques(self):
        return self.rtde_c.getJointTorques()

    def move_joints(self, q, vel=0.5, acc=0.5):
        self.rtde_c.moveJ(q, vel, acc)

    def move_tcp(self, pose, vel=0.1, acc=0.1, asynchronous=False):
        self.rtde_c.moveL(pose, vel, acc, asynchronous=asynchronous)

    def move_gripper(self, position, speed=255, force=255):
        pos = int(np.clip(position, self.gripper_min, self.gripper_max))
        return self.gripper.move(pos, speed, force)

    def close_gripper_until_z_threshold(
        self,
        delta_threshold=-4.0,
        target_position=255,
        speed=64,
        force=100,
    ):
        """
        그리퍼를 비동기로 이동시키면서 F/T 센서 Z값을 실시간 모니터링.
        초기 Z값 대비 delta_threshold 이상 감소하면 즉시 정지.
        """
        if self.ft_sensor is None:
            raise RuntimeError(
                "FT sensor is not initialized. Pass ft_mode when constructing RobotController."
            )

        # ── 현재 그리퍼 위치 확인 ──────────────────────────────
        current_position = self.gripper.get_current_position()
        if current_position is None:
            current_position = self.gripper_min
        current_position = int(np.clip(current_position, self.gripper_min, self.gripper_max))
        target_position  = int(np.clip(target_position,  self.gripper_min, self.gripper_max))

        if current_position >= target_position:
            print("Gripper already at or past target position.")
            return current_position, None

        # ── 초기 F/T Z값 측정 (기준값) ────────────────────────
        initial_ft = self.ft_sensor.receive()
        initial_z  = initial_ft[2] if initial_ft is not None and initial_ft[2] is not None else 0.0
        print(f"Initial FT z (baseline): {initial_z:.4f} N")
        print(f"Stop condition: FT z <= {initial_z + delta_threshold:.4f} N  "
              f"(= {initial_z:.4f} + ({delta_threshold:.4f}))")

        # ── 비동기 그리퍼 이동 명령 ───────────────────────────
        # move()는 즉시 반환 → 이후 루프에서 F/T를 실시간으로 읽을 수 있음
        self.gripper.move(target_position, speed, force)
        print(f"Gripper moving toward position {target_position} (async)...")

        # ── 실시간 F/T 모니터링 루프 ──────────────────────────
        last_z        = initial_z
        final_position = target_position

        while True:
            ft = self.ft_sensor.receive()

            if ft is None or ft[2] is None:
                print("[WARN] FT receive returned None, skipping...")
                continue

            last_z            = ft[2]
            current_pos_now   = self.gripper.get_current_position()
            threshold_value   = initial_z + delta_threshold

            print(f"  Gripper pos={current_pos_now:>3} | FT z={last_z:>8.4f} N "
                  f"| Δz={last_z - initial_z:>8.4f} N "
                  f"| threshold={threshold_value:.4f} N")

            # ── 종료 조건 1: F/T 임계값 초과 ─────────────────
            if last_z <= threshold_value:
                final_position = current_pos_now if current_pos_now is not None else target_position
                print(f"\n[STOP] FT z threshold reached!")
                print(f"       FT z={last_z:.4f} <= threshold={threshold_value:.4f}")
                print(f"       Delta from initial: {last_z - initial_z:.4f} N")
                # 즉시 정지: 현재 위치에서 그리퍼 멈춤
                self.gripper.move(final_position, speed, force)
                break

            # ── 종료 조건 2: 그리퍼가 목표에 도달 ──────────────
            if current_pos_now is not None and current_pos_now >= target_position:
                final_position = current_pos_now
                print(f"\n[DONE] Gripper reached target position {target_position} "
                      f"without triggering threshold.")
                break

        return final_position, last_z

    def start_contact_detection(self):
        self.rtde_c.startContactDetection()

    def read_contact(self):
        return self.rtde_c.readContactDetection()

    def stop_contact_detection(self):
        self.rtde_c.stopContactDetection()


class AFT20D15:

    def __init__(self, mode) -> None:
        if mode == "usb":
            self.dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
            if self.dev is None:
                raise RuntimeError("USB-CAN device not found.")
            self.bus = can.Bus(
                interface="gs_usb",
                channel=self.dev.product,
                bus=self.dev.bus,
                address=self.dev.address,
                bitrate=1000000,
            )
        elif mode == "robotell":
            self.bus = can.Bus(
                interface="robotell",
                channel="/dev/ttyUSB0@115200",
                rtscts=True,
                bitrate=1000000,
            )
            print("Robotell CAN bus initialized")
        elif mode == "socket":
            self.bus = can.Bus(channel="can0", interface="socketcan")
        else:
            raise ValueError(f"Unknown mode: '{mode}'. Choose from 'usb', 'robotell', 'socket'.")

        self.forceid  = int(0x01A)
        self.torqueid = int(0x01B)

    def byte_to_output(self, data):
        # Big-Endian signed int16 변환
        xout = int.from_bytes(data[0:2], byteorder='big', signed=True)
        yout = int.from_bytes(data[2:4], byteorder='big', signed=True)
        zout = int.from_bytes(data[4:6], byteorder='big', signed=True)
        return [xout, yout, zout]

    def get_force(self, data: list):
        return [data[0] / 1000.0 - 30.0,
                data[1] / 1000.0 - 30.0,
                data[2] / 1000.0 - 30.0]

    def get_torque(self, data: list):
        return [data[0] / 100000.0 - 0.3,
                data[1] / 100000.0 - 0.3,
                data[2] / 100000.0 - 0.3]

    def receive(self, timeout=0.05, max_attempts=20):
        """
        Force(0x01A)와 Torque(0x01B) 메시지를 모두 수신할 때까지 대기.
        timeout을 0.05초로 짧게 설정 → 실시간성 확보
        """
        ft       = [None] * 6
        attempts = 0

        while (ft[0] is None or ft[3] is None) and attempts < max_attempts:
            rxmsg = self.bus.recv(timeout=timeout)
            attempts += 1

            if rxmsg is None:
                continue

            if len(rxmsg.data) < 6:
                continue

            dataints = self.byte_to_output(rxmsg.data)
            canid    = rxmsg.arbitration_id

            if canid == self.forceid:
                force = self.get_force(dataints)
                ft[0], ft[1], ft[2] = force[0], force[1], force[2]
            elif canid == self.torqueid:
                torque = self.get_torque(dataints)
                ft[3], ft[4], ft[5] = torque[0], torque[1], torque[2]

        return ft

    def shutdown(self):
        self.bus.shutdown()


def test_gripper_sensors():
    robot_real = RobotController(ft_mode="robotell")
    robot_real.move_gripper(0)

    time.sleep(2)  # 초기 안정화 시간

    final_pos, final_z = robot_real.close_gripper_until_z_threshold(
        delta_threshold=-0.3,   # 초기값에서 1N 감소하면 정지
        target_position=255,
        speed=0,
        force=0,
    )
    print(f"\nFinal gripper position : {final_pos}")
    print(f"Final FT z             : {final_z:.4f} N")

    if robot_real.ft_sensor is not None:
        robot_real.ft_sensor.shutdown()
        print("FT sensor shutdown complete")


if __name__ == "__main__":
    test_gripper_sensors()
