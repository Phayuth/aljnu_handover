import rtde_control
import rtde_receive
import rtde_io
import robotiq_gripper
import math
import time


class RobotController:

    def __init__(self):
        self.hostip = "192.168.0.39"
        self.toolioport = 54321
        self.rtde_frequency = 500.0
        self.rtde_c = rtde_control.RTDEControlInterface(self.hostip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.hostip)
        self.rtde_i = rtde_io.RTDEIOInterface(self.hostip)

    def get_actual_q(self):
        return self.rtde_r.getActualQ()

    def get_jacobian(self):
        return self.rtde_c.getJacobian()

    def get_joint_torques(self):
        return self.rtde_c.getJointTorques()


if __name__ == "__main__":
    rc = RobotController()
    ip = "192.168.0.39"


    def log_info(gripper):
        print(
            f"Pos: {str(gripper.get_current_position()): >3}  "
            f"Open: {gripper.is_open(): <2}  "
            f"Closed: {gripper.is_closed(): <2}  "
        )


    gripper = robotiq_gripper.RobotiqGripper()
    gripper.connect(ip, 63352)
    if gripper.is_active():
        print("Gripper is active")
    else:
        gripper.activate()


    ack = gripper.move(150, 255, 255)
    print(f"Move command ack: {ack}")
    # gripper.direct_command()
    t = 0
    while True:
        p = 150 + int(50 * math.sin(t))
        ack = gripper.move(p, 255, 255)
        print(f"Move command ack: {ack}")
        t += 0.01
        time.sleep(0.01)
