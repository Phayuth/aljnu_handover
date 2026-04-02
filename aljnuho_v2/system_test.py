import rtde_control
import rtde_receive
import rtde_io

print(dir(rtde_control))

print(dir(rtde_control.RTDEControlInterface))


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
