import robotiq_gripper
import math
import time

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

