import numpy as np


class PlanarRRR:

    def __init__(self):
        # kinematic
        self.a1 = 0.5
        self.a2 = 0.5
        self.a3 = 0.1
        self.reach = self.a1 + self.a2 + self.a3

    def forward_kinematic(self, theta):
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]

        H01 = np.array(
            [
                [np.cos(theta1), -np.sin(theta1), 0, 0],
                [np.sin(theta1), np.cos(theta1), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        H12 = np.array(
            [
                [np.cos(theta2), -np.sin(theta2), 0, self.a1],
                [np.sin(theta2), np.cos(theta2), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        H23 = np.array(
            [
                [np.cos(theta3), -np.sin(theta3), 0, self.a2],
                [np.sin(theta3), np.cos(theta3), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        H34 = np.array(
            [
                [1, 0, 0, self.a3],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        H04 = H01 @ H12 @ H23 @ H34
        phi = theta1 + theta2 + theta3

        return np.array([H04[0, 3], H04[1, 3], phi])

    def forward_kinematic_link(self, theta):
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]

        # link 1 pose
        x1 = self.a1 * np.cos(theta1)
        y1 = self.a1 * np.sin(theta1)

        # link 2 pose
        x2 = self.a1 * np.cos(theta1) + self.a2 * np.cos(theta1 + theta2)
        y2 = self.a1 * np.sin(theta1) + self.a2 * np.sin(theta1 + theta2)

        # link 3 pose
        x3 = (
            self.a1 * np.cos(theta1)
            + self.a2 * np.cos(theta1 + theta2)
            + self.a3 * np.cos(theta1 + theta2 + theta3)
        )
        y3 = (
            self.a1 * np.sin(theta1)
            + self.a2 * np.sin(theta1 + theta2)
            + self.a3 * np.sin(theta1 + theta2 + theta3)
        )

        return np.array([[0, 0], [x1, y1], [x2, y2], [x3, y3]])

    def inverse_kinematic_geometry(self, desiredConfig):
        x = desiredConfig[0]
        y = desiredConfig[1]
        phi = desiredConfig[2]

        x2 = x - self.a3 * np.cos(phi)
        y2 = y - self.a3 * np.sin(phi)

        if np.hypot(x2, y2) > self.reach:
            return None

        t2term = (x2**2 + y2**2 - self.a1**2 - self.a2**2) / (
            2 * self.a1 * self.a2
        )
        # Ensure it's within the valid domain
        t2term = np.clip(t2term, -self.reach, self.reach)

        theta2_up = -1 * np.arccos(t2term)
        theta1_up = np.arctan2(y2, x2) - np.arctan2(
            self.a2 * np.sin(theta2_up), self.a1 + self.a2 * np.cos(theta2_up)
        )
        theta3_up = phi - theta1_up - theta2_up

        theta2_dn = np.arccos(t2term)
        theta1_dn = np.arctan2(y2, x2) - np.arctan2(
            self.a2 * np.sin(theta2_dn), self.a1 + self.a2 * np.cos(theta2_dn)
        )
        theta3_dn = phi - theta1_dn - theta2_dn
        return np.array(
            [
                [theta1_up, theta2_up, theta3_up],
                [theta1_dn, theta2_dn, theta3_dn],
            ]
        )

    def jacobian(self, theta):
        theta1 = theta[0]
        theta2 = theta[1]
        theta3 = theta[2]

        J = np.array(
            [
                [
                    -self.a1 * np.sin(theta1)
                    - self.a2 * np.sin(theta1 + theta2)
                    - self.a3 * np.sin(theta1 + theta2 + theta3),
                    -self.a2 * np.sin(theta1 + theta2)
                    - self.a3 * np.sin(theta1 + theta2 + theta3),
                    -self.a3 * np.sin(theta1 + theta2 + theta3),
                ],
                [
                    self.a1 * np.cos(theta1)
                    + self.a2 * np.cos(theta1 + theta2)
                    + self.a3 * np.cos(theta1 + theta2 + theta3),
                    self.a2 * np.cos(theta1 + theta2)
                    + self.a3 * np.cos(theta1 + theta2 + theta3),
                    self.a3 * np.cos(theta1 + theta2 + theta3),
                ],
                [1, 1, 1],
            ]
        )

        return J


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    robot = PlanarRRR()

    Xd = np.array([0.5, 0.0, 0.9])
    tik = robot.inverse_kinematic_geometry(Xd)
    print(f"==>> tik: \n{tik}")
    # theta = np.array([0.1, 0.1, 0.1])

    link_pos = robot.forward_kinematic_link(tik[0])

    fig, ax = plt.subplots()
    ax.plot(link_pos[:, 0], link_pos[:, 1], "o-")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")
    plt.show()
