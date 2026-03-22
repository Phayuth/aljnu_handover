import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis


def plot_parallel_gripper(
    ax,
    Hgrasp,
    finger_gap=0.08,
    finger_length=0.12,
    jaw_depth=0.02,
):
    """Plot a simple 2-finger parallel gripper.
    Finger direction: +Z axis of gripper frame.
    Opening/closing direction: X axis of gripper frame.
    """

    def transform_points(Hgrasp, points_local):
        """Transform Nx3 points from gripper frame to world frame."""
        points_h = np.hstack([points_local, np.ones((points_local.shape[0], 1))])
        return (Hgrasp @ points_h.T).T[:, :3]

    half_gap = 0.5 * finger_gap

    # Finger center lines in local frame.
    left_finger_local = np.array(
        [[-half_gap, 0.0, 0.0], [-half_gap, 0.0, finger_length]]
    )
    right_finger_local = np.array(
        [[half_gap, 0.0, 0.0], [half_gap, 0.0, finger_length]]
    )

    # Palm bar at z = 0 linking both fingers.
    palm_local = np.array([[-half_gap, 0.0, 0.0], [half_gap, 0.0, 0.0]])

    # Small jaw-depth lines make each finger easier to see in 3D.
    left_depth_local = np.array(
        [[-half_gap, -jaw_depth, 0.0], [-half_gap, jaw_depth, 0.0]]
    )
    right_depth_local = np.array(
        [[half_gap, -jaw_depth, 0.0], [half_gap, jaw_depth, 0.0]]
    )

    left_finger = transform_points(Hgrasp, left_finger_local)
    right_finger = transform_points(Hgrasp, right_finger_local)
    palm = transform_points(Hgrasp, palm_local)
    left_depth = transform_points(Hgrasp, left_depth_local)
    right_depth = transform_points(Hgrasp, right_depth_local)

    ax.plot(
        left_finger[:, 0],
        left_finger[:, 1],
        left_finger[:, 2],
        "r-",
        linewidth=3,
    )
    ax.plot(
        right_finger[:, 0],
        right_finger[:, 1],
        right_finger[:, 2],
        "r-",
        linewidth=3,
    )
    ax.plot(
        palm[:, 0],
        palm[:, 1],
        palm[:, 2],
        "k-",
        linewidth=2,
    )
    ax.plot(
        left_depth[:, 0],
        left_depth[:, 1],
        left_depth[:, 2],
        "k-",
        linewidth=2,
    )
    ax.plot(
        right_depth[:, 0],
        right_depth[:, 1],
        right_depth[:, 2],
        "k-",
        linewidth=2,
    )


H = np.array(
    [
        [0.0, 0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


ax = make_3d_axis(1.0, unit="m")
plot_transform(ax, np.eye(4), s=0.25)
plot_transform(ax, H, s=0.25)
plot_parallel_gripper(ax, H, finger_gap=0.08, finger_length=0.08, jaw_depth=0.02)
ax.set_box_aspect([1.0, 1.0, 1.0])
plt.show()
