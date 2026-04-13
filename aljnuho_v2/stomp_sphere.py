import numpy as np
import matplotlib.pyplot as plt
from stomp import STOMP, STOMPUTILS


def compute_traj_obscost(noisy_trajectories):
    # input : noisy_trajectories = K, N, d
    # output : costfully = K, N, d
    bodytoobsfulltraj = noisy_trajectories - obscxy.flatten()
    dminfull = np.linalg.norm(bodytoobsfulltraj, axis=2) - obsr - bodyr
    costfull = np.maximum(
        clearance + bodyr - dminfull, 0
    )  # i only have 1 body and 1 obstacle so no sum
    costfully = np.repeat(costfull[:, :, np.newaxis], 2, axis=2)
    return costfully


def compute_trajectory_obstacle_cost(trajectory):
    bodytoobs = trajectory - obscxy.flatten()
    dmin = np.linalg.norm(bodytoobs, axis=1) - obsr - bodyr
    cost = np.maximum(clearance + bodyr - dmin, 0)
    return np.sum(cost)


def compute_traj_zero_cost(noisy_trajectories):
    return np.zeros_like(noisy_trajectories)


def compute_trajectory_zero_cost(trajectory):
    return 0.0


if __name__ == "__main__":
    K = 50
    N = 50
    d = 2
    h = 10
    tolerance = 0.1
    Rscale = 10
    stddev = [0.5] * 2
    num_iterations = 40

    # environment
    theta_s = np.array([-2.0, -2.0])
    theta_g = np.array([2.0, 2.0])
    xlim = [-np.pi, np.pi]
    ylim = [-np.pi, np.pi]
    # obstacle
    obscxy = np.array([0.0, 0.0]).reshape(2, 1)
    obsr = 0.7
    bodyr = 0.2
    clearance = 0.1
    times = np.linspace(0, N, num=N)

    c = theta_g.copy()
    n = np.array([1.0, -1.0])
    L = 0.5
    R = 0.3

    # generate trajectory
    # trajectory = STOMPUTILS.generate_bezier_trajectory(theta_s, theta_g, 100)
    trajectory = STOMPUTILS.generate_lerp_trajectory(theta_s, theta_g, N)
    stomp = STOMP(
        K=K,
        N=N,
        d=d,
        h=h,
        tolerance=tolerance,
        Rscale=Rscale,
        stddev=stddev,
        num_iterations=num_iterations,
        func_obstacle_cost=compute_traj_conecost_all,
        func_constraint_cost=STOMPUTILS.constraint_cost,
        func_torque_cost=STOMPUTILS.torque_cost,
        func_obstacle_cost_single=compute_traj_conecost_single,
        func_constraint_cost_single=STOMPUTILS.compute_trajectory_constraint_cost,
        func_torque_cost_single=STOMPUTILS.compute_trajectory_torque_cost,
        seed=9,
        print_debug=False,
        print_log=False,
    )
    trajectory_opt = stomp.optimize(trajectory)

    # tracking current mouse position
    mouse_pos = {"x": None, "y": None}
    mouse_pos_prev = {"x": None, "y": None}

    def track_mouse(event):
        if event.xdata is not None and event.ydata is not None:
            mouse_pos["x"] = event.xdata
            mouse_pos["y"] = event.ydata

    fig, ax = plt.subplots()
    ax.plot(theta_s[0], theta_s[1], "ro", label="Start")
    ax.plot(theta_g[0], theta_g[1], "bo", label="Goal")
    # ax.plot(
    #     trajectory[:, 0],
    #     trajectory[:, 1],
    #     "k--",
    #     linewidth=4,
    #     label="Original Trajectory",
    # )
    # (traj_line,) = ax.plot([], [], "r+", linewidth=4, label="Optimal Trajectory")
    circle = plt.Circle(obscxy, obsr, color="r", alpha=0.5)
    ax.add_patch(circle)

    ax.plot(
        trajectory_opt[:, 0],
        trajectory_opt[:, 1],
        "r-",
        linewidth=2,
        label="Optimized Trajectory",
    )
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_title("Trajectory Comparison")
    ax.set_xlabel("Theta 1")
    ax.set_ylabel("Theta 2")
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    plt.show()

    # trajectory = STOMPUTILS.generate_lerp_trajectory(theta_s, theta_g, N)
    # def loop():
    #     global theta_s, theta_g, trajectory, c

    #     if mouse_pos["x"] is not None and mouse_pos["y"] is not None:
    #         theta_g = np.array([mouse_pos["x"], mouse_pos["y"]])
    #         c = theta_g.copy()
    #         trajectory = STOMPUTILS.generate_lerp_trajectory(theta_s, theta_g, N)
    #         # reuse seed trajectory to make it more faster
    #         stomp = STOMP(
    #             K=K,
    #             N=N,
    #             d=d,
    #             h=h,
    #             tolerance=tolerance,
    #             Rscale=Rscale,
    #             stddev=stddev,
    #             num_iterations=num_iterations,
    #             func_obstacle_cost=compute_traj_conecost_all,
    #             func_constraint_cost=STOMPUTILS.constraint_cost,
    #             func_torque_cost=STOMPUTILS.torque_cost,
    #             func_obstacle_cost_single=compute_traj_conecost_single,
    #             func_constraint_cost_single=STOMPUTILS.compute_trajectory_constraint_cost,
    #             func_torque_cost_single=STOMPUTILS.compute_trajectory_torque_cost,
    #             seed=9,
    #             print_debug=False,
    #             print_log=False,
    #         )
    #         trajectory = stomp.optimize(trajectory)

    #         traj_line.set_data(trajectory[:, 0], trajectory[:, 1])

    #         fig.canvas.draw_idle()
    #         # Update previous position
    #         mouse_pos_prev["x"] = mouse_pos["x"]
    #         mouse_pos_prev["y"] = mouse_pos["y"]

    # fig.canvas.mpl_connect("motion_notify_event", track_mouse)
    # timer = fig.canvas.new_timer(interval=int(0.01 * 1000))
    # timer.add_callback(loop)
    # timer.start()
    # ax.set_xlim(xlim)
    # ax.set_ylim(ylim)
    # ax.set_title("Trajectory Comparison")
    # ax.set_xlabel("Theta 1")
    # ax.set_ylabel("Theta 2")
    # ax.set_aspect("equal", adjustable="box")
    # ax.legend()
    # plt.show()
