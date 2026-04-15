import numpy as np
from vpsto.vpsto import VPSTO
from vpsto.vptraj import VPTraj
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Polygon, MultiPolygon, LineString


q0 = np.array([-0.5, 0.5])  # Initial position
qg = np.array([0.5, 0.5])  # Goal position
qangle = -0.5
dq0 = np.array([0, 0])  # Initial velocity
dqg = np.array([0, 0])  # Goal velocity
bounds = np.array([[-1, 1], [-1, 1]])  # Bounds on position

R = 1e1  # Penalty on control effort (acceleration)
Q_min = 1e0  # Minimum penalty on control error (position)
Q_max = 1e3  # Maximum penalty on control error (position)
factor_Q_min = 1e-1
factor_Q_max = 1e1

N_via = 4  # Number of via points
N_candidates = 1000  # Number of candidates to sample
N_eval = 25  # Number of pos,vel,acc samples to evaluate along each candidate
vel_lim = 0.2  # Velocity limit (m/s in each dimension)
acc_lim = 1  # Acceleration limit (m/s^2 in each dimension)

num_obstacles = 10  # Number of obstacles
robot_radius = 0.1  # Radius of robot (m)
obstacle_radius = 0.1  # Radius of obstacle (m)

dt_control = 0.05  # Time step for control (s)


class Controller:

    def __init__(self):
        self.vptraj = VPTraj(
            ndof=2, N_eval=N_eval, N_via=N_via, vel_lim=vel_lim, acc_lim=acc_lim
        )
        self.vptraj_idle = VPTraj(
            ndof=2, N_eval=N_eval, N_via=1, vel_lim=vel_lim, acc_lim=acc_lim
        )

        self.bounds = bounds  # Bounds on position
        self.qg = qg.copy()  # Filtered goal position
        self.qangle = qangle  # Goal angle (not used in this 2D example)
        self.dt_control = dt_control  # Time step for control
        self.N_candidates = (
            N_candidates  # Number of candidate trajectories to sample
        )
        self.R = R  # Acceleration penalty at sampling stage (not considered in loss function)
        self.Q = Q_max  # Bias towards trajectories that go closer to the goal (updated at each iteration)
        self.acc_lim = (
            acc_lim  # Acceleration limit (used for constructing idle trajectory)
        )

        self.p_next = None  # Candidate trajectory parameter for next iteration
        self.T_next = None  # Candidate trajectory duration for next iteration
        self.samples_log = []  # Log of all candidate trajectories
        self.samples_loss_log = []  # Log of all candidate trajectories' losses
        self.sol_log = []  # Log of all solutions
        self.Q_log = []  # Log of all Q values
        self.goal_alpha = 0.2  # Goal smoothing factor
        self.goal_deadband = 0.01  # Ignore tiny mouse jitter

    def update_goal(self, qg_live):
        qg_live = np.asarray(qg_live, dtype=float)
        if np.linalg.norm(qg_live - self.qg) < self.goal_deadband:
            return
        self.qg = (1.0 - self.goal_alpha) * self.qg + self.goal_alpha * qg_live

    # Loss function for the candidate trajectories
    def loss_fn(self, q, dq, ddq, T):
        # Penalize trajectory duration
        duration_cost = T
        # Penalize control error
        qT = q[:, -1]  # Final position of all candidates
        terminal_cost = 1e3 * np.sum((qT - self.qg) ** 2, axis=1)  # Squared error
        # Penalize position limit violations (soft constraint)
        num_violations = np.sum(
            (q < self.bounds[:, 0] + robot_radius)
            | (q > self.bounds[:, 1] - robot_radius),
            axis=(1, 2),
        )
        limit_violation_cost = 1e6 * num_violations

        v = q[:, -1] - q[:, -2]  # (P,2)
        v_norm = v / (np.linalg.norm(v, axis=1, keepdims=True) + 1e-8)
        d_star = np.array([np.cos(self.qangle), np.sin(self.qangle)])
        dot = np.sum(v_norm * d_star, axis=1)
        L_angle = 1 - dot
        w_angle = 5.0
        angle_cost = 1e2 * w_angle * L_angle
        return terminal_cost + limit_violation_cost + duration_cost + angle_cost

    # Control function that samples candidate trajectories and chooses the best one
    def predictive_sampling(self, q, dq):
        # Compute how many constraint violations have occurred in the previous iteration
        if len(self.samples_loss_log) > 0:
            num_violations = np.sum(self.samples_loss_log[-1] > 1e6)
        else:
            num_violations = 0
        # Compute how strong the bias should be towards the goal:
        # - If there have been few constraint violations, increase the bias
        # - If there have been many constraint violations, decrease the bias
        self.Q *= np.clip(
            np.exp(-3 * (num_violations / self.N_candidates - 0.5)),
            factor_Q_min,
            factor_Q_max,
        )
        self.Q = np.clip(self.Q, Q_min, Q_max)
        self.Q_log.append(self.Q)

        # Sample candidate trajectories, compute their loss and return the best one
        pos, vel, acc, p, T = self.vptraj.sample_trajectories(
            self.N_candidates,
            q,
            dq0=dq,
            qT=self.qg,
            dqT=np.zeros_like(dq),
            Q=self.Q,
            R=self.R,
        )
        loss = self.loss_fn(pos[:, 1:], vel[:, 1:], acc[:, 1:], T)
        self.samples_log.append(pos)
        self.samples_loss_log.append(loss)
        i_best = np.argmin(loss)
        return p[i_best], loss[i_best], T[i_best]

    # Control function that reuses the previous solution
    def previous_sol(self, q, dq):
        if self.p_next is None:
            return None, np.inf, 0
        # Compute trajectory with previous solution
        pos, vel, acc = self.vptraj.get_trajectory(
            self.p_next, q, dq0=dq, dqT=np.zeros_like(dq), T=self.T_next
        )
        loss = self.loss_fn(pos[:, 1:], vel[:, 1:], acc[:, 1:], [self.T_next])[0]
        return self.p_next, loss, self.T_next

    # Control function that makes the robot come to stop as fast as possible
    def idle(self, q, dq):
        # Assuming constant acceleration
        T_idle = np.max(np.abs(dq) / self.acc_lim)
        # Compute the stopping position
        q_idle = q + 0.5 * dq * T_idle
        pos, vel, acc = self.vptraj_idle.get_trajectory(
            q_idle, q, dq0=dq, dqT=np.zeros_like(dq), T=T_idle
        )
        loss = self.loss_fn(pos[:, 1:], vel[:, 1:], acc[:, 1:], [T_idle])[0]
        # # Check obstacle collision also for two seconds ahead
        # q_list = np.vstack((q, q_idle))
        # loss += 1e6 * np.sum(
        #     [obs.predict_collision(q_list, T_idle + 2) for obs in self.obstacles]
        # )
        t_next = np.min([T_idle, self.dt_control])
        if t_next < dt_control:
            return q_idle, np.zeros_like(dq), loss, 0
        q_next, dq_next, _ = self.vptraj_idle.get_trajectory_at_time(
            t_next, q_idle, q, dq0=dq, dqT=np.zeros_like(dq), T=T_idle
        )
        return q_next.squeeze(), dq_next.squeeze(), loss, T_idle

    def control(self, q, dq):
        # Compute idle trajectory
        q_next_idle, dq_next_idle, loss_idle, T_idle = self.idle(q, dq)
        # Compute trajectory with previous solution
        p_prev, loss_prev, T_prev = self.previous_sol(q, dq)
        # Compute trajectory with predictive sampling
        p_samp, loss_samp, T_samp = self.predictive_sampling(q, dq)

        # Choose the best trajectory
        if loss_idle < loss_prev and loss_idle < loss_samp:
            print("Idle:", loss_idle, end="\r")
            self.p_next = None
            self.T_next = None
            loss_best = loss_idle
            self.sol_log.append(np.vstack((q, q_next_idle)))
            return q_next_idle, dq_next_idle
        elif loss_prev < loss_samp:
            print("Previous:", loss_prev, end="\r")
            p_best = p_prev
            T_best = T_prev
            loss_best = loss_prev
        else:
            print("Sampling:", loss_samp, end="\r")
            p_best = p_samp
            T_best = T_samp
            loss_best = loss_samp
        if T_best < self.dt_control:
            self.p_next = None
            self.T_next = None
            self.sol_log.append(np.vstack((q, q)))
            return p_best[-self.vptraj.ndof :], np.zeros_like(dq)
        # Compute trajectory parameter and duration for next time step
        self.T_next = T_best - self.dt_control
        t_next = (
            np.linspace(0, self.T_next, self.vptraj.N_via + 1) + self.dt_control
        )
        q_next, dq_next, _ = self.vptraj.get_trajectory_at_time(
            t_next, p_best, q, dq0=dq, dqT=np.zeros_like(dq), T=T_best
        )
        self.p_next = q_next[1:].flatten()
        self.sol_log.append(q_next)

        return q_next[0], dq_next[0]


control = Controller()


# tracking current mouse position
mouse_pos = {"x": None, "y": None}
mouse_pos_prev = {"x": None, "y": None}


def track_mouse(event):
    if event.xdata is not None and event.ydata is not None:
        mouse_pos["x"] = event.xdata
        mouse_pos["y"] = event.ydata


fig, ax = plt.subplots()
ax.set_xlim(bounds[0, 0], bounds[0, 1])
ax.set_ylim(bounds[1, 0], bounds[1, 1])
ax.set_aspect("equal")
(follower_line,) = ax.plot([], [], "b*", label="Follower Trajectory")
(desired_line,) = ax.plot([], [], "ro", label="Desired Trajectory")
(angle_line,) = ax.plot([], [], "g-", label="Orientation")


def loop():
    global q0, dq0
    if mouse_pos["x"] is not None and mouse_pos["y"] is not None:

        xc = mouse_pos["x"]
        yc = mouse_pos["y"]

        # Update goal from live mouse position and keep it inside workspace bounds.
        control.update_goal(
            np.clip(np.array([xc, yc]), control.bounds[:, 0], control.bounds[:, 1])
        )

        # Calculate velocity from previous measurement
        if mouse_pos_prev["x"] is not None:
            vxc = (xc - mouse_pos_prev["x"]) / 0.01
            vyc = (yc - mouse_pos_prev["y"]) / 0.01
        else:
            vxc = 0
            vyc = 0

        q0, dq0 = control.control(q0, dq0)
        xsim = q0[0]
        ysim = q0[1]

        follower_line.set_data([xc], [yc])
        desired_line.set_data([xsim], [ysim])
        angle_line.set_data(
            [xc, xc + 0.2 * np.cos(control.qangle)],
            [yc, yc + 0.2 * np.sin(control.qangle)],
        )
        fig.canvas.draw_idle()

        # Update previous position
        mouse_pos_prev["x"] = mouse_pos["x"]
        mouse_pos_prev["y"] = mouse_pos["y"]


fig.canvas.mpl_connect("motion_notify_event", track_mouse)
timer = fig.canvas.new_timer(interval=int(0.01 * 1000))
timer.add_callback(loop)
timer.start()
plt.show()
