import numpy as np
from icecream import ic

# TODO : Add buffer to save best noise trajectory for later use

class STOMP:

    def __init__(
        self,
        K,
        N,
        d,
        h,
        tolerance,
        Rscale,
        stddev,
        num_iterations,
        func_obstacle_cost,
        func_constraint_cost,
        func_torque_cost,
        func_obstacle_cost_single,
        func_constraint_cost_single,
        func_torque_cost_single,
        seed=9,
        print_debug=False,
        print_log=False,
    ):
        # hyper parameters
        self.K = K  # Number of trajectories
        self.N = N  # Number of time steps (waypoints)
        self.d = d  # Dimension of the joint space
        self.h = h  # sensitivity scale
        self.tolerance = tolerance  # terminate upon the value is below
        self.Rscale = Rscale  # scale R inverse
        self.stddev = stddev  # the degree of noise that can be applied to the joints ex: stddev = [0.05, 0.8, 0.8, 0.4, 0.4, 0.4] # 6 joints
        # Each value in this array is the amplitude of the noise applied to the joint at that position in the array.
        # For instace, the leftmost value in the array will be the value used to set the noise of the first joint of the robot (panda_joint1 in our case).
        # The dimensionality of this array should be equal to the number of joints in the planning group name.
        # Larger “stddev” values correspond to larger motions of the joints.
        self.num_iterations = num_iterations

        # cost computation
        self.func_obstacle_cost = func_obstacle_cost
        self.func_constraint_cost = func_constraint_cost
        self.func_torque_cost = func_torque_cost
        self.func_obstacle_cost_single = func_obstacle_cost_single
        self.func_constraint_cost_single = func_constraint_cost_single
        self.func_torque_cost_single = func_torque_cost_single

        # debug and log
        self.seed = seed
        self.print_debug = print_debug
        self.print_log = print_log

        self.cost_history = []

        np.random.seed(self.seed)

    def compute_finite_diff_acceleration_matrix_A(self):
        # diff_rule = [0, 0, -1, 1, 0, 0, 0]
        diff_rule = [0, 0, 1, -2, 1, 0, 0]  # start with -2
        # diff_rule = [0, 1, -2, 1, 0, 0, 0]  # start with the last 1
        # diff_rule = [0, 0, 0, 1, -2, 1, 0]  # start with the first 1
        half_length = len(diff_rule) // 2
        A = np.zeros([self.N, self.N])
        for i in range(0, self.N):
            for j in range(-half_length, half_length):
                index = i + j
                if index >= 0 and index < self.N:
                    A[i, index] = diff_rule[j + half_length]

        if self.print_debug:
            ic(A.shape)
            ic(A)
        return A

    def compute_finite_diff_acceleration_matrix_A_2(self):
        diff_rule = [0, 1, -2, 1, 0, 0, 0]  # start with the last 1
        half_length = len(diff_rule) // 2
        A = np.zeros([self.N + 2, self.N])
        for i in range(0, self.N + 2):
            for j in range(-half_length, half_length):
                index = i + j
                if index >= 0 and index < self.N:
                    A[i, index] = diff_rule[j + half_length]

        if self.print_debug:
            ic(A.shape)
            ic(A)
        return A

    def compute_R_R_inverse_R_inverse_scale(self, A):
        R = A.T @ A
        R_inv = np.linalg.inv(R)

        # scale covariance matrix, from stomp cpp code
        m = np.abs(R_inv).max()
        R_inv_scaled = self.Rscale * (R_inv / m)

        if self.print_debug:
            ic(R.shape)
            ic(R)
            ic(R_inv.shape)
            ic(R_inv)
            ic(R_inv_scaled.shape)
            ic(R_inv_scaled)

        return R, R_inv, R_inv_scaled

    def compute_M(self, R_inv):
        # M = R_inv with each column scaled such that the maximum element is 1/N
        max_element = R_inv.max(axis=0)
        scaletoval = 1.0 / self.N
        scale = scaletoval / max_element
        M = scale * R_inv

        # zeroing out first and last rows
        M[0] = 0.0
        M[0, 0] = 1.0
        M[-1] = 0.0
        M[-1, -1] = 1.0

        if self.print_debug:
            ic(M.shape)
            ic(M)

        return M

    def generate_noisy_trajectories(self, R_inv, trajectory):
        # generate raw noise
        epsilon = np.random.multivariate_normal(mean=np.zeros(self.N), cov=R_inv, size=(self.K, self.d))  # K, d, N
        epsilon = self.stddev * epsilon.transpose(0, 2, 1)  # K, N, d scale with std per each joint

        # zeroing out the start and end noise values
        epsilon[:, 0, :] = 0.0
        epsilon[:, -1, :] = 0.0

        # create K noisy trajectories add each generated noise to trajectory
        noisy_trajectories = epsilon + trajectory

        if self.print_debug:
            ic(epsilon.shape)
            ic(epsilon)
            ic(noisy_trajectories.shape)
            ic(noisy_trajectories)

        return noisy_trajectories, epsilon

    def compute_cost_S(self, noisy_trajectories):
        # a point in trajectory must have aleast some concept of cost.
        # that cost will be use to compute probability and later determine the update value.
        # input : noisy_trajectories = K, N, d
        # output from each cost computation is K, N, d
        qo = self.func_obstacle_cost(noisy_trajectories)
        qc = self.func_constraint_cost(noisy_trajectories)
        qt = self.func_torque_cost(noisy_trajectories)
        S = qo + qc + qt

        if self.print_debug:
            ic(S.shape)
            ic(S)

        return S

    def compute_probability_P(self, S):
        # In Eq 11
        Smin = S.min(axis=0, keepdims=True)  # K trajectory wise
        Smax = S.max(axis=0, keepdims=True)  # K trajectory wise

        numerator = np.exp(-self.h * (S - Smin) / (Smax - Smin))  # inverse softmax
        denominator = np.sum(numerator, axis=0, keepdims=True)  # sum over k trajectories
        P = numerator / denominator
        P[:, 0, :] = 0.0  # avoid first element 0 to NaN
        P[:, -1, :] = 0.0  # zero out last element

        if self.print_debug:
            ic(Smin.shape)
            ic(Smin)
            ic(Smax.shape)
            ic(Smax)
            ic(P.shape)
            ic(P)

        return P

    def compute_noisy_update(self, P, epsilon):
        # a probability-weighted (convex combination) of noisy parameter from that time step
        P_epsilon = P * epsilon
        delta_trajectory_noise = P_epsilon.sum(axis=0)
        # delta_trajectory_noise[-1] = 0.0  # pin the last node exactly at the goal state, we don't want to update it

        if self.print_debug:
            ic(P_epsilon.shape)
            ic(P_epsilon)
            ic(delta_trajectory_noise.shape)
            ic(delta_trajectory_noise)

        return delta_trajectory_noise

    def smooth_noisy(self, M, delta_trajectory_noise):
        # compute update value scaling ensures that no updated parameter exceeds the range that was explored in the noisy trajectories.
        # ensures that the updated trajectory remains smooth.
        # it is essentially a projection onto the basis vectors of Rinverse
        delta_trajectory_smoothed = M @ delta_trajectory_noise

        if self.print_debug:
            ic(delta_trajectory_smoothed.shape)
            ic(delta_trajectory_smoothed)

        return delta_trajectory_smoothed

    def update_trajectory(self, trajectory, delta_trajectory_smoothed):
        trajectory = trajectory + delta_trajectory_smoothed  # column-wise addition

        if self.print_debug:
            ic(trajectory.shape)
            ic(trajectory)

        return trajectory

    def unupdate_trajectory(self, trajectory, delta_trajectory_smoothed):
        trajectory = trajectory - delta_trajectory_smoothed  # column-wise subtraction

        if self.print_debug:
            ic(trajectory.shape)
            ic(trajectory)

        return trajectory

    def compute_trajectory_distance_cost(self, trajectory):
        # path lenght cost
        diffs = np.diff(trajectory, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)

    def compute_trajectory_total_cost_Q(self, trajectory, R):
        smoothness_cost = 0.5 * np.trace(trajectory.T @ R @ trajectory)
        trajectory_cost = self.func_obstacle_cost_single(trajectory) + self.func_constraint_cost_single(trajectory) + self.func_torque_cost_single(trajectory) + self.compute_trajectory_distance_cost(trajectory)
        totalcost = trajectory_cost + smoothness_cost

        if self.print_debug:
            ic(smoothness_cost)
            ic(trajectory_cost)
            ic(totalcost)

        return totalcost

    def optimize(self, trajectory):
        ic("Computing STOMP Optimization")

        # we must have index [0,0] and [-1,-1] equal to 1. otherwise, it is gonna be look weird
        # A = self.compute_finite_diff_acceleration_matrix_A_2()
        A = self.compute_finite_diff_acceleration_matrix_A()

        trajectory_opt = trajectory.copy()

        R, R_inv, R_inv_scaled = self.compute_R_R_inverse_R_inverse_scale(A)
        M = self.compute_M(R_inv_scaled)
        cost = self.compute_trajectory_total_cost_Q(trajectory_opt, R)

        self.cost_history.clear()
        for iters in range(self.num_iterations):
            noisy_trajectories, epsilon = self.generate_noisy_trajectories(R_inv_scaled, trajectory_opt)

            S = self.compute_cost_S(noisy_trajectories)
            P = self.compute_probability_P(S)

            delta_trajectory_noise = self.compute_noisy_update(P, epsilon)
            delta_trajectory_smoothed = self.smooth_noisy(M, delta_trajectory_noise)

            trajectory_opt = self.update_trajectory(trajectory_opt, delta_trajectory_smoothed)

            current_cost = self.compute_trajectory_total_cost_Q(trajectory_opt, R)

            if current_cost < cost:  # check whether to keep the updated trajectory or revert back
                cost = current_cost
            else:  # if it become worst, go back to the previous trajectory
                trajectory_opt = self.unupdate_trajectory(trajectory_opt, delta_trajectory_smoothed)

            self.cost_history.append(cost)

        return trajectory_opt


class STOMPUTILS:

    @staticmethod
    def generate_bezier_trajectory(theta_s, theta_g, N):
        t = np.linspace(0, 1, N)
        P0 = theta_s.reshape(2, 1)
        P1 = np.array([-3, 3]).reshape(2, 1)
        P2 = theta_g.reshape(2, 1)
        trajectory = ((1 - t) ** 2) * P0 + (2 * (1 - t) * t) * P1 + (t**2) * P2
        return trajectory.T

    @staticmethod
    def generate_lerp_trajectory(theta_s, theta_g, N):
        trajectory = np.linspace(theta_s, theta_g, num=N, endpoint=True)
        return trajectory

    @staticmethod
    def add_uniform_noise_to_trajectory(trajectory, scale=0.05):
        noise = scale * np.random.uniform(-np.pi, np.pi, size=trajectory.shape)
        trajectory = trajectory + noise
        return trajectory

    @staticmethod
    def obstacle_cost(noisy_trajectories):
        # input : noisy_trajectories = K, N, d
        # output : costfully = K, N, d
        obscost = np.zeros((noisy_trajectories.shape))
        return obscost

    @staticmethod
    def constraint_cost(noisy_trajectories):
        # input : noisy_trajectories = K, N, d
        # output : costfully = K, N, d
        # constraint cost is joint together
        conscost = np.zeros((noisy_trajectories.shape))
        return conscost

    @staticmethod
    def torque_cost(noisy_trajectories):
        # input : noisy_trajectories = K, N, d
        # output : costfully = K, N, d
        # torque cost is individual joint
        # change to joint displacement instead. because I dont have a dynamic model.
        # torqcost = np.zeros(noisy_trajectories.shape))
        cost = np.diff(noisy_trajectories, axis=1, prepend=0.0)
        cost[:, 0, :] = 0.0
        torqcost = np.abs(cost)
        return torqcost

    @staticmethod
    def compute_trajectory_obstacle_cost(trajectory):
        return 0.0

    @staticmethod
    def compute_trajectory_constraint_cost(trajectory):
        return 0.0

    @staticmethod
    def compute_trajectory_torque_cost(trajectory):
        return 0.0


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    K = 50
    N = 70
    d = 2
    h = 10
    tolerance = 0.1
    Rscale = 10
    stddev = [0.095] * 2
    num_iterations = 50

    # environment
    theta_s = np.array([-2.0, -2.0])
    theta_g = np.array([2.0, 2.0])
    xlim = [-np.pi, np.pi]
    ylim = [-np.pi, np.pi]
    times = np.linspace(0, N, num=N)

    # generate trajectory
    trajectory = STOMPUTILS.generate_bezier_trajectory(theta_s, theta_g, N)
    # trajectory = STOMPUTILS.generate_lerp_trajectory(theta_s, theta_g, N)
    stomp = STOMP(
        K=K,
        N=N,
        d=d,
        h=h,
        tolerance=tolerance,
        Rscale=Rscale,
        stddev=stddev,
        num_iterations=num_iterations,
        func_obstacle_cost=STOMPUTILS.obstacle_cost,
        func_constraint_cost=STOMPUTILS.constraint_cost,
        func_torque_cost=STOMPUTILS.torque_cost,
        func_obstacle_cost_single=STOMPUTILS.compute_trajectory_obstacle_cost,
        func_constraint_cost_single=STOMPUTILS.compute_trajectory_constraint_cost,
        func_torque_cost_single=STOMPUTILS.compute_trajectory_torque_cost,
        seed=9,
        print_debug=False,
        print_log=False,
    )
    trajectory_opt = stomp.optimize(trajectory)

    # plot comparison
    if True:
        fig5, ax5 = plt.subplots()
        ax5.plot(theta_s[0], theta_s[1], "ro", label="Start")
        ax5.plot(theta_g[0], theta_g[1], "bo", label="Goal")
        ax5.plot(trajectory[:, 0], trajectory[:, 1], "k--", linewidth=4, label="Original Trajectory")
        ax5.plot(trajectory_opt[:, 0], trajectory_opt[:, 1], "r+", linewidth=4, label="Optimal Trajectory")

        for point in trajectory_opt:
            circle1 = plt.Circle((point[0], point[1]), radius=0.2, color="r", fill=False, alpha=0.5)
            ax5.add_patch(circle1)

        ax5.set_xlim(xlim)
        ax5.set_ylim(ylim)
        ax5.set_title("Trajectory Comparison")
        ax5.set_xlabel("Theta 1")
        ax5.set_ylabel("Theta 2")
        ax5.set_aspect("equal", adjustable="box")
        ax5.legend()

        if True:
            fig55, axs55 = plt.subplots(d, 1, sharex=True)
            for i in range(d):
                axs55[i].plot(times, trajectory[..., i], "k--", label=f"Original {i+1}")
                axs55[i].plot(times, trajectory_opt[..., i], "r+", label=f"Optimal {i+1}")
                axs55[i].plot(times, [-np.pi] * len(times), "m", label=f"Joint Min : {-np.pi:.3f}")
                axs55[i].plot(times, [np.pi] * len(times), "c", label=f"Joint Max : {np.pi:.3f}")
                axs55[i].set_ylabel(f"theta {i+1}")
                axs55[i].set_xlim(times[0], times[-1])
                axs55[i].legend(loc="upper right")
                axs55[i].grid(True)
            axs55[-1].set_xlabel("Time")

    plt.show()
