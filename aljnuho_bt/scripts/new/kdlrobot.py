import kdlurdf as kdlurdf
import PyKDL as kdl


class KDLChain:

    def __init__(self, tree, root, tip):
        self.tree = tree
        self.root = root
        self.tip = tip
        self.chain = self.tree.getChain(self.root, self.tip)
        self.jointnum = self.chain.getNrOfJoints()

        self._fk_pos_kdl = kdl.ChainFkSolverPos_recursive(self.chain)  # Compute Joint Positions to End Effector Pose
        self._fk_vel_kdl = kdl.ChainFkSolverVel_recursive(self.chain)  # Compute Joint Velocities to End Effector Velocities

        self._ik_vel_kdl = kdl.ChainIkSolverVel_pinv(self.chain)  # Compute End Effector Velocities to Joint Velocities
        self._ik_pos_kdl = kdl.ChainIkSolverPos_NR(self.chain, self._fk_pos_kdl, self._ik_vel_kdl)  # Compute End Effector Pose to Joint Positions

        self._jac_kdl = kdl.ChainJntToJacSolver(self.chain)

    @classmethod
    def from_string(cls, urdf_string, root, tip):
        done, tree = kdlurdf.treeFromString(urdf_string)
        if not done:
            raise ValueError("Failed to create KDL tree from URDF string.")

        return cls(tree, root, tip)

    @classmethod
    def from_file(cls, urdf_file, root, tip):
        done, tree = kdlurdf.treeFromFile(urdf_file)
        if not done:
            raise ValueError("Failed to create KDL tree from URDF file.")

        return cls(tree, root, tip)

    def forward_kinematics(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_array[i] = joint_positions[i]

        end_effector_frame = kdl.Frame()
        self._fk_pos_kdl.JntToCart(joint_array, end_effector_frame)

        return end_effector_frame

    def forward_kinematics_velocity(self, joint_positions, joint_velocities):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")
        if len(joint_velocities) != self.jointnum:
            raise ValueError("Joint velocities length does not match the number of joints in the chain.")

        joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_array[i] = joint_positions[i]

        joint_velocity_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_velocity_array[i] = joint_velocities[i]

        end_effector_frame = kdl.Frame()
        self._fk_vel_kdl.JntToCart(joint_array, end_effector_frame, joint_velocity_array)

        return end_effector_frame

    def inverse_kinematics(self, target_frame, initial_joint_positions):
        if len(initial_joint_positions) != self.jointnum:
            raise ValueError("Initial joint positions length does not match the number of joints in the chain.")

        initial_joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            initial_joint_array[i] = initial_joint_positions[i]

        final_joint_positions = kdl.JntArray(self.jointnum)
        self._ik_pos_kdl.CartToJnt(initial_joint_array, target_frame, final_joint_positions)

        return final_joint_positions

    def inverse_kinematics_velocity(self, target_frame, joint_velocities):
        if len(joint_velocities) != self.jointnum:
            raise ValueError("Joint velocities length does not match the number of joints in the chain.")

        joint_velocity_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_velocity_array[i] = joint_velocities[i]

        end_effector_frame = kdl.Frame()
        self._ik_vel_kdl.CartToJnt(joint_velocity_array, target_frame, end_effector_frame)

        return end_effector_frame

    def jacobian(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        j_kdl = kdl.Jacobian(self.jointnum)
        q_kdl = self.make_joint_array(joint_positions)
        self._jac_kdl.JntToJac(q_kdl, j_kdl)

        return j_kdl

    def make_joint_array(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        joint_array = kdl.JntArray(self.jointnum)
        for i in range(self.jointnum):
            joint_array[i] = joint_positions[i]

        return joint_array

    def make_frame(self, position, orientation):
        if len(position) != 3:
            raise ValueError("Position length must be 3 (x, y, z).")
        if len(orientation) != 4:
            raise ValueError("Orientation length must be 4 (x, y, z, w).")

        frame = kdl.Frame()
        frame.p = kdl.Vector(
            position[0],
            position[1],
            position[2],
        )
        frame.M = kdl.Rotation.Quaternion(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        )

        return frame

    def make_joint_list(self, joint_positions):
        if len(joint_positions) != self.jointnum:
            raise ValueError("Joint positions length does not match the number of joints in the chain.")

        joint_list = []
        for i in range(self.jointnum):
            joint_list.append(joint_positions[i])

        return joint_list

    def make_position_orientation(self, end_effector_frame):
        position = [
            end_effector_frame.p[0],
            end_effector_frame.p[1],
            end_effector_frame.p[2],
        ]
        orientation = [
            end_effector_frame.M.GetQuaternion()[0],
            end_effector_frame.M.GetQuaternion()[1],
            end_effector_frame.M.GetQuaternion()[2],
            end_effector_frame.M.GetQuaternion()[3],
        ]

        return position, orientation
