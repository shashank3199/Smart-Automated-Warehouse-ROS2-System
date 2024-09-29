import numpy as np
from scipy.linalg import logm, inv

class AXBsolver:
    """
    AXBsolver class solves the AX = XB problem for hand-eye calibration.
    Attributes:
    e_bh (numpy.ndarray): Array of base to hand transformations.
    e_sc (numpy.ndarray): Array of sensor to camera transformations.
    """

    def __init__(self, e_bh, e_sc):
        """
        Initialize the AXBsolver with base-hand and sensor-camera transformations.

        Parameters:
        e_bh (numpy.ndarray): Array of base to hand transformations.
        e_sc (numpy.ndarray): Array of sensor to camera transformations.
        """
        self.e_bh = e_bh
        self.e_sc = e_sc

    def quat2rotm(self, quat):
        """
        Convert a quaternion to a rotation matrix.

        Parameters:
        quat (numpy.ndarray): Quaternion array [w, x, y, z].

        Returns:
        numpy.ndarray: Corresponding rotation matrix.
        """
        # Normalize the quaternion
        q = quat / np.linalg.norm(quat)
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2 * (q2**2 + q3**2), 2 * (q1*q2 - q0*q3), 2 * (q1*q3 + q0*q2)],
            [2 * (q1*q2 + q0*q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2*q3 - q0*q1)],
            [2 * (q1*q3 - q0*q2), 2 * (q2*q3 + q0*q1), 1 - 2 * (q1**2 + q2**2)]
        ])

    def axxb(self):
        """
        Solve the AX = XB problem.

        Returns:
        numpy.ndarray: The 4x4 transformation matrix X.
        """
        # Number of transformations
        N = self.e_bh.shape[0]
        E_bh = []
        E_sc = []
        alphas = []
        betas = []

        RA_list = np.zeros((3, 3, N-1))
        tA_list = np.zeros((3, N-1))
        RB_list = np.zeros((3, 3, N-1))
        tB_list = np.zeros((3, N-1))

        # Convert transformations to matrices and calculate alphas and betas
        for i in range(N):
            t_bh = self.e_bh[i, :3]
            q_bh = self.e_bh[i, 6:7+1].tolist() + self.e_bh[i, 3:6].tolist()
            R_bh = self.quat2rotm(np.array(q_bh))
            E_bh.append(np.vstack((np.hstack((R_bh, t_bh.reshape(3, 1))), [0, 0, 0, 1])))

            t_sc = self.e_sc[i, :3]
            q_sc = self.e_sc[i, 6:7+1].tolist() + self.e_sc[i, 3:6].tolist()
            R_sc = self.quat2rotm(np.array(q_sc))
            E_sc.append(np.vstack((np.hstack((R_sc, t_sc.reshape(3, 1))), [0, 0, 0, 1])))

            if i == 0:
                continue

            A = inv(E_bh[0]).dot(E_bh[i])
            B = E_sc[0].dot(inv(E_sc[i]))

            RA_list[:, :, i-1] = A[:3, :3]
            tA_list[:, i-1] = A[:3, 3]

            RB_list[:, :, i-1] = B[:3, :3]
            tB_list[:, i-1] = B[:3, 3]

            alphahat = logm(A)
            betahat = logm(B)

            alpha = np.array([alphahat[2, 1], alphahat[0, 2], alphahat[1, 0]])
            beta = np.array([betahat[2, 1], betahat[0, 2], betahat[1, 0]])

            alphas.append(alpha)
            betas.append(beta)

        alphas = np.array(alphas).T
        betas = np.array(betas).T

        # Solve for Rx and tx
        Rx = self.solve_rx(alphas, betas)
        tx = self.solve_tx(RA_list, tA_list, RB_list, tB_list, Rx)

        # Construct the transformation matrix X
        X = np.vstack((np.hstack((Rx, tx.reshape(3, 1))), [0, 0, 0, 1]))
        return np.round(X, 6)

    def solve_rx(self, alphas, betas):
        """
        Solve for the rotation matrix Rx.

        Parameters:
        alphas (numpy.ndarray): Array of alpha vectors.
        betas (numpy.ndarray): Array of beta vectors.

        Returns:
        numpy.ndarray: The rotation matrix Rx.
        """
        N = alphas.shape[1]
        M = np.zeros((3, 3))

        # Construct matrix M from alphas and betas
        for i in range(N):
            M += np.outer(betas[:, i], alphas[:, i])

        # Perform Singular Value Decomposition (SVD) on M
        U, _, Vt = np.linalg.svd(M)

        # The rotation matrix Rx can be found as:
        Rx = U @ Vt

        # Ensure Rx is a proper rotation matrix (det(Rx) = 1)
        if np.linalg.det(Rx) < 0:
            U[:, -1] *= -1
            Rx = U @ Vt

        return Rx

    def solve_tx(self, RA, tA, RB, tB, RX):
        """
        Solve for the translation vector tx.

        Parameters:
        RA (numpy.ndarray): Rotation parts of the A matrices.
        tA (numpy.ndarray): Translation parts of the A matrices.
        RB (numpy.ndarray): Rotation parts of the B matrices.
        tB (numpy.ndarray): Translation parts of the B matrices.
        RX (numpy.ndarray): Rotation matrix Rx.

        Returns:
        numpy.ndarray: The translation vector tx.
        """
        N = tA.shape[1]
        left_matrix = np.zeros((3 * N, 3))
        right_matrix = np.zeros((3 * N, 1))

        # Construct the linear system to solve for tx
        for i in range(N):
            left_matrix[3*i:3*(i+1), :3] = RA[:, :, i] - np.eye(3)
            right_matrix[3*i:3*(i+1), 0] = RX @ tB[:, i] - tA[:, i]

        # Solve the linear system
        tx = np.linalg.lstsq(left_matrix, right_matrix, rcond=None)[0]
        return tx.flatten()
