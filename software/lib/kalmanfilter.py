from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
import matplotlib.pyplot as plt

class EKF:
    def __init__(self, *, dim_x: int=6, dim_z: int=1, dt: float=0.02, q_pos: float=0.1, q_vel: float=0.1, covar: float=5):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.ekf.P *= 5
        self.ekf.x = np.zeros((dim_x, dim_z))
        self.ekf = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)
    
    def state_trans_mat(self):
        self.ekf.F = np.block([
            [np.eye(3), self.dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)],
        ])
    
    def process_noise(self):
        self.ekf.Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel,
        ])

    # Prediction steps
    def predict_accel(self, accel: tuple[float, float, float]):
        ax, ay, az = accel

        B = np.block([
            [0.5 * self.dt**2 * np.eye(3)],
            [self.dt * np.eye(3)],
        ])

        self.ekf.x = self.ekf.F @ self.ekf.x + B @ np.array([[ax], [ay], [az]])
        self.ekf.P = self.ekf.F @ self.ekf.P @ self.ekf.F.T + self.ekf.Q
    
    def update_camera(self, z_cam):
        H = np.hstack((np.eye(3), np.zeros((3, 3))))

        R_cam = np.diag([0.05, 0.05, 0.1])

        y = z_cam.reshape((3, 1)) - H @ self.ekf.x
        S = H @ self.ekf.P @ H.T + R_cam
        K = self.ekf.P @ H.T @ np.linalg.inv(S)

        self.ekf.x += K @ y
        self.ekf.P = (np.eye(6) - K @ H) @ self.ekf.P

    def update_uwb(self, z_range):
        def h_uwb(x):
            px, py, pz= x[0, 0], x[1, 0], x[2, 0]
            return np.array([[np.sqrt(px**2 + py**2 + pz**2)]])

        def H_uwb(x):
            px, py, pz= x[0, 0], x[1, 0], x[2, 0]
            r = np.sqrt(px**2 + py**2 + pz**2)
            return np.array([[px/r, py/r, pz/r, 0, 0, 0]])

        H = H_uwb(self.ekf.x)
        z_pred = h_uwb(self.ekf.x)

        R_uwb = np.array([[0.04]])  # (20 cm)^2

        y = np.array([[z_range]]) - z_pred
        S = H @ self.ekf.P @ H.T + R_uwb
        K = self.ekf.P @ H.T @ np.linalg.inv(S)

        self.ekf.x += K @ y
        self.ekf.P = (np.eye(6) - K @ H) @ self.ekf.P

    # Pre-flight simulation
    def preflight_sim(self):
        true_pos = []
        est_pos = []

        for t in np.arange(0, 20, self.dt):
            # Simulate true motion
            px = 3* np.cos(0.2 * t)
            py = 3* np.sin(0.2 * t)
            pz = 1

            # Noisy camera
            z_cam = np.array([
                px + np.random.normal(0, 0.1),
                py + np.random.normal(0, 0.1),
                pz + np.random.normal(0, 0.1),
            ])
            
            self.predict_accel((0.0, 0.0, 0.0))
            self.update_camera(z_cam)

            true_pos.append([px, py])
            est_pos.append([self.ekf.x[0, 0], self.ekf.x[1, 0]])

        true_pos = np.array(true_pos)
        est_pos = np.array(est_pos)

        # Plot to show overlap (should see close overlap)
        # TODO: convert graphical information to numerical
        plt.plot(true_pos[:, 0], true_pos[:, 1])
        plt.plot(est_pos[:, 0], est_pos[:, 1])
        plt.legend(['True', 'EKF'])
        plt.show()

