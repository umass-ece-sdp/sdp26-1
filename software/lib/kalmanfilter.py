from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
import matplotlib.pyplot as plt

# TODO: Add functionality to change covariance (R) values mid-flight (if needed)
    # - May need to send lots of UWB measurements (~500) to EKF before flight
# TODO: Process noise (Q) depending on filter lag/noise

class EKF:
    """
    Extended Kalman Filter for tracking a moving ground target from a drone.

    Sensor fusion sources:
      - Target IMU (Adafruit MPU-6050): accelerometer readings used as a
        control input to propagate the predicted state forward in time.
      - Drone camera (fiducial): provides a noisy but direct
        observation of the target's 3-D position in the drone's camera frame.
      - UWB ranging (drone tag <-> target anchor): provides a scalar
        line-of-sight distance measurement. This is the nonlinear part of the
        EKF — the range function h(x) = ||p|| is not linear in the state.

    State vector  x  (6 * 1):
        x = [px, py, pz, vx, vy, vz]^T
        where (px, py, pz) is the target's position relative to the drone
        and   (vx, vy, vz) is the target's velocity in that same frame.

    Units: meters and seconds throughout.

    Example usage:
    ```python
        from kalmanfilter import EKF
        ekf = EKF()
        while True: # drone control loop
            ekf.predict_accel(accel_values)
            ekf.update_camera(aruco_estimate_from_opencv)
            ekf.update_uwb(z_range)
            commands = ekf.filter_output()
            tello.send_rc_control(
                commands['left_right_velocity'],
                commands['forward_backward_velocity'],
                commands['up_down_velocity'],
                commands['yaw_velocity'],
            )
            tello.set_speed(commands['speed])
    ```

    Parameters:
        dim_x (int, default=6): State dimension [px, py, pz, vx, vy, vz]
        dim_z (int, default=1):  Scalar measurement dimension (UWB range)
        dt (float, default=0.02):  Prediction time step in seconds (50 Hz IMU)
        q_pos (float, default=0.1): Process noise variance for position states (m^2)
        q_vel (float, default=0.1): Process noise variance for velocity states ((m/s)^2)
        covar (float, default=5): Initial state covariance scale (higher = more uncertainty)
        r_desired (float, default=3): Desired distance between target and drone (meters)
    """

    def __init__(
        self,
        *,
        dim_x: int = 6,
        dim_z: int = 1,
        dt: float = 0.02,
        q_pos: float = 0.1,
        q_vel: float = 0.1,
        covar: float = 5,
        r_desired: float=10
    ) -> None:
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_desired = r_desired
        self.ekf = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)

        # Scale the initial error covariance matrix P (identity by default)
        # by `covar`.  A larger value means we start with low confidence in
        # the initial state estimate and let early measurements correct it
        # quickly.
        self.ekf.P *= covar

        # Initialise the state vector to zero — the filter has no prior
        # knowledge of where the target is before the first measurements.
        self.ekf.x = np.zeros((dim_x, dim_z))

        # Process states and noise so EKF is ready to use
        self.state_trans_mat()
        self.process_noise()

    def state_trans_mat(self) -> None:
        """
        Build and store the state transition (process) matrix F.

        Assumes a constant-velocity kinematic model integrated over one
        time step dt:
            p_{k+1} = p_k + dt * v_k
            v_{k+1} = v_k

        In block form:
            F = | I   dt*I |
                | 0      I |

        where I is the 3*3 identity matrix and 0 is the 3*3 zero matrix.
        This gives a 6*6 matrix that maps [p, v] -> [p + dt*v, v].
        """

        self.ekf.F = np.block([
            [np.eye(3), self.dt * np.eye(3)],
            [np.zeros((3, 3)), np.eye(3)],
        ])

    def process_noise(self) -> None:
        """
        Build and store the process noise covariance matrix Q.

        Q is a diagonal matrix whose entries represent how much the true
        state is expected to deviate from the constant-velocity model
        between time steps (e.g. due to wind, target manoeuvres, etc.).

            Q = diag(q_pos, q_pos, q_pos, q_vel, q_vel, q_vel)

        Larger values allow the filter to track faster, more erratic motion
        at the cost of noisier estimates.
        """

        self.ekf.Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel,
        ])

    def predict_accel(self, accel: tuple[float, float, float]) -> None:
        """
        Propagate the state estimate forward by one time step using an
        accelerometer reading as a control input.

        The target's IMU (MPU-6050) supplies (ax, ay, az) in m/s^2.
        These accelerations are integrated twice (for position) and once
        (for velocity) to form the control-input effect matrix B:

            B = | 0.5*dt^2*I |
                |    dt*I   |

        State prediction (prior estimate):
            x⁻ = F * x + B * u,   u = [ax, ay, az]^T

        Covariance prediction:
            P⁻ = F * P * F^T + Q

        Args:
            accel: (ax, ay, az) — target accelerometer reading in m/s^2.
        """

        ax, ay, az = accel

        # Control-input matrix B maps acceleration vector to change in state.
        # Upper block: position contribution (1/2 * dt^2 * a) from kinematics
        # Lower block: velocity contribution (dt * a)
        B = np.block([
            [0.5 * self.dt**2 * np.eye(3)],
            [self.dt * np.eye(3)],
        ])

        # --- State prediction ---
        # x⁻ = F*x + B*u
        self.ekf.x = self.ekf.F @ self.ekf.x + B @ np.array([[ax], [ay], [az]])

        # --- Covariance prediction ---
        # P⁻ = F*P*F^T + Q
        self.ekf.P = self.ekf.F @ self.ekf.P @ self.ekf.F.T + self.ekf.Q

    def update_camera(self, z_cam: np.ndarray) -> None:
        """
        Correct the state estimate using a fiducial position
        measured by the drone's camera.

        The camera directly observes the target's 3-D position in the drone
        frame, so the measurement model is linear:
            z = H * x + noise,   H = [I | 0]  (extract position from state)

        Because H is linear, this step is an ordinary Kalman update (no
        linearisation needed), but it is implemented here manually for
        consistency with the UWB nonlinear update.

        Measurement noise covariance R_cam:
            R_cam = diag(0.05, 0.05, 0.1)  [m^2]
        Larger z-variance (0.10) reflects reduced depth precision from the
        camera compared with lateral (x, y) precision (0.05).

        Standard Kalman update equations:
            Innovation:       y = z - H * x⁻
            Innovation cov:   S = H * P⁻ * H^T + R
            Kalman gain:      K = P⁻ * H^T * S⁻¹
            State update:     x = x⁻ + K * y
            Cov update:       P = (I - K*H) * P⁻

        Args:
            z_cam: (3,) or (3,1) array — [px, py, pz] measured by the camera.
        """
        # H extracts only the position states [px, py, pz] from the 6-D state.
        # Shape: 3×6
        H = np.hstack((np.eye(3), np.zeros((3, 3))))

        # Camera measurement noise (position uncertainty in meters^2).
        # x/y lateral noise is tighter (0.05 m^2) than depth z noise (0.10 m^2).
        R_cam = np.diag([0.05, 0.05, 0.1])

        # Innovation: difference between the actual camera measurement and
        # what the current state predicts the camera should see.
        y = z_cam.reshape((3, 1)) - H @ self.ekf.x

        # Innovation covariance: combines prediction uncertainty (H*P*H^T)
        # with sensor noise (R_cam).
        S = H @ self.ekf.P @ H.T + R_cam

        # Kalman gain: weights how much to trust the measurement vs. prediction.
        # K → 0 means trust the prediction; K → H⁻¹ means trust the measurement.
        K = self.ekf.P @ H.T @ np.linalg.inv(S)

        # Posterior state and covariance updates.
        self.ekf.x += K @ y
        self.ekf.P = (np.eye(6) - K @ H) @ self.ekf.P

    def update_uwb(self, z_range: float) -> None:
        """
        Correct the state estimate using a UWB range measurement.

        The UWB module measures the scalar Euclidean distance between the
        drone's tag and the target's anchor.  This is a nonlinear function
        of the position states:

            h(x) = ||p|| = sqrt(px^2 + py^2 + pz^2)

        Because h is nonlinear the EKF linearises it at the current state
        estimate by computing its Jacobian H_uwb (1*6 row vector):

            H_uwb = ∂h/∂x = [px/r, py/r, pz/r, 0, 0, 0]

        where r = h(x).  The velocity states have no direct effect on range
        so their partial derivatives are zero.

        Measurement noise covariance R_uwb:
            R_uwb = [[0.04]]   →  sigma ≈ 0.20 m  (±20 cm UWB ranging error)

        EKF update equations (same structure as linear KF but using the
        linearised H in place of a constant matrix):
            Innovation:       y = z - h(x⁻)
            Innovation cov:   S = H * P⁻ * H^T + R
            Kalman gain:      K = P⁻ * H^T * S⁻¹
            State update:     x = x⁻ + K * y
            Cov update:       P = (I - K*H) * P⁻

        Args:
            z_range: scalar UWB range measurement in meters.
        """

        def h_uwb(x: np.ndarray) -> np.ndarray:
            """Nonlinear measurement function: returns predicted range from origin."""
            px, py, pz = x[0, 0], x[1, 0], x[2, 0]
            return np.array([[np.sqrt(px**2 + py**2 + pz**2)]])

        def H_uwb(x: np.ndarray) -> np.ndarray:
            """
            Jacobian of h_uwb with respect to the state vector x.
            Linearises the range measurement around the current estimate.
            Shape: 1*6.
            """
            px, py, pz = x[0, 0], x[1, 0], x[2, 0]
            r = np.sqrt(px**2 + py**2 + pz**2)
            # Partial derivatives
            # Velocity states do not appear in h(x), so their partials are 0.
            return np.array([[px/r, py/r, pz/r, 0, 0, 0]])

        # Linearised measurement matrix at the current state estimate.
        H = H_uwb(self.ekf.x)

        # Predicted range from the current state estimate.
        z_pred = h_uwb(self.ekf.x)

        # UWB measurement noise variance in m^2  (sigma ≈ 0.20 m → sigma^2 = 0.04 m^2).
        R_uwb = np.array([[0.04]])

        # Innovation: actual range minus predicted range.
        y = np.array([[z_range]]) - z_pred

        # Innovation covariance, Kalman gain, and posterior updates
        # follow the same structure as the linear camera update.
        S = H @ self.ekf.P @ H.T + R_uwb
        K = self.ekf.P @ H.T @ np.linalg.inv(S)

        self.ekf.x += K @ y
        self.ekf.P = (np.eye(6) - K @ H) @ self.ekf.P

    def preflight_sim(self) -> None:
        """
        Run a synthetic smoke-test of the filter before flight.

        Simulates a target moving in a horizontal circle of radius 3 m at a
        fixed altitude (pz = 1 m) over 20 seconds.  Only camera updates are
        used (no UWB, no real IMU) to keep the test self-contained.

        The filter receives:
          - Zero acceleration input (predict_accel with (0, 0, 0)) at each step.
          - Noisy camera observations drawn from N(true_pos, 0.1^2).

        After the run, the true X-Y trajectory and the EKF-estimated
        X-Y trajectory are plotted side-by-side.  Close visual overlap
        indicates the filter is tracking correctly.
        """

        true_pos = []
        est_pos = []

        for t in np.arange(0, 20, self.dt):
            # --- Ground-truth circular trajectory ---
            # Angular frequency = 0.2 rad/s  →  period ≈ 31.4 s
            px = 3 * np.cos(0.2 * t)
            py = 3 * np.sin(0.2 * t)
            pz = 1.0  # constant altitude (meters)

            # --- Simulated noisy camera measurement ---
            # Gaussian noise with sigma = 0.1 m models fiducial detection error.
            z_cam = np.array([
                px + np.random.normal(0, 0.1),
                py + np.random.normal(0, 0.1),
                pz + np.random.normal(0, 0.1),
            ])

            # Prediction step: no real IMU in the sim — use zero acceleration.
            self.predict_accel((0.0, 0.0, 0.0))

            # Measurement update from the simulated camera observation.
            self.update_camera(z_cam)

            true_pos.append([px, py])
            est_pos.append([self.ekf.x[0, 0], self.ekf.x[1, 0]])

        true_pos_arr = np.array(true_pos)
        est_pos_arr = np.array(est_pos)

        # Plot to show overlap (should see close overlap)
        # TODO: convert graphical information to numerical
        plt.plot(true_pos_arr[:, 0], true_pos_arr[:, 1], label='True')
        plt.plot(est_pos_arr[:, 0], est_pos_arr[:, 1], label='EKF')
        plt.legend()
        plt.title('Pre-flight EKF Simulation — X/Y Trajectory')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.axis('equal')
        plt.show()

    def filter_output(self) -> dict[str, int]:
        # Get filter output
        px, py, pz, vx, vy, vz = self.ekf.x.flatten()    # filter outputs
        
        # Distance control (forward/backward)
        r = np.sqrt(px**2 + py**2 + pz**2)  # distance between drone and target
        e_d = r - self.r_desired    # error in distance
        
        v_r = (px*vx + py*vy + pz*vz) / r   # radial velocity
        Kp_d = 0.8
        Kd_d = 0.4
        v_forward = Kp_d * e_d + Kd_d * v_r # PD control
        v_forward = np.clip(v_forward, -2.0, 2.0)   # clamp

        # Left/Right control
        e_x = px    # set lateral offset to be 0
        Kp_hor = 1.2
        Kd_hor = 0.3
        v_left_right = Kp_hor * e_x + Kd_hor * vx
        v_left_right = np.clip(v_left_right, -2.0, 2.0)   # clamp

        # Up/Down control
        e_y = py    # set vertical offset to be 0
        Kp_vert = 1.0
        Kd_vert = 0.3
        v_up_down = Kp_vert * e_y + Kd_vert * vy
        v_up_down = np.clip(v_up_down, -2.0, 2.0)   # clamp

        # Yaw control
        yaw_error = np.arctan2(px, pz)
        Kp_yaw = 2.0
        Kd_yaw = 0.2
        yaw_rate = Kp_yaw * yaw_error + Kd_yaw * vx
        yaw_rate = np.clip(yaw_rate, -1.0, 1.0)   # clamp

        # Match speed
        target_speed = np.sqrt(vx**2 + vy**2 + vz**2)
        if target_speed < 10:
            target_speed = 10
        elif target_speed > 100:
            target_speed = 100

        return {
            'left_right_velocity': int(v_left_right),
            'forward_backward_velocity': int(v_forward),
            'up_down_velocity': int(v_up_down),
            'yaw_velocity': int(yaw_rate),
            'speed': int(target_speed),
        }
