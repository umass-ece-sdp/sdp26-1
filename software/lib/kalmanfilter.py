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
        Gyroscope readings drive the orientation states and are used to
        rotate accelerometer data from the body frame to the world frame
        before gravity compensation.
      - Drone camera (fiducial): provides a noisy but direct
        observation of the target's 3-D position in the drone's camera frame.
      - UWB ranging (drone tag <-> target anchor): provides a scalar
        line-of-sight distance measurement. This is the nonlinear part of the
        EKF — the range function h(x) = ||p|| is not linear in the state.

    State vector  x  (9 × 1):
        x = [px, py, pz, vx, vy, vz, roll, pitch, yaw]^T
        where (px, py, pz) is the target's position relative to the drone,
              (vx, vy, vz) is the target's velocity in that same frame, and
              (roll, pitch, yaw) is the target's orientation in Euler angles (rad).

    Units: meters, seconds, and radians throughout.

    Example usage:
    ```python
        from kalmanfilter import EKF
        ekf = EKF()
        while True: # drone control loop
            ekf.predict(accel_values, gyro_values)
            ekf.update_camera(aruco_estimate_from_opencv)
            ekf.update_uwb(z_range)
            commands = ekf.filter_output()
            tello.send_rc_control(
                commands['left_right_velocity'],
                commands['forward_backward_velocity'],
                commands['up_down_velocity'],
                commands['yaw_velocity'],
            )
            tello.set_speed(commands['speed'])
    ```

    Parameters:
        dim_x (int, default=9): State dimension [px, py, pz, vx, vy, vz, roll, pitch, yaw]
        dim_z (int, default=1):  Scalar measurement dimension (UWB range)
        dt (float, default=0.02):  Prediction time step in seconds (50 Hz IMU)
        q_pos (float, default=0.1): Process noise variance for position states (m^2)
        q_vel (float, default=0.1): Process noise variance for velocity states ((m/s)^2)
        q_orient (float, default=0.01): Process noise variance for orientation states (rad^2)
        covar (float, default=5): Initial state covariance scale (higher = more uncertainty)
        r_desired (float, default=10): Desired distance between target and drone (meters)
    """

    def __init__(
        self,
        *,
        dim_x: int=9,
        dim_z: int=1,
        dt: float=0.02,
        q_pos: float=0.1,
        q_vel: float=0.1,
        q_orient: float=0.01,
        covar: float=5,
        r_desired: float=10
    ) -> None:
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dt = dt
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_orient = q_orient
        self.r_desired = r_desired
        self.ekf = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)

        # Scale the initial error covariance matrix P (identity by default)
        # by `covar`.  A larger value means we start with low confidence in
        # the initial state estimate and let early measurements correct it
        # quickly.
        self.ekf.P *= covar

        # Initialise the state vector to zero — the filter has no prior
        # knowledge of where the target is before the first measurements.
        # Orientation [roll, pitch, yaw] = [0, 0, 0] assumes the target
        # starts in a level, forward-facing pose.
        self.ekf.x = np.zeros((dim_x, 1))

        # Process states and noise so EKF is ready to use
        self.state_trans_mat()
        self.process_noise()

    def state_trans_mat(self) -> None:
        """
        Build and store the state transition (process) matrix F.

        Assumes a constant-velocity kinematic model for position/velocity
        and a constant-orientation model for Euler angles, integrated over
        one time step dt:
            p_{k+1} = p_k + dt * v_k
            v_{k+1} = v_k
            θ_{k+1} = θ_k   (gyro drives orientation via the B·u control term)

        In block form:
            F = | I   dt*I   0 |
                | 0      I   0 |
                | 0      0   I |

        where I is the 3×3 identity and 0 is the 3×3 zero matrix.
        This gives a 9×9 matrix mapping [p, v, θ] -> [p + dt*v, v, θ].
        """

        self.ekf.F = np.block([
            [np.eye(3), self.dt * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), np.eye(3),    np.zeros((3, 3))],
            [np.zeros((3, 3)), np.zeros((3, 3)), np.eye(3)],
        ])

    def process_noise(self) -> None:
        """
        Build and store the process noise covariance matrix Q.

        Q is a diagonal matrix whose entries represent how much the true
        state is expected to deviate from the kinematic model between time
        steps (e.g. due to wind, target manoeuvres, gyro drift, etc.).

            Q = diag(q_pos×3, q_vel×3, q_orient×3)

        Orientation noise (q_orient) is kept smaller than position/velocity
        noise because gyro drift is slow relative to translational dynamics.
        """

        self.ekf.Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,
            self.q_vel, self.q_vel, self.q_vel,
            self.q_orient, self.q_orient, self.q_orient,
        ])

    def _rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Compute the 3×3 ZYX intrinsic rotation matrix R_body_to_world.

        Transforms a vector expressed in the body frame (glove frame) to
        the world frame using the current Euler angle estimates.  Convention
        is right-handed, ZYX order (yaw applied first, then pitch, then roll):

            R = R_roll @ R_pitch @ R_yaw

        Args:
            roll:  rotation around X-axis (rad)
            pitch: rotation around Y-axis (rad)
            yaw:   rotation around Z-axis (rad)

        Returns:
            3×3 rotation matrix.
        """
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)

        R_roll  = np.array([[1,  0,   0 ],
                             [0,  cr, -sr],
                             [0,  sr,  cr]])
        R_pitch = np.array([[ cp, 0, sp],
                             [ 0,  1,  0],
                             [-sp, 0, cp]])
        R_yaw   = np.array([[cy, -sy, 0],
                             [sy,  cy, 0],
                             [0,   0,  1]])

        return R_roll @ R_pitch @ R_yaw

    def predict_imu(self, accel: tuple[float, float, float], gyro: tuple[float, float, float]) -> None:
        """
        Propagate the state estimate forward by one time step using IMU
        readings as control inputs.

        Accelerometer readings (ax, ay, az) are rotated from the body frame
        to the world frame using the current orientation estimate, then
        gravity is subtracted to isolate motion-induced acceleration:

            a_world = R_body_to_world · a_body − [0, 0, 9.81]^T

        Gyroscope readings (ωx, ωy, ωz) in rad/s are integrated directly
        to drive the orientation states.

        Control input vector u (6×1):
            u = [a_world_x, a_world_y, a_world_z, ωx, ωy, ωz]^T

        Control-input matrix B (9×6):
            B = | 0.5·dt²·I   0    |    (position rows)
                |    dt·I     0    |    (velocity rows)
                |     0      dt·I  |    (orientation rows)

        filterpy handles the full prediction:
            x⁻ = F·x + B·u
            P⁻ = F·P·F^T + Q

        Args:
            accel: (ax, ay, az) — raw accelerometer reading in m/s^2
                   (body frame, includes gravity).
            gyro:  (ωx, ωy, ωz) — gyroscope reading in rad/s (body frame).
        """
        ax, ay, az = accel
        wx, wy, wz = gyro

        # Extract current orientation estimate from the state vector.
        roll  = float(self.ekf.x[6, 0])
        pitch = float(self.ekf.x[7, 0])
        yaw   = float(self.ekf.x[8, 0])

        # Rotate raw accelerometer vector from body frame to world frame,
        # then subtract gravity (aligned with world-frame Z axis).
        R = self._rotation_matrix(roll, pitch, yaw)
        a_world = R @ np.array([ax, ay, az]) - np.array([0.0, 0.0, 9.81])

        # Control input: world-frame linear acceleration + body-frame angular rate.
        u = np.array([[a_world[0]],
                      [a_world[1]],
                      [a_world[2]],
                      [wx],
                      [wy],
                      [wz]])

        # Control-input matrix B (9×6).
        # Upper-left  (3×3): position rows    — ½·dt²· accel
        # Middle-left (3×3): velocity rows    — dt · accel
        # Lower-right (3×3): orientation rows — dt · gyro
        B = np.block([
            [0.5 * self.dt**2 * np.eye(3), np.zeros((3, 3))],
            [self.dt * np.eye(3),           np.zeros((3, 3))],
            [np.zeros((3, 3)),              self.dt * np.eye(3)],
        ])

        # filterpy predict: x = Fx + Bu,  P = FPF^T + Q
        self.ekf.B = B
        self.ekf.predict(u=u)

    def update_camera(self, z_cam: np.ndarray) -> None:
        """
        Correct the state estimate using a fiducial position
        measured by the drone's camera.

        The camera directly observes the target's 3-D position in the drone
        frame, so the measurement model is linear:
            z = H · x + noise,   H = [I₃ | 0₃ | 0₃]  (extract position from state)

        Measurement noise covariance R_cam:
            R_cam = diag(0.05, 0.05, 0.1)  [m^2]
        Larger z-variance (0.10) reflects reduced depth precision from the
        camera compared with lateral (x, y) precision (0.05).

        filterpy's update() uses the numerically stable Joseph form:
            P = (I - K·H)·P·(I - K·H)^T + K·R·K^T

        Args:
            z_cam: (3,) or (3,1) array — [px, py, pz] measured by the camera.
        """
        def HJacobian(x: np.ndarray) -> np.ndarray:
            """Jacobian of the camera measurement: 3×9 matrix [I₃ | 0₃ | 0₃]."""
            return np.hstack((np.eye(3), np.zeros((3, 6))))

        def Hx(x: np.ndarray) -> np.ndarray:
            """Predicted camera measurement: position states [px, py, pz]."""
            return x[:3].reshape((3, 1))

        # Camera measurement noise (position uncertainty in meters^2).
        # x/y lateral noise is tighter (0.05 m^2) than depth z noise (0.10 m^2).
        R_cam = np.diag([0.05, 0.05, 0.1])

        self.ekf.update(
            z_cam.reshape((3, 1)),
            HJacobian,
            Hx,
            R=R_cam,
        )

    def update_uwb(self, z_range: float) -> None:
        """
        Correct the state estimate using a UWB range measurement.

        The UWB module measures the scalar Euclidean distance between the
        drone's tag and the target's anchor.  This is a nonlinear function
        of the position states:

            h(x) = ||p|| = sqrt(px^2 + py^2 + pz^2)

        Jacobian H_uwb (1×9):
            H_uwb = [px/r, py/r, pz/r, 0, 0, 0, 0, 0, 0]

        Velocity and orientation states have zero partial derivatives.

        Measurement noise covariance R_uwb:
            R_uwb = [[0.04]]   →  sigma ≈ 0.20 m  (±20 cm UWB ranging error)

        filterpy's update() uses the numerically stable Joseph form for the
        covariance and handles all innovation/gain/posterior equations.

        Args:
            z_range: scalar UWB range measurement in meters.
        """

        def h_uwb(x: np.ndarray) -> np.ndarray:
            """Nonlinear measurement function: predicted range from origin."""
            px, py, pz = float(x[0, 0]), float(x[1, 0]), float(x[2, 0])
            return np.array([[np.sqrt(px**2 + py**2 + pz**2)]])

        def H_uwb(x: np.ndarray) -> np.ndarray:
            """
            Jacobian of h_uwb with respect to the state vector (1×9).
            Velocity and orientation states have zero partial derivatives.
            """
            px, py, pz = float(x[0, 0]), float(x[1, 0]), float(x[2, 0])
            r = np.sqrt(px**2 + py**2 + pz**2)
            return np.array([[px/r, py/r, pz/r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

        # Guard: skip update if position estimate is at origin (r ≈ 0) to avoid
        # division by zero in H_uwb. This can occur on the first iterations before
        # any measurements have been incorporated.
        px_c, py_c, pz_c = float(self.ekf.x[0, 0]), float(self.ekf.x[1, 0]), float(self.ekf.x[2, 0])
        if np.sqrt(px_c**2 + py_c**2 + pz_c**2) < 1e-6:
            return

        # UWB measurement noise variance in m^2  (sigma ≈ 0.20 m → sigma^2 = 0.04 m^2).
        R_uwb = np.array([[0.04]])

        self.ekf.update(
            np.array([[z_range]]),
            H_uwb,
            h_uwb,
            R=R_uwb,
        )

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

            # Prediction step: no real IMU in the sim — use zero acceleration and zero gyro.
            self.predict_imu((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

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
        # Get filter output — unpack all 9 states
        px, py, pz, vx, vy, vz, roll, pitch, yaw = self.ekf.x.flatten()

        # Distance control (forward/backward)
        r = np.sqrt(px**2 + py**2 + pz**2)  # distance between drone and target (m)
        e_d = r - self.r_desired    # error in distance (m)

        # Guard against division by zero when drone and target are co-located
        v_r = (px*vx + py*vy + pz*vz) / r if r > 1e-6 else 0.0  # radial velocity (m/s)
        Kp_d = 0.8
        Kd_d = 0.4
        v_forward = Kp_d * e_d + Kd_d * v_r  # PD control (m/s)
        v_forward = np.clip(v_forward * 100, -100, 100)   # convert m/s → cm/s, clamp

        # Left/Right control
        e_x = px    # set lateral offset to be 0
        Kp_hor = 1.2
        Kd_hor = 0.3
        v_left_right = Kp_hor * e_x + Kd_hor * vx  # m/s
        v_left_right = np.clip(v_left_right * 100, -100, 100)   # convert m/s → cm/s, clamp

        # Up/Down control
        e_y = py    # set vertical offset to be 0
        Kp_vert = 1.0
        Kd_vert = 0.3
        v_up_down = Kp_vert * e_y + Kd_vert * vy  # m/s
        v_up_down = np.clip(v_up_down * 100, -100, 100)   # convert m/s → cm/s, clamp

        # Yaw control — use the estimated yaw state directly instead of a
        # geometric arctan2 proxy.  This lets the drone track the target's
        # actual heading independently of lateral position offset.
        Kp_yaw = 2.0
        Kd_yaw = 0.2
        yaw_rate = Kp_yaw * yaw + Kd_yaw * vx  # rad-scale
        yaw_rate = np.clip(yaw_rate * 100, -100, 100)  # scale to [-100, 100], clamp

        # Match speed — convert target velocity magnitude from m/s to cm/s
        target_speed = np.sqrt(vx**2 + vy**2 + vz**2) * 100
        target_speed = float(np.clip(target_speed, 10, 100))

        return {
            'left_right_velocity': int(v_left_right),
            'forward_backward_velocity': int(v_forward),
            'up_down_velocity': int(v_up_down),
            'yaw_velocity': int(yaw_rate),
            'speed': int(target_speed),
        }
