import math
import sys
import threading
import time

import numpy as np
import rclpy
from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


WHEEL_RADIUS_M = (0.0475 / 2.0) * 1.01
WHEEL_TRACK_M = 0.20
MIN_WHEEL_SETPOINT_RAD_S = 0.6
MAX_WHEEL_SETPOINT_RAD_S = 8.0
MAX_CALIBRATION_LINEAR_SPEED_M_S = 0.18
TOF_OFFSET_M = 0.12
TOF_ANGLES_RAD = (math.radians(30.0), 0.0, math.radians(-30.0))
TOF_MAX_MM = 700
FEEDBACK_TIMEOUT_S = 0.6
TOF_TIMEOUT_S = 0.8
OBSTACLE_STOP_M = 0.22
OBSTACLE_SLOW_M = 0.42
OBSTACLE_TIMEOUT_S = 3.0
GOAL_TOLERANCE_M = 0.08
STALL_TIMEOUT_S = 2.0


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PlanarEkf:
    """Wheel-odometry EKF with gyro heading and wheel-based gyro-bias updates."""

    def __init__(self):
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.diag([0.01, 0.01, 0.1, 0.25, 0.25, 0.03])
        self.process_noise = np.diag([1e-5, 1e-5, 2e-3, 0.08, 0.08, 2e-5])
        self.forward_noise = 0.035**2
        self.lateral_noise = 0.06**2
        self.yaw_rate_noise = 0.08**2

    def reset(self):
        self.state.fill(0.0)
        self.covariance = np.diag([0.01, 0.01, 0.1, 0.25, 0.25, 0.03])

    def predict(self, gyro_z, dt):
        if dt <= 0.0:
            return
        dt = min(dt, 0.1)
        x, y, yaw, vx, vy, gyro_bias = self.state

        self.state[0] = x + vx * dt
        self.state[1] = y + vy * dt
        self.state[2] = wrap_angle(yaw + (gyro_z - gyro_bias) * dt)

        transition = np.eye(6)
        transition[0, 3] = dt
        transition[1, 4] = dt
        transition[2, 5] = -dt
        self.covariance = (
            transition @ self.covariance @ transition.T
            + self.process_noise * dt
        )

    def update_wheel_speeds(self, left_rad_s, right_rad_s, gyro_z):
        yaw = self.state[2]
        vx, vy = self.state[3], self.state[4]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        forward_speed = WHEEL_RADIUS_M * (left_rad_s + right_rad_s) / 2.0
        wheel_yaw_rate = (
            WHEEL_RADIUS_M * (right_rad_s - left_rad_s) / WHEEL_TRACK_M
        )

        forward_model = vx * cos_yaw + vy * sin_yaw
        forward_jacobian = np.zeros((1, 6))
        forward_jacobian[0, 2] = -vx * sin_yaw + vy * cos_yaw
        forward_jacobian[0, 3] = cos_yaw
        forward_jacobian[0, 4] = sin_yaw
        self._measurement_update(
            np.array([forward_speed]),
            np.array([forward_model]),
            forward_jacobian,
            np.array([[self.forward_noise]]),
        )

        yaw = self.state[2]
        vx, vy = self.state[3], self.state[4]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        lateral_model = -vx * sin_yaw + vy * cos_yaw
        lateral_jacobian = np.zeros((1, 6))
        lateral_jacobian[0, 2] = -vx * cos_yaw - vy * sin_yaw
        lateral_jacobian[0, 3] = -sin_yaw
        lateral_jacobian[0, 4] = cos_yaw
        self._measurement_update(
            np.array([0.0]),
            np.array([lateral_model]),
            lateral_jacobian,
            np.array([[self.lateral_noise]]),
        )

        yaw_measurement_jacobian = np.zeros((1, 6))
        yaw_measurement_jacobian[0, 5] = -1.0
        self._measurement_update(
            np.array([wheel_yaw_rate]),
            np.array([gyro_z - self.state[5]]),
            yaw_measurement_jacobian,
            np.array([[self.yaw_rate_noise]]),
        )
        self.state[2] = wrap_angle(self.state[2])

    def _measurement_update(self, measurement, prediction, jacobian, noise):
        residual = measurement - prediction
        innovation_covariance = (
            jacobian @ self.covariance @ jacobian.T + noise
        )
        gain = (
            self.covariance
            @ jacobian.T
            @ np.linalg.inv(innovation_covariance)
        )
        self.state += (gain @ residual).reshape(-1)
        identity = np.eye(6)
        residual_projection = identity - gain @ jacobian
        self.covariance = (
            residual_projection
            @ self.covariance
            @ residual_projection.T
            + gain @ noise @ gain.T
        )


class NavigationNode(Node):
    def __init__(self):
        super().__init__("robot_differential_navigation")
        self._lock = threading.Lock()
        self._filter = PlanarEkf()
        self._wheel_rad_s = [0.0, 0.0]
        self._wheel_sample_pending = False
        self._imu = [0.0, 0.0, 0.0]
        self._tof_mm = [0, 0, 0]
        self._tof_sample_pending = False
        self._wheel_time = 0.0
        self._wheel_yaw_time = time.monotonic()
        self._wheel_yaw_unwrapped = 0.0
        self._gyro_yaw_unwrapped = 0.0
        self._ekf_yaw_unwrapped = 0.0
        self._ekf_yaw_last = 0.0
        self._imu_time = 0.0
        self._tof_time = 0.0
        self._filter_time = None
        self._goal = None
        self._active = False
        self._status = "Esperando datos del robot"
        self._max_linear_speed = 0.18
        self._max_angular_speed = 0.75
        self._command = [0.0, 0.0]
        self._stall_since = None
        self._obstacle_since = None
        self._calibration_trial = None
        self._calibration_results = []
        self._trajectory = []

        self._command_publisher = self.create_publisher(
            Float32MultiArray,
            "/robot/wheel_angular_velocity_setpoints",
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/robot/wheel_angular_velocities",
            self._wheel_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/robot/imu/planar",
            self._imu_callback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            "/robot/tof/distances_mm",
            self._tof_callback,
            10,
        )
        self.create_timer(0.05, self._control_tick)
        self.create_timer(0.1, self.publish_command)

    def _wheel_callback(self, message):
        if len(message.data) != 2 or not all(
            math.isfinite(value) for value in message.data
        ):
            return
        with self._lock:
            self._wheel_rad_s = list(message.data)
            self._wheel_time = time.monotonic()
            self._wheel_sample_pending = True

    def _imu_callback(self, message):
        if len(message.data) != 3 or not all(
            math.isfinite(value) for value in message.data
        ):
            return
        with self._lock:
            self._imu = list(message.data)
            self._imu_time = time.monotonic()

    def _tof_callback(self, message):
        if len(message.data) != 3:
            return
        with self._lock:
            self._tof_mm = [
                max(0, min(int(value), TOF_MAX_MM)) for value in message.data
            ]
            self._tof_time = time.monotonic()
            self._tof_sample_pending = True

    def set_goal(self, x, y, max_linear_speed, max_angular_speed):
        with self._lock:
            now = time.monotonic()
            if self._calibration_trial is not None:
                return False, "Detén primero la prueba manual."
            if not self._feedback_ready(now):
                return False, "Esperando lecturas recientes de ruedas, IMU y ToF."
            self._goal = (float(x), float(y))
            self._max_linear_speed = float(max_linear_speed)
            self._max_angular_speed = float(max_angular_speed)
            self._active = True
            self._stall_since = None
            self._obstacle_since = None
            self._status = "Navegando al objetivo"
            return True, self._status

    def start_calibration_trial(self, kind, reference, speed, tof_target_mm=150):
        if kind not in ("manual_distance", "tof_distance", "manual_rotation"):
            return False, "Tipo de prueba de calibración desconocido."
        if not all(math.isfinite(value) for value in (reference, speed)):
            return False, "Los valores deben ser números finitos."
        if kind == "manual_distance":
            if abs(reference) < 0.1 or abs(reference) > 5.0:
                return False, "La referencia lineal debe estar entre 0.1 y 5 m."
            if not 0.02 <= speed <= MAX_CALIBRATION_LINEAR_SPEED_M_S:
                return False, (
                    "La velocidad lineal debe estar entre 0.02 y "
                    "%.2f m/s." % MAX_CALIBRATION_LINEAR_SPEED_M_S
                )
        elif kind == "manual_rotation":
            if abs(reference) < 5.0 or abs(reference) > 720.0:
                return False, "El giro de referencia debe estar entre 5° y 720°."
            if not 0.10 <= speed <= 0.60:
                return False, "La velocidad de giro debe estar entre 0.10 y 0.60 rad/s."
        else:
            if not 100 <= tof_target_mm <= 500:
                return False, "El objetivo ToF debe estar entre 100 y 500 mm."
            if not 0.02 <= speed <= MAX_CALIBRATION_LINEAR_SPEED_M_S:
                return False, (
                    "La velocidad lineal debe estar entre 0.02 y "
                    "%.2f m/s." % MAX_CALIBRATION_LINEAR_SPEED_M_S
                )

        with self._lock:
            now = time.monotonic()
            if self._active or self._calibration_trial is not None:
                return False, "Detén el movimiento actual antes de iniciar la prueba."
            if not self._odometry_ready(now):
                return False, "Esperando lecturas recientes de ruedas e IMU."
            if kind == "tof_distance":
                center_mm = self._tof_mm[1]
                if now - self._tof_time > TOF_TIMEOUT_S or center_mm <= 0:
                    return False, "El ToF central no tiene una lectura válida reciente."
                if center_mm <= tof_target_mm + 100:
                    return False, (
                        "Aleja el robot: la lectura inicial debe superar "
                        "el objetivo ToF por al menos 100 mm."
                    )

            yaw = float(self._filter.state[2])
            if kind == "manual_distance":
                expected_duration = abs(reference) / speed
            elif kind == "manual_rotation":
                expected_duration = abs(math.radians(reference)) / speed
            else:
                expected_duration = (
                    center_mm - tof_target_mm
                ) / 1000.0 / speed
            self._calibration_trial = {
                "kind": kind,
                "reference": float(reference),
                "speed": float(speed),
                "started_at": now,
                "timeout_s": min(90.0, max(15.0, 2.0 * expected_duration + 5.0)),
                "tof_target_mm": int(tof_target_mm),
                "tof_start_mm": int(self._tof_mm[1]) if kind == "tof_distance" else 0,
                "tof_below_count": 0,
                "start_x": float(self._filter.state[0]),
                "start_y": float(self._filter.state[1]),
                "start_yaw": yaw,
                "start_gyro_yaw": self._gyro_yaw_unwrapped,
                "start_wheel_yaw": self._wheel_yaw_unwrapped,
                "start_ekf_yaw": self._ekf_yaw_unwrapped,
            }
            self._tof_sample_pending = False
            self._stall_since = None

            if kind == "manual_rotation":
                angular_speed = math.copysign(speed, reference)
                self._command = self._to_wheel_speeds(0.0, angular_speed)
                self._status = "Giro de calibración activo; marca el final manualmente"
            else:
                linear_speed = speed
                if kind == "manual_distance":
                    linear_speed = math.copysign(speed, reference)
                self._command = self._to_wheel_speeds(linear_speed, 0.0)
                self._status = (
                    "Avance activo; marca el punto final manualmente"
                    if kind == "manual_distance"
                    else "Avance activo; esperando objetivo del ToF central"
                )
            return True, self._status

    def finish_calibration_trial(self):
        with self._lock:
            if self._calibration_trial is None:
                return False, None
            return True, self._finish_calibration_trial_locked()

    def _finish_calibration_trial_locked(self):
        trial = self._calibration_trial
        state = self._filter.state
        dx = float(state[0]) - trial["start_x"]
        dy = float(state[1]) - trial["start_y"]
        forward_distance = (
            math.cos(trial["start_yaw"]) * dx
            + math.sin(trial["start_yaw"]) * dy
        )
        lateral_distance = (
            -math.sin(trial["start_yaw"]) * dx
            + math.cos(trial["start_yaw"]) * dy
        )
        gyro_rotation = math.degrees(
            self._gyro_yaw_unwrapped - trial["start_gyro_yaw"]
        )
        wheel_rotation = math.degrees(
            self._wheel_yaw_unwrapped - trial["start_wheel_yaw"]
        )
        ekf_rotation = math.degrees(
            self._ekf_yaw_unwrapped - trial["start_ekf_yaw"]
        )

        if trial["kind"] == "tof_distance":
            tof_end_mm = int(self._tof_mm[1])
            reference = (trial["tof_start_mm"] - tof_end_mm) / 1000.0
            label = "recta_tof"
            unit = "m"
        elif trial["kind"] == "manual_distance":
            tof_end_mm = 0
            reference = trial["reference"]
            label = "recta_manual"
            unit = "m"
        else:
            tof_end_mm = 0
            reference = trial["reference"]
            label = "giro_manual"
            unit = "deg"

        linear_scale_factor = (
            reference / forward_distance
            if unit == "m" and abs(forward_distance) > 0.01
            else None
        )
        wheel_separation_factor = (
            wheel_rotation / reference
            if unit == "deg" and abs(reference) > 1.0
            else None
        )
        gyro_scale_factor = (
            reference / gyro_rotation
            if unit == "deg" and abs(gyro_rotation) > 1.0
            else None
        )
        result = {
            "test": label,
            "reference": reference,
            "unit": unit,
            "odom_distance_m": forward_distance,
            "lateral_error_m": lateral_distance,
            "ekf_rotation_deg": ekf_rotation,
            "gyro_rotation_deg": gyro_rotation,
            "wheel_rotation_deg": wheel_rotation,
            "comparison_error": (
                forward_distance - reference
                if unit == "m"
                else ekf_rotation - reference
            ),
            "linear_scale_factor": linear_scale_factor,
            "wheel_separation_factor": wheel_separation_factor,
            "gyro_scale_factor": gyro_scale_factor,
            "tof_start_mm": trial["tof_start_mm"],
            "tof_end_mm": tof_end_mm,
        }
        self._calibration_results.append(result)
        self._calibration_trial = None
        self._command = [0.0, 0.0]
        self._stall_since = None
        self._status = "Prueba registrada: %s" % label
        return result

    def stop(self, status="Detenido por el usuario"):
        with self._lock:
            self._active = False
            self._goal = None
            self._calibration_trial = None
            self._stall_since = None
            self._obstacle_since = None
            self._command = [0.0, 0.0]
            self._status = status

    def reset_pose(self):
        with self._lock:
            if (
                self._active
                or self._calibration_trial is not None
                or max(abs(value) for value in self._wheel_rad_s) > 0.3
            ):
                return False
            self._filter.reset()
            self._wheel_sample_pending = False
            self._wheel_yaw_unwrapped = 0.0
            self._gyro_yaw_unwrapped = 0.0
            self._ekf_yaw_unwrapped = 0.0
            self._ekf_yaw_last = 0.0
            self._wheel_yaw_time = time.monotonic()
            self._trajectory.clear()
            self._filter_time = time.monotonic()
            self._status = "Pose reiniciada en (0, 0, 0)"
            return True

    def snapshot(self):
        with self._lock:
            state = self._filter.state
            now = time.monotonic()
            return {
                "x": float(state[0]),
                "y": float(state[1]),
                "yaw": float(state[2]),
                "gyro_yaw": self._gyro_yaw_unwrapped,
                "wheel_yaw": self._wheel_yaw_unwrapped,
                "ekf_yaw": self._ekf_yaw_unwrapped,
                "vx": float(state[3]),
                "vy": float(state[4]),
                "gyro_bias": float(state[5]),
                "wheels": list(self._wheel_rad_s),
                "wheel_sample_time": self._wheel_time,
                "wheel_yaw_rate": float(
                    WHEEL_RADIUS_M
                    * (self._wheel_rad_s[1] - self._wheel_rad_s[0])
                    / WHEEL_TRACK_M
                ),
                "command": list(self._command),
                "imu": list(self._imu),
                "tof": list(self._tof_mm),
                "goal": self._goal,
                "active": self._active,
                "manual_active": self._calibration_trial is not None,
                "calibration_trial": (
                    dict(self._calibration_trial)
                    if self._calibration_trial is not None
                    else None
                ),
                "calibration_results": [dict(row) for row in self._calibration_results],
                "status": self._status,
                "wheel_fresh": now - self._wheel_time <= FEEDBACK_TIMEOUT_S,
                "imu_fresh": now - self._imu_time <= FEEDBACK_TIMEOUT_S,
                "tof_fresh": now - self._tof_time <= TOF_TIMEOUT_S,
                "trajectory": list(self._trajectory),
            }

    def _feedback_ready(self, now):
        return self._odometry_ready(now) and (
            now - self._tof_time <= TOF_TIMEOUT_S
        )

    def _odometry_ready(self, now):
        return (
            now - self._wheel_time <= FEEDBACK_TIMEOUT_S
            and now - self._imu_time <= FEEDBACK_TIMEOUT_S
        )

    def _control_tick(self):
        now = time.monotonic()
        with self._lock:
            trial_needs_tof = (
                self._calibration_trial is not None
                and self._calibration_trial["kind"] == "tof_distance"
            )
            ready = (
                self._feedback_ready(now)
                if self._active or trial_needs_tof
                else self._odometry_ready(now)
            )
            if not ready:
                self._command = [0.0, 0.0]
                if self._active or self._calibration_trial is not None:
                    self._active = False
                    self._goal = None
                    self._calibration_trial = None
                    self._status = "Parada de seguridad: sensores ROS sin datos recientes"
                elif self._status.startswith("Esperando"):
                    self._status = "Esperando datos recientes de ruedas, IMU y ToF"
                return

            if self._filter_time is None:
                self._filter_time = now
                self._status = "Sensores conectados; pose inicial (0, 0, 0)"
                return

            dt = now - self._filter_time
            self._filter_time = now
            if dt > FEEDBACK_TIMEOUT_S:
                self._command = [0.0, 0.0]
                self._active = False
                self._goal = None
                self._calibration_trial = None
                self._status = "Parada de seguridad: pausa en datos de sensores"
                return

            gyro_z, _, _ = self._imu
            self._filter.predict(gyro_z, dt)
            self._gyro_yaw_unwrapped += gyro_z * dt
            if self._wheel_sample_pending:
                wheel_dt = max(
                    0.0, min(self._wheel_time - self._wheel_yaw_time, 0.3)
                )
                self._wheel_yaw_unwrapped += (
                    WHEEL_RADIUS_M
                    * (self._wheel_rad_s[1] - self._wheel_rad_s[0])
                    / WHEEL_TRACK_M
                    * wheel_dt
                )
                self._wheel_yaw_time = self._wheel_time
                self._filter.update_wheel_speeds(
                    self._wheel_rad_s[0], self._wheel_rad_s[1], gyro_z
                )
                self._wheel_sample_pending = False
            current_yaw = float(self._filter.state[2])
            self._ekf_yaw_unwrapped += wrap_angle(
                current_yaw - self._ekf_yaw_last
            )
            self._ekf_yaw_last = current_yaw
            x, y, yaw = self._filter.state[:3]
            self._trajectory.append((float(x), float(y)))
            if len(self._trajectory) > 400:
                del self._trajectory[:100]

            if self._calibration_trial is not None:
                self._run_calibration_trial(now)
                return

            if not self._active or self._goal is None:
                self._command = [0.0, 0.0]
                return

            if self._goal_is_reached(x, y):
                self._active = False
                self._goal = None
                self._command = [0.0, 0.0]
                self._stall_since = None
                self._status = "Objetivo alcanzado (odometría)"
                return

            linear_speed, angular_speed = self._goal_command(x, y, yaw)
            self._command = self._to_wheel_speeds(linear_speed, angular_speed)
            if self._command == [0.0, 0.0]:
                self._status = "Comando bajo el mínimo de 0.6 rad/s; detenido"
            elif 0.0 in self._command:
                self._status = "Una rueda bajo 0.6 rad/s; comando limitado"
            self._check_stall(now)

    def _run_calibration_trial(self, now):
        trial = self._calibration_trial
        if now - trial["started_at"] > trial["timeout_s"]:
            self._cancel_calibration_trial(
                "Tiempo máximo de prueba excedido; motores detenidos"
            )
            return
        if trial["kind"] == "tof_distance" and self._tof_sample_pending:
            center_mm = self._tof_mm[1]
            if center_mm <= 0:
                self._cancel_calibration_trial(
                    "ToF central inválido; prueba detenida sin registrar"
                )
                return
            if 0 < center_mm <= trial["tof_target_mm"]:
                trial["tof_below_count"] += 1
            else:
                trial["tof_below_count"] = 0
            self._tof_sample_pending = False
            if trial["tof_below_count"] >= 3:
                self._finish_calibration_trial_locked()
                return
        self._check_stall(now)

    def _cancel_calibration_trial(self, status):
        self._calibration_trial = None
        self._command = [0.0, 0.0]
        self._stall_since = None
        self._status = status

    def _goal_is_reached(self, x, y):
        return math.hypot(self._goal[0] - x, self._goal[1] - y) <= GOAL_TOLERANCE_M

    def _goal_command(self, x, y, yaw):
        dx = self._goal[0] - x
        dy = self._goal[1] - y
        distance = math.hypot(dx, dy)
        bearing_error = wrap_angle(math.atan2(dy, dx) - yaw)
        angular_speed = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, 1.8 * bearing_error),
        )
        if abs(bearing_error) > math.pi / 2.0:
            linear_speed = 0.0
        else:
            linear_speed = min(self._max_linear_speed, 0.65 * distance)
            linear_speed *= max(0.0, math.cos(bearing_error))

        tof = self._tof_mm
        left_mm, center_mm, right_mm = tof
        left_close = left_mm and left_mm / 1000.0 < 0.19
        right_close = right_mm and right_mm / 1000.0 < 0.19
        center_close = (
            center_mm and center_mm / 1000.0 < OBSTACLE_STOP_M
        )
        if left_close or right_close or center_close:
            if self._obstacle_since is None:
                self._obstacle_since = time.monotonic()
            elif time.monotonic() - self._obstacle_since >= OBSTACLE_TIMEOUT_S:
                self._active = False
                self._goal = None
                self._status = (
                    "Obstaculo persistente; detenido. "
                    "Revise el entorno y defina otro destino"
                )
                return 0.0, 0.0
        else:
            self._obstacle_since = None

        if left_close and right_close:
            self._status = "Obstaculos a ambos lados; detenido"
            return 0.0, 0.0
        if left_close:
            angular_speed = -self._max_angular_speed
            linear_speed = min(linear_speed, 0.04)
            self._status = "Obstaculo cercano a la izquierda; desviando"
        elif right_close:
            angular_speed = self._max_angular_speed
            linear_speed = min(linear_speed, 0.04)
            self._status = "Obstaculo cercano a la derecha; desviando"
        elif center_close:
            left_clearance = left_mm if left_mm else TOF_MAX_MM
            right_clearance = right_mm if right_mm else TOF_MAX_MM
            angular_speed = (
                self._max_angular_speed
                if left_clearance >= right_clearance
                else -self._max_angular_speed
            )
            linear_speed = 0.0
            self._status = "Obstaculo frontal; giro para evitarlo"
        elif left_mm and left_mm / 1000.0 < OBSTACLE_SLOW_M:
            angular_speed = max(
                -self._max_angular_speed,
                min(self._max_angular_speed, angular_speed - 0.35),
            )
            linear_speed = min(linear_speed, 0.07)
            self._status = "Obstaculo frontal izquierdo; reduciendo velocidad"
        elif right_mm and right_mm / 1000.0 < OBSTACLE_SLOW_M:
            angular_speed = max(
                -self._max_angular_speed,
                min(self._max_angular_speed, angular_speed + 0.35),
            )
            linear_speed = min(linear_speed, 0.07)
            self._status = "Obstaculo frontal derecho; reduciendo velocidad"
        elif center_mm and center_mm / 1000.0 < OBSTACLE_SLOW_M:
            linear_speed = min(linear_speed, 0.07)
            if left_mm and right_mm:
                angular_speed += max(
                    -0.35,
                    min(0.35, (left_mm - right_mm) / 1000.0),
                )
            self._status = "Obstaculo frontal; reduciendo velocidad"
        else:
            self._status = "Navegando al objetivo"
        angular_speed = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, angular_speed),
        )
        return linear_speed, angular_speed

    @staticmethod
    def _to_wheel_speeds(linear_speed, angular_speed):
        left = (
            linear_speed - angular_speed * WHEEL_TRACK_M / 2.0
        ) / WHEEL_RADIUS_M
        right = (
            linear_speed + angular_speed * WHEEL_TRACK_M / 2.0
        ) / WHEEL_RADIUS_M

        if (
            left * right > 0.0
            and max(abs(left), abs(right)) < MIN_WHEEL_SETPOINT_RAD_S
        ):
            direction = 1.0 if left > 0.0 else -1.0
            left = direction * MIN_WHEEL_SETPOINT_RAD_S
            right = direction * MIN_WHEEL_SETPOINT_RAD_S

        peak_speed = max(abs(left), abs(right))
        if peak_speed > MAX_WHEEL_SETPOINT_RAD_S:
            scale = MAX_WHEEL_SETPOINT_RAD_S / peak_speed
            left *= scale
            right *= scale

        if abs(left) < MIN_WHEEL_SETPOINT_RAD_S:
            left = 0.0
        if abs(right) < MIN_WHEEL_SETPOINT_RAD_S:
            right = 0.0
        return [left, right]

    def _check_stall(self, now):
        wheel_stalled = any(
            abs(command) >= MIN_WHEEL_SETPOINT_RAD_S
            and abs(measured) < max(0.12, abs(command) * 0.15)
            for command, measured in zip(self._command, self._wheel_rad_s)
        )
        if wheel_stalled:
            if self._stall_since is None:
                self._stall_since = now
            elif now - self._stall_since >= STALL_TIMEOUT_S:
                self._active = False
                self._goal = None
                self._calibration_trial = None
                self._command = [0.0, 0.0]
                self._status = (
                    "Posible atasco o fallo de motor: detenido; "
                    "revise el robot antes de reintentar"
                )
        else:
            self._stall_since = None

    def publish_command(self):
        with self._lock:
            message = Float32MultiArray()
            message.data = list(self._command)
        self._command_publisher.publish(message)


class RobotMap(QWidget):
    def __init__(self):
        super().__init__()
        self.data = {}
        self.setMinimumSize(500, 400)

    def set_data(self, data):
        self.data = data
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101820"))
        center_x = self.width() / 2.0
        center_y = self.height() / 2.0
        scale = min(self.width(), self.height()) / 8.0
        pose_x = self.data.get("x", 0.0)
        pose_y = self.data.get("y", 0.0)

        def screen_point(x, y):
            return (
                center_x + (x - pose_x) * scale,
                center_y - (y - pose_y) * scale,
            )

        painter.setPen(QPen(QColor("#263746"), 1))
        grid_step = scale
        x_offset = (center_x - pose_x * scale) % grid_step
        y_offset = (center_y + pose_y * scale) % grid_step
        x = x_offset
        while x < self.width():
            painter.drawLine(int(x), 0, int(x), self.height())
            x += grid_step
        y = y_offset
        while y < self.height():
            painter.drawLine(0, int(y), self.width(), int(y))
            y += grid_step

        trajectory = self.data.get("trajectory", [])
        if len(trajectory) > 1:
            painter.setPen(QPen(QColor("#4f9d69"), 2))
            for start, end in zip(trajectory, trajectory[1:]):
                painter.drawLine(
                    *map(round, screen_point(*start)),
                    *map(round, screen_point(*end)),
                )

        goal = self.data.get("goal")
        if goal:
            goal_x, goal_y = screen_point(*goal)
            painter.setPen(QPen(QColor("#ffd166"), 2))
            painter.setBrush(QColor("#ffd166"))
            painter.drawEllipse(round(goal_x - 6), round(goal_y - 6), 12, 12)

        tof = self.data.get("tof", [0, 0, 0])
        yaw = self.data.get("yaw", 0.0)
        for distance_mm, angle in zip(tof, TOF_ANGLES_RAD):
            if distance_mm <= 0:
                continue
            ray_angle = yaw + angle
            sensor_x = pose_x + TOF_OFFSET_M * math.cos(ray_angle)
            sensor_y = pose_y + TOF_OFFSET_M * math.sin(ray_angle)
            obstacle_x = sensor_x + distance_mm / 1000.0 * math.cos(ray_angle)
            obstacle_y = sensor_y + distance_mm / 1000.0 * math.sin(ray_angle)
            painter.setPen(QPen(QColor("#ef476f"), 2, Qt.DashLine))
            painter.drawLine(
                *map(round, screen_point(sensor_x, sensor_y)),
                *map(round, screen_point(obstacle_x, obstacle_y)),
            )
            ox, oy = screen_point(obstacle_x, obstacle_y)
            painter.setBrush(QColor("#ef476f"))
            painter.drawEllipse(round(ox - 4), round(oy - 4), 8, 8)

        robot_x, robot_y = screen_point(pose_x, pose_y)
        painter.save()
        painter.translate(robot_x, robot_y)
        painter.rotate(-math.degrees(yaw))
        painter.setPen(QPen(QColor("#50c4d9"), 2))
        painter.setBrush(QColor("#174a58"))
        painter.drawEllipse(-12, -12, 24, 24)
        painter.setBrush(QColor("#50c4d9"))
        painter.drawPolygon(
            QPolygonF(
                [
                    QPointF(13, 0),
                    QPointF(3, -5),
                    QPointF(3, 5),
                ]
            )
        )
        painter.restore()
        painter.setPen(QColor("#e6edf3"))
        painter.setFont(QFont("Sans Serif", 9))
        painter.drawText(10, 20, "Verde: trayectoria | Amarillo: objetivo | Rojo: lectura ToF")
        painter.drawText(10, self.height() - 10, "Escala aproximada: %.2f m/cuadro" % (1.0))
        painter.end()


class NavigationWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Navegación - Robot diferencial")
        self.resize(1050, 700)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        self.map = RobotMap()
        layout.addWidget(self.map, 3)

        panel = QVBoxLayout()
        layout.addLayout(panel, 2)

        telemetry = QGroupBox("Estado del robot")
        telemetry_layout = QVBoxLayout(telemetry)
        self.status_label = QLabel("Esperando sensores ROS")
        self.pose_label = QLabel("Pose: --")
        self.velocity_label = QLabel("Ruedas: --")
        self.command_label = QLabel("Setpoints: --")
        self.imu_label = QLabel("IMU: --")
        self.tof_label = QLabel("ToF [izq, centro, der]: --")
        for label in (
            self.status_label,
            self.pose_label,
            self.velocity_label,
            self.command_label,
            self.imu_label,
            self.tof_label,
        ):
            label.setWordWrap(True)
            telemetry_layout.addWidget(label)
        panel.addWidget(telemetry)

        goal_box = QGroupBox("Destino (metros, marco relativo al inicio)")
        goal_form = QFormLayout(goal_box)
        self.goal_x = self._number_box(-20.0, 20.0, 1.0)
        self.goal_y = self._number_box(-20.0, 20.0, 0.0)
        self.max_linear = self._number_box(0.03, 0.30, 0.12, 2)
        self.max_angular = self._number_box(0.6, 1.0, 0.75, 2)
        goal_form.addRow("X:", self.goal_x)
        goal_form.addRow("Y:", self.goal_y)
        goal_form.addRow("Velocidad máxima (m/s):", self.max_linear)
        goal_form.addRow("Giro máximo (rad/s):", self.max_angular)
        panel.addWidget(goal_box)

        controls = QGridLayout()
        self.go_button = QPushButton("Ir al destino")
        self.go_button.clicked.connect(self._start_navigation)
        self.stop_button = QPushButton("DETENER")
        self.stop_button.setStyleSheet(
            "QPushButton { background-color: #b42318; color: white; "
            "font-weight: bold; min-height: 36px; }"
        )
        self.stop_button.clicked.connect(self._stop_navigation)
        reset_button = QPushButton("Reiniciar pose aquí")
        reset_button.clicked.connect(self._reset_pose)
        controls.addWidget(self.go_button, 0, 0)
        controls.addWidget(self.stop_button, 0, 1)
        controls.addWidget(reset_button, 1, 0, 1, 2)
        panel.addLayout(controls)

        note = QLabel(
            "La pose es odometría relativa y deriva con el tiempo. "
            "Los ToF solo cubren el abanico frontal; no hay sensores traseros. "
            "La navegación usa encoders y gyro Z; ax/ay se muestran pero no se "
            "integran porque su sesgo deriva la posición. El PID corta bajo "
            "0.6 rad/s; no se amplifica una rueda para compensar a la otra."
        )
        note.setWordWrap(True)
        panel.addWidget(note)
        panel.addStretch()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    @staticmethod
    def _number_box(minimum, maximum, value, decimals=2):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setDecimals(decimals)
        box.setSingleStep(0.1)
        box.setValue(value)
        return box

    def _start_navigation(self):
        accepted, reason = self.node.set_goal(
            self.goal_x.value(),
            self.goal_y.value(),
            self.max_linear.value(),
            self.max_angular.value(),
        )
        if not accepted:
            QMessageBox.warning(self, "No se puede iniciar", reason)

    def _stop_navigation(self):
        self.node.stop()

    def _reset_pose(self):
        if not self.node.reset_pose():
            QMessageBox.warning(
                self,
                "Robot en movimiento",
                "Detén la navegación y espera a que las ruedas estén quietas.",
            )

    def _refresh(self):
        state = self.node.snapshot()
        self.status_label.setText("Estado: " + state["status"])
        self.pose_label.setText(
            "Pose EKF: x=%.2f m, y=%.2f m, θ=%.1f°"
            % (
                state["x"],
                state["y"],
                math.degrees(state["yaw"]),
            )
        )
        self.velocity_label.setText(
            "Ruedas [izq, der]: %.2f, %.2f rad/s | "
            "velocidad estimada: %.2f m/s"
            % (
                state["wheels"][0],
                state["wheels"][1],
                math.hypot(state["vx"], state["vy"]),
            )
        )
        self.command_label.setText(
            "Setpoints enviados [izq, der]: %.2f, %.2f rad/s"
            % (state["command"][0], state["command"][1])
        )
        self.imu_label.setText(
            "IMU ωz=%.3f | encoder ωz=%.3f | bias=%.3f rad/s; "
            "ax=%.2f, ay=%.2f m/s²"
            % (
                state["imu"][0],
                state["wheel_yaw_rate"],
                state["gyro_bias"],
                state["imu"][1],
                state["imu"][2],
            )
        )
        self.tof_label.setText(
            "ToF [izq, centro, der]: %s mm"
            % ", ".join(str(value) if value else "--" for value in state["tof"])
        )
        freshness = "  ".join(
            (
                "ruedas " + ("OK" if state["wheel_fresh"] else "SIN DATOS"),
                "IMU " + ("OK" if state["imu_fresh"] else "SIN DATOS"),
                "ToF " + ("OK" if state["tof_fresh"] else "SIN DATOS"),
            )
        )
        self.status_label.setToolTip(freshness)
        self.map.set_data(state)
        self.go_button.setEnabled(not state["active"])

    def closeEvent(self, event):
        self.node.stop("Aplicación cerrada")
        self.node.publish_command()
        event.accept()


def main(args=None):
    del args
    rclpy.init(args=[])
    node = NavigationNode()

    def spin_ros():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(
        target=spin_ros,
        name="robot-navigation-ros",
        daemon=True,
    )
    spin_thread.start()

    application = QApplication(sys.argv)
    window = NavigationWindow(node)
    window.show()
    try:
        return application.exec_()
    finally:
        node.stop("Aplicación cerrada")
        node.publish_command()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    raise SystemExit(main())
