from collections import deque
import math
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import ExternalShutdownException

from robot_differential_test.navigation import NavigationNode, RobotMap


class WheelSpeedPlot(QWidget):
    HISTORY_SECONDS = 10.0
    HISTORY_CAPACITY = 700

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples = deque(maxlen=self.HISTORY_CAPACITY)
        self._last_sample_time = 0.0
        self.setMinimumHeight(210)

    def add_sample(self, timestamp, setpoints, measured):
        values = (*setpoints, *measured)
        if (
            timestamp <= self._last_sample_time
            or len(setpoints) != 2
            or len(measured) != 2
            or not all(math.isfinite(value) for value in (timestamp, *values))
        ):
            return

        self._samples.append((timestamp, *values))
        self._last_sample_time = timestamp
        cutoff = timestamp - self.HISTORY_SECONDS
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))

        plot = QRectF(self.rect()).adjusted(58, 31, -12, -28)
        if plot.width() <= 0 or plot.height() <= 0:
            return

        samples = list(self._samples)
        if not samples:
            painter.setPen(QColor("#475569"))
            painter.drawText(plot, Qt.AlignCenter, "Esperando velocidades del encoder")
            return

        latest_time = samples[-1][0]
        start_time = max(samples[0][0], latest_time - self.HISTORY_SECONDS)
        time_span = max(latest_time - start_time, 0.1)
        max_speed = max(abs(value) for sample in samples for value in sample[1:])
        speed_limit = max(1.0, math.ceil(max_speed * 1.15))

        grid_pen = QPen(QColor("#cbd5e1"), 1)
        painter.setPen(grid_pen)
        for fraction, label in (
            (0.0, "%.1f" % speed_limit),
            (0.5, "0"),
            (1.0, "%.1f" % -speed_limit),
        ):
            y = plot.top() + fraction * plot.height()
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            painter.drawText(
                QRectF(0, y - 9, 52, 18),
                Qt.AlignRight | Qt.AlignVCenter,
                label,
            )
        for fraction in (0.0, 0.5, 1.0):
            x = plot.left() + fraction * plot.width()
            painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))
            remaining = time_span * (1.0 - fraction)
            label = "ahora" if fraction == 1.0 else "-%.0f s" % remaining
            painter.drawText(
                QRectF(x - 30, plot.bottom() + 4, 60, 18),
                Qt.AlignHCenter | Qt.AlignTop,
                label,
            )

        left_color = QColor("#2563eb")
        right_color = QColor("#dc2626")
        legend = (
            ("Setpoint izq.", left_color, Qt.DashLine),
            ("Medida izq.", left_color, Qt.SolidLine),
            ("Setpoint der.", right_color, Qt.DashLine),
            ("Medida der.", right_color, Qt.SolidLine),
        )
        x = plot.left()
        for label, color, style in legend:
            painter.setPen(QPen(color, 2, style))
            painter.drawLine(QPointF(x, 15), QPointF(x + 20, 15))
            painter.setPen(QColor("#334155"))
            painter.drawText(QPointF(x + 25, 19), label)
            x += 34 + painter.fontMetrics().width(label)

        painter.save()
        painter.setClipRect(plot)
        for value_index, color, style in (
            (1, left_color, Qt.DashLine),
            (3, left_color, Qt.SolidLine),
            (2, right_color, Qt.DashLine),
            (4, right_color, Qt.SolidLine),
        ):
            painter.setPen(QPen(color, 2, style))
            previous_point = None
            for sample in samples:
                x = plot.left() + (
                    (sample[0] - start_time) / time_span
                ) * plot.width()
                y = plot.top() + (
                    (speed_limit - sample[value_index])
                    / (2.0 * speed_limit)
                ) * plot.height()
                point = QPointF(x, y)
                if previous_point is not None:
                    painter.drawLine(previous_point, point)
                else:
                    painter.drawEllipse(point, 1.5, 1.5)
                previous_point = point
        painter.restore()
        painter.end()


class ManualControlWindow(QMainWindow):
    START_SPEED_THRESHOLD_RAD_S = 0.15
    STOP_SPEED_THRESHOLD_RAD_S = 0.15
    RESPONSE_CONFIRMATION_SAMPLES = 2
    STOP_CONFIRMATION_SAMPLES = 3
    RESPONSE_TIMEOUT_S = 5.0

    TEST_PRESETS = (
        ("Recta +2 m: marcar llegada", "manual_distance", 2.0, 0.06),
        ("Recta -2 m: marcar llegada", "manual_distance", -2.0, 0.06),
        ("Recta a pared: detener con ToF", "tof_distance", 0.0, 0.04),
        ("Giro +360°: marcar alineación", "manual_rotation", 360.0, 0.30),
        ("Giro -360°: marcar alineación", "manual_rotation", -360.0, 0.30),
        ("Giro +90°: marcar llegada", "manual_rotation", 90.0, 0.20),
        ("Giro -90°: marcar llegada", "manual_rotation", -90.0, 0.20),
    )

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Pruebas de calibración de odometría")
        self.resize(1250, 900)

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        content_layout = QHBoxLayout()
        layout.addLayout(content_layout, 3)
        self.map = RobotMap()
        content_layout.addWidget(self.map, 3)

        panel = QVBoxLayout()
        content_layout.addLayout(panel, 2)

        telemetry = QGroupBox("Telemetría")
        telemetry_layout = QVBoxLayout(telemetry)
        self.status_label = QLabel("Esperando sensores ROS")
        self.pose_label = QLabel("Pose: --")
        self.wheel_label = QLabel("Ruedas [izq, der]: --")
        self.response_latency_label = QLabel(
            "Latencia desde botón (encoder ~100 ms): sin mediciones"
        )
        self.heading_label = QLabel("Giro [EKF, gyro, encoders]: --")
        self.imu_label = QLabel("ωz gyro / ruedas: --")
        self.tof_label = QLabel("ToF [izq, centro, der]: --")
        for label in (
            self.status_label,
            self.pose_label,
            self.wheel_label,
            self.response_latency_label,
            self.heading_label,
            self.imu_label,
            self.tof_label,
        ):
            label.setWordWrap(True)
            telemetry_layout.addWidget(label)
        panel.addWidget(telemetry)

        test_box = QGroupBox("Prueba")
        test_form = QFormLayout(test_box)
        self.test_selector = QComboBox()
        for label, _, _, _ in self.TEST_PRESETS:
            self.test_selector.addItem(label)
        self.test_selector.currentIndexChanged.connect(self._update_test_form)
        self.reference = self._number_box(-5.0, 5.0, 2.0, 2)
        self.reference_label = QLabel("Distancia física de referencia [m]:")
        self.tof_target = self._number_box(100, 500, 150, 0)
        self.tof_target_label = QLabel("Distancia ToF central objetivo [mm]:")
        self.speed = self._number_box(0.02, 0.18, 0.06, 2)
        self.speed_label = QLabel("Velocidad lineal [m/s]:")
        test_form.addRow("Ensayo:", self.test_selector)
        test_form.addRow(self.reference_label, self.reference)
        test_form.addRow(self.tof_target_label, self.tof_target)
        test_form.addRow(self.speed_label, self.speed)
        panel.addWidget(test_box)

        controls = QGridLayout()
        self.start_button = QPushButton("Iniciar prueba")
        self.start_button.clicked.connect(self._start_test)
        self.finish_button = QPushButton("Marcar llegada y registrar")
        self.finish_button.clicked.connect(self._finish_test)
        self.reset_pose_button = QPushButton("Reiniciar pose aquí")
        self.reset_pose_button.clicked.connect(self._reset_pose)
        self.stop_button = QPushButton("DETENER sin registrar")
        self.stop_button.setStyleSheet(
            "QPushButton { background-color: #b42318; color: white; "
            "font-weight: bold; min-height: 36px; }"
        )
        self.stop_button.clicked.connect(self._stop_test)
        controls.addWidget(self.start_button, 0, 0)
        controls.addWidget(self.finish_button, 0, 1)
        controls.addWidget(self.reset_pose_button, 1, 0)
        controls.addWidget(self.stop_button, 1, 1)
        panel.addLayout(controls)

        results_box = QGroupBox("Resultados de esta sesión")
        results_layout = QVBoxLayout(results_box)
        self.results = QPlainTextEdit()
        self.results.setReadOnly(True)
        self.results.setPlaceholderText(
            "Los ensayos registrados aparecerán aquí. Puedes copiar el reporte "
            "y compartirlo para ajustar la odometría."
        )
        results_layout.addWidget(self.results)
        copy_button = QPushButton("Copiar reporte")
        copy_button.clicked.connect(self._copy_report)
        results_layout.addWidget(copy_button)
        panel.addWidget(results_box, 1)

        note = QLabel(
            "Los ToF solo se usan para detener el ensayo de pared; en los demás "
            "ensayos solo se muestran. La prueba no evita otros obstáculos. "
            "Despeja el área, mantén el robot supervisado y usa DETENER ante "
            "cualquier riesgo. Si hay atasco, sensores caducados o se excede "
            "el tiempo máximo, el ensayo se cancela. Ejecuta un solo controlador "
            "ROS a la vez."
        )
        note.setWordWrap(True)
        panel.addWidget(note)

        plot_box = QGroupBox("Setpoints y velocidades medidas (últimos 60 s)")
        plot_layout = QVBoxLayout(plot_box)
        self.wheel_plot = WheelSpeedPlot()
        plot_layout.addWidget(self.wheel_plot)
        layout.addWidget(plot_box, 2)
        self._last_plotted_wheel_time = 0.0

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self._start_latency_measurement = None
        self._stop_latency_measurement = None
        self._start_latency_text = "arranque: --"
        self._stop_latency_text = "frenado: --"
        self._update_test_form(0)

    @staticmethod
    def _number_box(minimum, maximum, value, decimals):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setDecimals(decimals)
        box.setSingleStep(0.1 if decimals else 10)
        box.setValue(value)
        return box

    def _update_test_form(self, index):
        _, kind, reference, speed = self.TEST_PRESETS[index]
        is_distance = kind in ("manual_distance", "tof_distance")
        is_tof = kind == "tof_distance"
        self.reference_label.setText(
            "Distancia física esperada [m]:"
            if is_distance
            else "Ángulo físico esperado [°]:"
        )
        if kind == "manual_rotation":
            self.reference.setRange(-720.0, 720.0)
            self.speed.setRange(0.10, 0.60)
            self.speed_label.setText("Velocidad de giro [rad/s]:")
        else:
            self.reference.setRange(-5.0, 5.0)
            self.speed.setRange(0.02, 0.18)
            self.speed_label.setText("Velocidad lineal [m/s]:")
        self.reference.setValue(reference)
        self.speed.setValue(speed)
        self.reference.setEnabled(not is_tof)
        self.tof_target.setEnabled(is_tof)
        self.tof_target_label.setEnabled(is_tof)

    def _start_test(self):
        request_time = time.monotonic()
        state_before_start = self.node.snapshot()
        _, kind, _, _ = self.TEST_PRESETS[self.test_selector.currentIndex()]
        accepted, reason = self.node.start_calibration_trial(
            kind,
            self.reference.value(),
            self.speed.value(),
            int(self.tof_target.value()),
        )
        if not accepted:
            QMessageBox.warning(self, "No se puede iniciar", reason)
            return

        state_after_start = self.node.snapshot()
        self._stop_latency_measurement = None
        self._stop_latency_text = "frenado: --"
        if not state_before_start["wheel_fresh"]:
            self._start_latency_measurement = None
            self._start_latency_text = "arranque: no medible (encoder sin datos recientes)"
        elif any(
            abs(speed) > self.STOP_SPEED_THRESHOLD_RAD_S
            for speed in state_before_start["wheels"]
        ):
            self._start_latency_measurement = None
            self._start_latency_text = "arranque: no medible (ruedas ya en movimiento)"
        else:
            targets = state_after_start["command"]
            self._start_latency_measurement = {
                "request_time": request_time,
                "targets": targets,
                "counts": [0, 0],
                "candidate_times": [None, None],
                "latencies": [
                    0.0 if abs(target) < self.START_SPEED_THRESHOLD_RAD_S else None
                    for target in targets
                ],
            }
            self._start_latency_text = "arranque: midiendo..."
        self._update_response_latency_label()

    def _finish_test(self):
        request_time = time.monotonic()
        state_before_stop = self.node.snapshot()
        finished, _ = self.node.finish_calibration_trial()
        if not finished:
            return
        self._arm_stop_latency_measurement(request_time, state_before_stop)
        self._refresh_results()
        QMessageBox.information(
            self,
            "Prueba registrada",
            "Se guardaron las lecturas de odometría y referencia de esta prueba.",
        )

    def _stop_test(self):
        request_time = time.monotonic()
        state_before_stop = self.node.snapshot()
        self._start_latency_measurement = None
        self.node.stop("Prueba cancelada; no se registró resultado")
        self._arm_stop_latency_measurement(request_time, state_before_stop)

    def _arm_stop_latency_measurement(self, request_time, state_before_stop):
        self._start_latency_text = "arranque: --"
        self._start_latency_measurement = None
        if not state_before_stop["wheel_fresh"]:
            self._stop_latency_measurement = None
            self._stop_latency_text = "frenado: no medible (encoder sin datos recientes)"
        else:
            wheels = state_before_stop["wheels"]
            latencies = [
                0.0 if abs(speed) <= self.STOP_SPEED_THRESHOLD_RAD_S else None
                for speed in wheels
            ]
            if all(latency is not None for latency in latencies):
                self._stop_latency_measurement = None
                self._stop_latency_text = "frenado: ruedas ya estaban detenidas"
            else:
                self._stop_latency_measurement = {
                    "request_time": request_time,
                    "counts": [0, 0],
                    "candidate_times": [None, None],
                    "latencies": latencies,
                }
                self._stop_latency_text = "frenado: midiendo..."
        self._update_response_latency_label()

    def _update_response_latency(self, sample_time, measured_wheels):
        if self._start_latency_measurement is not None:
            measurement = self._start_latency_measurement
            if sample_time >= measurement["request_time"]:
                for index, (target, measured) in enumerate(
                    zip(measurement["targets"], measured_wheels)
                ):
                    if measurement["latencies"][index] is not None:
                        continue
                    threshold = max(
                        self.START_SPEED_THRESHOLD_RAD_S,
                        abs(target) * 0.15,
                    )
                    moving_in_commanded_direction = (
                        measured * (1.0 if target > 0.0 else -1.0) >= threshold
                    )
                    if moving_in_commanded_direction:
                        if measurement["counts"][index] == 0:
                            measurement["candidate_times"][index] = sample_time
                        measurement["counts"][index] += 1
                        if (
                            measurement["counts"][index]
                            >= self.RESPONSE_CONFIRMATION_SAMPLES
                        ):
                            measurement["latencies"][index] = max(
                                0.0,
                                measurement["candidate_times"][index]
                                - measurement["request_time"],
                            )
                    else:
                        measurement["counts"][index] = 0
                        measurement["candidate_times"][index] = None

                if all(
                    latency is not None for latency in measurement["latencies"]
                ):
                    self._start_latency_text = self._format_latency(
                        "arranque", measurement["latencies"]
                    )
                    self._start_latency_measurement = None
                elif (
                    sample_time - measurement["request_time"]
                    >= self.RESPONSE_TIMEOUT_S
                ):
                    self._start_latency_text = self._format_latency(
                        "arranque", measurement["latencies"], "sin respuesta"
                    )
                    self._start_latency_measurement = None

        if self._stop_latency_measurement is not None:
            measurement = self._stop_latency_measurement
            if sample_time >= measurement["request_time"]:
                for index, measured in enumerate(measured_wheels):
                    if measurement["latencies"][index] is not None:
                        continue
                    if abs(measured) <= self.STOP_SPEED_THRESHOLD_RAD_S:
                        if measurement["counts"][index] == 0:
                            measurement["candidate_times"][index] = sample_time
                        measurement["counts"][index] += 1
                        if (
                            measurement["counts"][index]
                            >= self.STOP_CONFIRMATION_SAMPLES
                        ):
                            measurement["latencies"][index] = max(
                                0.0,
                                measurement["candidate_times"][index]
                                - measurement["request_time"],
                            )
                    else:
                        measurement["counts"][index] = 0
                        measurement["candidate_times"][index] = None

                if all(
                    latency is not None for latency in measurement["latencies"]
                ):
                    self._stop_latency_text = self._format_latency(
                        "frenado", measurement["latencies"]
                    )
                    self._stop_latency_measurement = None
                elif (
                    sample_time - measurement["request_time"]
                    >= self.RESPONSE_TIMEOUT_S
                ):
                    self._stop_latency_text = self._format_latency(
                        "frenado", measurement["latencies"], "sin parada detectada"
                    )
                    self._stop_latency_measurement = None

        self._update_response_latency_label()

    @staticmethod
    def _format_latency(label, latencies, missing=""):
        wheel_labels = ("izq", "der")
        values = []
        for wheel, latency in zip(wheel_labels, latencies):
            value = missing if latency is None else "%.2f s" % latency
            values.append("%s %s" % (wheel, value))
        return "%s: %s" % (label, ", ".join(values))

    def _update_response_latency_label(self):
        self.response_latency_label.setText(
            "Latencia desde botón (encoder ~100 ms): %s | %s"
            % (self._start_latency_text, self._stop_latency_text)
        )

    def _refresh_results(self):
        rows = self.node.snapshot()["calibration_results"]
        if not rows:
            self.results.clear()
            return
        header = (
            "prueba;referencia;unidad;avance_odom_m;error_lateral_m;"
            "giro_ekf_deg;giro_gyro_deg;giro_encoder_deg;error_medicion;"
            "factor_radio;factor_separacion_ruedas;factor_escala_gyro;"
            "tof_inicio_mm;tof_fin_mm"
        )
        lines = [header]
        for row in rows:
            factors = (
                row["linear_scale_factor"],
                row["wheel_separation_factor"],
                row["gyro_scale_factor"],
            )
            factor_values = [
                "%.4f" % factor if factor is not None else ""
                for factor in factors
            ]
            values = (
                row["test"],
                "%.4f" % row["reference"],
                row["unit"],
                "%.4f" % row["odom_distance_m"],
                "%.4f" % row["lateral_error_m"],
                "%.2f" % row["ekf_rotation_deg"],
                "%.2f" % row["gyro_rotation_deg"],
                "%.2f" % row["wheel_rotation_deg"],
                "%.2f" % row["comparison_error"],
                *factor_values,
                str(row["tof_start_mm"]),
                str(row["tof_end_mm"]),
            )
            lines.append(";".join(values))
        report = "\n".join(lines)
        if self.results.toPlainText() != report:
            self.results.setPlainText(report)

    def _copy_report(self):
        self._refresh_results()
        if not self.results.toPlainText():
            return
        QApplication.clipboard().setText(self.results.toPlainText())

    def _refresh(self):
        state = self.node.snapshot()
        wheel_sample_time = state["wheel_sample_time"]
        if wheel_sample_time > self._last_plotted_wheel_time:
            self._update_response_latency(wheel_sample_time, state["wheels"])
            self.wheel_plot.add_sample(
                wheel_sample_time,
                state["command"],
                state["wheels"],
            )
            self._last_plotted_wheel_time = wheel_sample_time
        self.status_label.setText("Estado: " + state["status"])
        self.pose_label.setText(
            "Pose EKF: x=%.3f m, y=%.3f m, θ=%.1f°"
            % (state["x"], state["y"], math.degrees(state["yaw"]))
        )
        self.wheel_label.setText(
            "Ruedas medidas [izq, der]: %.2f, %.2f rad/s | "
            "setpoints: %.2f, %.2f rad/s"
            % (
                state["wheels"][0],
                state["wheels"][1],
                state["command"][0],
                state["command"][1],
            )
        )
        self.heading_label.setText(
            "Giro acumulado [EKF, gyro, encoders]: %.1f°, %.1f°, %.1f°"
            % tuple(
                math.degrees(angle)
                for angle in (
                    state["ekf_yaw"],
                    state["gyro_yaw"],
                    state["wheel_yaw"],
                )
            )
        )
        self.imu_label.setText(
            "ωz gyro / ruedas: %.3f / %.3f rad/s | sesgo gyro: %.3f rad/s"
            % (state["imu"][0], state["wheel_yaw_rate"], state["gyro_bias"])
        )
        self.tof_label.setText(
            "ToF [izq, centro, der]: %s mm | %s"
            % (
                ", ".join(str(value) if value else "--" for value in state["tof"]),
                "actual" if state["tof_fresh"] else "sin datos recientes",
            )
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
        busy = state["active"] or state["manual_active"]
        self.start_button.setEnabled(not busy)
        self.finish_button.setEnabled(state["manual_active"])
        self.test_selector.setEnabled(not busy)
        self.reference.setEnabled(
            not busy
            and self.TEST_PRESETS[self.test_selector.currentIndex()][1]
            != "tof_distance"
        )
        self.tof_target.setEnabled(
            not busy
            and self.TEST_PRESETS[self.test_selector.currentIndex()][1]
            == "tof_distance"
        )
        self.speed.setEnabled(not busy)
        self.reset_pose_button.setEnabled(not busy)
        self._refresh_results()

    def _reset_pose(self):
        if not self.node.reset_pose():
            QMessageBox.warning(
                self,
                "Robot en movimiento",
                "Espera a que las ruedas estén quietas antes de reiniciar la pose.",
            )

    def closeEvent(self, event):
        self.node.stop("Aplicación de calibración cerrada")
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
        name="robot-calibration-ros",
        daemon=True,
    )
    spin_thread.start()

    application = QApplication(sys.argv[:1])
    window = ManualControlWindow(node)
    window.show()
    try:
        application.exec_()
    finally:
        node.stop("Aplicación cerrada")
        node.destroy_node()
        rclpy.shutdown()
