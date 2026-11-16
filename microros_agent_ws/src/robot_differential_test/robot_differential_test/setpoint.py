import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray


class SetpointPublisher(Node):
    def __init__(self, left_setpoint_rad_s, right_setpoint_rad_s):
        super().__init__("robot_differential_setpoint_test")
        self.declare_parameter(
            "velocidad_izquierda_rad_s", left_setpoint_rad_s
        )
        self.declare_parameter("velocidad_derecha_rad_s", right_setpoint_rad_s)
        self._left_setpoint_rad_s = float(
            self.get_parameter("velocidad_izquierda_rad_s").value
        )
        self._right_setpoint_rad_s = float(
            self.get_parameter("velocidad_derecha_rad_s").value
        )
        self.add_on_set_parameters_callback(self._update_setpoint)
        self._publisher = self.create_publisher(
            Float32MultiArray,
            "/robot/wheel_angular_velocity_setpoints",
            10,
        )
        self.create_timer(0.1, self._publish_setpoint)
        self.get_logger().info(
            "Publicando setpoints [izquierda, derecha] = "
            "[%.3f, %.3f] rad/s."
            % (self._left_setpoint_rad_s, self._right_setpoint_rad_s)
        )

    def _update_setpoint(self, parameters):
        updated_values = {}
        for parameter in parameters:
            if parameter.name == "velocidad_izquierda_rad_s":
                wheel_name = "izquierda"
            elif parameter.name == "velocidad_derecha_rad_s":
                wheel_name = "derecha"
            else:
                continue

            value = parameter.value
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return SetParametersResult(
                    successful=False,
                    reason="Los setpoints deben ser valores numéricos.",
                )
            if not math.isfinite(value):
                return SetParametersResult(
                    successful=False,
                    reason="Los setpoints deben ser números finitos.",
                )

            updated_values[wheel_name] = float(value)

        if "izquierda" in updated_values:
            self._left_setpoint_rad_s = updated_values["izquierda"]
        if "derecha" in updated_values:
            self._right_setpoint_rad_s = updated_values["derecha"]
        if updated_values:
            self.get_logger().info(
                "Nuevos setpoints [izquierda, derecha] = [%.3f, %.3f] rad/s."
                % (self._left_setpoint_rad_s, self._right_setpoint_rad_s)
            )
        return SetParametersResult(successful=True)

    def _publish_setpoint(self):
        message = Float32MultiArray()
        message.data = [
            self._left_setpoint_rad_s,
            self._right_setpoint_rad_s,
        ]
        self._publisher.publish(message)


def main(args=None):
    raw_args = sys.argv[1:] if args is None else args
    if len(raw_args) not in (1, 2):
        raise SystemExit(
            "Uso: ros2 run robot_differential_test setpoint "
            "<velocidad_izquierda_rad_s> [velocidad_derecha_rad_s]"
        )
    try:
        left_setpoint_rad_s = float(raw_args[0])
        right_setpoint_rad_s = (
            float(raw_args[1]) if len(raw_args) == 2 else left_setpoint_rad_s
        )
    except ValueError as error:
        raise SystemExit("Los setpoints deben ser números en rad/s.") from error
    if not math.isfinite(left_setpoint_rad_s) or not math.isfinite(
        right_setpoint_rad_s
    ):
        raise SystemExit("Los setpoints deben ser números finitos.")

    rclpy.init(args=[])
    node = SetpointPublisher(left_setpoint_rad_s, right_setpoint_rad_s)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
