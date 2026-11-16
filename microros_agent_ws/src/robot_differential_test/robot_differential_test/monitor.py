import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class RobotMonitor(Node):
    def __init__(self):
        super().__init__("robot_differential_monitor")
        self._wheel_velocities = None
        self._imu_data = None
        self._tof_distances = None

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
        self.create_timer(0.5, self._report)
        self.get_logger().info("Escuchando los tópicos del robot diferencial.")

    def _wheel_callback(self, message):
        self._wheel_velocities = list(message.data)

    def _imu_callback(self, message):
        self._imu_data = list(message.data)

    def _tof_callback(self, message):
        self._tof_distances = list(message.data)

    def _report(self):
        self.get_logger().info(
            "ruedas [izq, der] rad/s: %s | "
            "IMU [gyro_z rad/s, accel_x/y m/s^2]: %s | "
            "ToF [izq, centro, der] mm: %s"
            % (
                self._format(self._wheel_velocities),
                self._format(self._imu_data),
                self._format(self._tof_distances),
            )
        )

    @staticmethod
    def _format(values):
        return "esperando datos" if values is None else str(values)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
