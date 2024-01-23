import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from gpiozero import CPUTemperature
import psutil

class DiagnosticPublisher(Node):

    def __init__(self):
        super().__init__('diagnostic_publisher')
        self.diagnostic_publisher = self.create_publisher(DiagnosticStatus, 'diagnostic_info', 10)
        self.timer = self.create_timer(10.0, self.publish_diagnostics)

        # Initialize CPUTemperature instance
        self.cpu_temperature_sensor = CPUTemperature()

    def get_system_info(self):
        cpu_temperature = self.cpu_temperature_sensor.temperature
        cpu_usage = psutil.cpu_percent()
        ram_usage = psutil.virtual_memory().percent

        return cpu_temperature, cpu_usage, ram_usage

    def publish_diagnostics(self):
        cpu_temperature, cpu_usage, ram_usage = self.get_system_info()

        message = DiagnosticStatus()
        message.level = DiagnosticStatus.OK
        message.name = self.get_name()
        message.message = "OK"

        message.values.append(KeyValue(key="CPU Temperature", value=f"{cpu_temperature} C"))
        message.values.append(KeyValue(key="CPU Usage", value=f"{cpu_usage}%"))
        message.values.append(KeyValue(key="RAM Usage", value=f"{ram_usage}%"))

        self.diagnostic_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    publisher_node = DiagnosticPublisher()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
