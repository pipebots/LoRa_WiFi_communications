import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus

import csv

class DiagnosticSubscriber(Node):

    def __init__(self):
        super().__init__('diagnostic_subscriber')
        self.diagnostic_subscriber = self.create_subscription(
            DiagnosticStatus, 'diagnostic_info', self.diagnostic_callback, 10)

        self.csv_filename = '/home/pi1/Documents/wifi_data/diagnostic_data.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header to CSV file
        self.csv_writer.writerow(['Message', 'Key', 'Value'])

    def diagnostic_callback(self, msg):
        self.get_logger().info("Received Diagnostic Information:")
        self.get_logger().info(f"Level: {msg.level}")
        self.get_logger().info(f"Name: {msg.name}")
        self.get_logger().info(f"Message: {msg.message}")

        for kv in msg.values:
            self.get_logger().info(f"{kv.key}: {kv.value}")

            # Append data to CSV file
            self.csv_writer.writerow([msg.message, kv.key, kv.value])
            self.csv_file.flush()  # Flush the buffer to ensure data is written immediately

    def destroy(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = DiagnosticSubscriber()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
