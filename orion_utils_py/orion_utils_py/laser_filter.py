import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')

        # Declare parameter as a flat list
        self.declare_parameter('filter_ranges', [
            0.5585054, 0.9250245,
            2.2165685, 2.6878075,
            3.7000975, 4.1364305,
            5.393067, 5.7421335
        ])

        filter_ranges_flat = self.get_parameter('filter_ranges').get_parameter_value().double_array_value
        self.filter_ranges = [(filter_ranges_flat[i], filter_ranges_flat[i+1]) for i in range(0, len(filter_ranges_flat), 2)]

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for filtered scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

    def scan_callback(self, msg):
        filtered_ranges = list(msg.ranges)  # Copy original ranges

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i in range(len(filtered_ranges)):
            angle = angle_min + i * angle_increment 

            for lower, upper in self.filter_ranges:
                if lower <= angle <= upper:
                    filtered_ranges[i] = float('nan')

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities

        self.scan_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
