import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Tf_Node(Node):
    def __init__(self):
        super().__init__('tf_node')
        self.min_inten = 50.0
        self.pointCloud_sub = self.create_subscription(PointCloud2, '/livox/lidar', self.PC_sub, 10)
        self.pointCloud_pub = self.create_publisher(PointCloud2, '/processed_scan', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def process(self, points):
        PC2 = points
        if self.tf_buffer.can_transform('base_link', 'livox', points.header.stamp):
            transform = self.tf_buffer.lookup_transform('base_link', 'livox', points.header.stamp)        
            #print(transform.transform)
            PC2 = do_transform_cloud(points, transform)

        if self.tf_buffer.can_transform('odom', 'base_link', points.header.stamp):
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', points.header.stamp)        
            #print(transform.transform)
            PC2 = do_transform_cloud(PC2, transform)
        else:
            print("Warn")
        return PC2

    def filter_scan(self, PC2):
        filtered_points = []
        for point in point_cloud2.read_points(PC2, field_names=None, skip_nans=True):
            if point['intensity'] < self.min_inten:
                filtered_points.append(point)

        return point_cloud2.create_cloud(
            header = PC2.header,
            fields = PC2.fields,
            points = filtered_points
        )
    
    def PC_sub(self, msg):
        msg = self.filter_scan(msg)
        msg = self.process(msg)
        self.PC2_publish(msg)

    def PC2_publish(self, PC2):
        self.pointCloud_pub.publish(PC2)

def main(args=None):
    rclpy.init(args=args)

    node = Tf_Node()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()