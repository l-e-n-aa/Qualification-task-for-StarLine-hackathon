import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv
import os
from ament_index_python.packages import get_package_share_directory

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        
        # параметры
        self.declare_parameter('marker_file', 'marker_coordinates.csv')
        self.declare_parameter('marker_topic', '/markers')
        self.declare_parameter('marker_frame', 'map')
        
        
        # получение параметров
        marker_file = self.get_parameter('marker_file').value
        marker_topic = self.get_parameter('marker_topic').value
        self.marker_frame = self.get_parameter('marker_frame').value
        
        # полный путь к файлу
        package_share_dir = get_package_share_directory('marker_detection')
        self.marker_file = os.path.join(package_share_dir, marker_file)
        
        # создаём публикатор маркеров
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
    
    # считываем маркеры из таблицы
    def read_markers(self):
        markers = []
        try:
            with open(self.marker_file, 'r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if len(row) >= 3:
                        x = float(row[0])
                        y = float(row[1])
                        z = float(row[2])
                        markers.append((x, y, z))
        except Exception as e:
            self.get_logger().error(f"Error reading marker file: {str(e)}")
        
        return markers

    def publish_markers(self):
        markers = self.read_markers()
        
        # создаём сообщения для топика Marker
        marker_msg = Marker()
        marker_msg.header.frame_id = self.marker_frame
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "detected_markers"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE_LIST
        marker_msg.action = Marker.ADD
        
        # задаём размер маркеров (10 см в диаметре)
        marker_msg.scale.x = 0.1  
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        
        # цвет маркеров 
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0  # непрозрачность
        
        # означает, что система координат маркера совпадает с системой координат map:
        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.0
        # задаём отсутствие вращения (оси маркеров совпадают с осями map):
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        
        # добавляем точеки маркеров в сообщение
        for x, y, z in markers:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            marker_msg.points.append(point)
        
        # публикуем маркеры
        self.marker_pub.publish(marker_msg)
        self.get_logger().info(f"Published {len(markers)} markers")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()