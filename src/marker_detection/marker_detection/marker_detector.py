import csv
import os
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_share_directory



class SimpleMarkerDetector(Node):
    """ Инициализация класса (constructor)"""
    def __init__(self):
        super().__init__('simple_marker_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wall_scans',
            self.cloud_callback,
            10)
        
        # параметры
        self.declare_parameter('marker_topic', '/markers')
        self.declare_parameter('marker_frame', 'map')

        # получение параметров
        marker_topic = self.get_parameter('marker_topic').value
        self.marker_frame = self.get_parameter('marker_frame').value

        self.marker_pub = self.create_publisher(PoseArray, '/detected_markers', 10)

        package_name = 'slam'
        package_path = get_package_share_directory(package_name) # возвращает абсолютный путь к директории share
        self.output_file = os.path.join(package_path, "marker_coordinates.csv")
        self.marker_file = self.output_file

        file = open(self.output_file, 'w')
        file.close

        self.detected_markers = list()

        # создаём публикатор маркеров
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)


        """ Логгирование"""
        self.get_logger().info(f"Файл будет сохранен: {self.output_file}. Детектор маркеров запущен...")


    def cloud_callback(self, msg):
        """Основная функция поиска маркеров"""
        try:
            points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
            if not points:
                return
            
            bright_points = list()
            for point in points:
                x, y, z, intensity = point
                if intensity > 50:
                    bright_points.append([x, y, z, intensity])
            if not bright_points:
                return
            
            points_array = np.array(bright_points)
            clusters = self.simple_cluster(points_array)

            detected_markers = PoseArray()
            detected_markers.header = msg.header
            
            for cluster in clusters:
                if len(cluster) >= 10:
                    center = np.mean(cluster[:, :3], axis=0)
                    
                    if self.looks_like_marker(cluster) and self.looks_like_cross(cluster, center):
                        if self.new_marker(center):
                            self.detected_markers.append(center)

                            marker_pose = Pose()
                            marker_pose.position.x = float(center[0])
                            marker_pose.position.y = float(center[1])
                            marker_pose.position.z = float(center[2])
                            marker_pose.orientation.w = 1.0
                            
                            detected_markers.poses.append(marker_pose)
                            
                            with open(self.output_file, 'a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow([
                                    f"{center[0]:.3f}",
                                    f"{center[1]:.3f}",
                                    f"{center[2]:.3f}"
                                ])
                            self.get_logger().info(f"Записан маркер: {center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}")
                            self.publish_markers()

            if detected_markers.poses:
                self.marker_pub.publish(detected_markers)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка: {str(e)}")


    def new_marker(self, maybe_marker_center):
        """Проверка на повторяющиеся маркеры"""
        for marker in self.detected_markers:
            distance = np.linalg.norm(maybe_marker_center[:2] - marker[:2])
            if distance <= 0.5:
                self.get_logger().debug(f"Обнаружен дубликат. Расстояние: {distance:.3f} м")
                return False
        return True


    def simple_cluster(self, points, distance_threshold=0.04):
        """Группируем точки вместе"""
        clusters = list()
        
        for point in points:
            placed = False
            for i, cluster in enumerate(clusters):
                cluster_center = np.mean(cluster[:, :3], axis=0)
                distance = np.linalg.norm(point[:3] - cluster_center)
                if distance < distance_threshold:
                    clusters[i] = np.vstack([cluster, point])
                    placed = True
                    break
            if not placed:
                clusters.append(np.array([point]))
        
        return clusters


    def looks_like_marker(self, cluster_points):
        """ Проверка на схожесть с маркером"""
        if len(cluster_points) < 10 or len(cluster_points) > 100:
            return False
        
        avg_intensity = np.mean(cluster_points[:, 3])
        if avg_intensity < 120:
            return False
            
        return True
    
    
    def looks_like_cross(self, cluster_points, center):
        """ Проверка на совпадение с формой маркера: проверяем что в углах маркера нет точек"""
        half_size = 0.085
        corner = half_size - 0.06  # 85-60=25мм от края
    
        points_in_corners = 0

        for point in cluster_points:
            dx = abs(point[0] - center[0])
            dy = abs(point[1] - center[1])

            # Если точка в угловой области (обе координаты > 25мм от центра)
            if dx > corner and dy > corner:
                points_in_corners += 1

        return points_in_corners <= len(cluster_points) * 0.1
    
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
        
        # задаём размер маркеров (20 см в диаметре)
        marker_msg.scale.x = 0.2  
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        
        # цвет маркеров 
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
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



def main(args = None):
    rclpy.init(args = args)
    node = SimpleMarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

