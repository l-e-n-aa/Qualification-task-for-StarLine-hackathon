import csv
import os
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from sensor_msgs_py import point_cloud2


class SimpleMarkerDetector(Node):
    """ Инициализация класса (constructor)"""
    def __init__(self):
        super().__init__('simple_marker_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/processed_scan',
            self.cloud_callback,
            10)
        
        self.marker_pub = self.create_publisher(PoseArray, '/detected_markers', 10)

        package_path = os.path.dirname(__file__)
        self.output_file = os.path.join(package_path, "marker_coordinates.csv")

        self.detected_markers = list()

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


def main(args = None):
    rclpy.init(args = args)
    node = SimpleMarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

