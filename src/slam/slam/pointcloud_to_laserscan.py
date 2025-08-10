import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2 # модуль для чтения и обработки PointCloud2
import numpy as np

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
        
        # Параметры 
        self.declare_parameter('min_height', -0.1) #(10 см) диапазон по z вниз
        self.declare_parameter('max_height', 0.1) # диапазон по z вверх 
        self.declare_parameter('min_range', 0.1) # минимальное расстояние распознования точек
        self.declare_parameter('max_range', 10.0) # максимальное расстояние распознования точек
        self.declare_parameter('angle_min', -3.14)  # секор обзора
        self.declare_parameter('angle_max', 3.14)   
        self.declare_parameter('angle_increment', 0.0087)  # (0.5°) угловой шаг между лучами 
        self.declare_parameter('scan_frame', 'livox') # система координат сканов
        
        # подписка и публикация
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.cloud_callback,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10)
        
        self.get_logger().info("Node initialized. Waiting for pointcloud...")

    def cloud_callback(self, msg):
        min_height = self.get_parameter('min_height').value
        max_height = self.get_parameter('max_height').value
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        angle_min = self.get_parameter('angle_min').value
        angle_max = self.get_parameter('angle_max').value
        alpha_inc = self.get_parameter('angle_increment').value
        
        num_readings = int(round((angle_max - angle_min) / alpha_inc)) # рассчет считанных лучей в одном скане
        
        scan = LaserScan() # создаём пустое сообщение
        scan.header = msg.header # Копирует временную метку и frame_id из имеющегося облака точек
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = alpha_inc
        scan.time_increment = 0.0 # время снятия одного скана
        scan.scan_time = 0.1  # время между сканами (в характеристиках указано 10 гц)
        scan.range_min = min_range
        scan.range_max = max_range
        scan.ranges = [max_range] * num_readings # создаём массив в который будем закидывать расстояния до препядствий
        
        # извлекаем координаты из сообщения PointCloud2:
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True) 
        # msg - тип исходного сообщения, далее запрашиваемые поля данных и параметр отвечающий за отсеивание невалидных значений
        filtered_points = []
        
        # проверяем каждую точку из облака
        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            # отсеиваем по высоте
            if min_height <= z <= max_height:
                # по расстоянию
                rho = np.sqrt(x**2 + y**2)
                if min_range <= rho <= max_range:
                    filtered_points.append((x, y, rho))
        
        # преобразуем отфильтрованные точки в LaserScan
        for (x, y, rho) in filtered_points:
            alpha = np.arctan2(y, x)
            
            # нормализуем угол
            if alpha < angle_min:
                alpha += 2*np.pi
            if alpha > angle_max:
                alpha -= 2*np.pi
            if alpha < angle_min or alpha > angle_max:
                continue
            
            # определяем индекс луча
            index = int((alpha - angle_min) / alpha_inc)
            if 0 <= index < num_readings:
                # сохраняем минимальное расстояние чтобы отображать ближайшие препядствия
                if rho < scan.ranges[index]:
                    scan.ranges[index] = rho
        
        
        self.publisher.publish(scan)
        self.get_logger().debug(f"Published scan with {len(filtered_points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()