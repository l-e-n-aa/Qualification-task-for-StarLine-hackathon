# Квалификационное задание для хакатона “StarLine Беспилотный”

## Описание проекта
Данный репозиторий содержит решение квалификационного задания для хакатона StarLine, реализованное на ROS2 Humble. Проект включает систему одновременной
локализации и построения карты (SLAM) с детекцией маркеров и визуализацией результатов.

## Требования
- **Операционная система**: Linux Ubuntu 22.04 
- **ROS 2**: дистрибутив Humble  
- **Python**: 3.10 или выше

**Для установки ROS 2** вы можете воспользоваться данной статьёй:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Так же вам понадобятся дополнительные пакеты ROS 2 и Python, для их установки выполните следующие команды:

**Настройка окружения ROS2 и обновление apt**
```
source /opt/ros/humble/setup.bash
sudo apt update
```
**Установка python-библиотек**
```
sudo apt install -y \
  python3-numpy
```
**Установка инструментов для сборки**
```
sudo apt install -y \
  build-essential cmake \
  python3-colcon-common-extensions python3-rosdep python3-argcomplete
```
**Установка ROS-пакетов**
```
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-rclcpp \
  ros-humble-rviz2 \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs \
  ros-humble-ros2bag \
  ros-humble-rosbag2-storage-mcap\
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan
```
**Настройте rosdep**
```
sudo rosdep init 2>/dev/null || true
rosdep update
```

## Установка и настройка репозитория
Все команды выполняются в терминале Linux.

**Клонирование репозитория**
```
git clone https://github.com/l-e-n-aa/Qualification-task-for-StarLine-hackathon.git
cd Qualification-task-for-StarLine-hackathon
```
**Скачивание bag-файла**

Скачайте папку tb_office_v02 перейдя по ссылке: https://disk.yandex.ru/d/XVCkAQLCmJwf3g.

Распакуйте скачанный архив.

Далее переместите tb_office_v02 в пакет slam:
```
mv ~/Downloads/tb_office_v02 ~/Qualification-task-for-StarLine-hackathon/src/slam/
```
**Настройка окружения ROS2**
```
source /opt/ros/humble/setup.bash
```
**Установка зависимостей**
```
rosdep install -i --from-path src --rosdistro humble -y

```
**Сборка пакетов**
```
colcon build
```
**Настройка окружения рабочего пространства**
```
source install/setup.bash
```

## Запуск
Выполните команду:
```
ros2 launch slam pointcloud_to_laserscan_launch.py
```
Данная команда открывает окно rviz2, в котором запустится визуализация построения 2D карты, с отображением центров маркеров. Кроме того будут осуществляться вычисления для построения 3D карты, но визуализировать 3D карту, во избежание зависания rviz2, мы рекомендуем только после того, как в терминале выведется надпись "ICP ended". Для того, чтобы визуализировать 3D карту отобразите PointCloud2, поставив напротив него галочку в поле Displays.

## Сохранение Карты
После того как в терминале, в котором вы запускали `pointcloud_to_laserscan_launch.py`, отобразится надпись "process has finished cleanly", можно сохранить получившуюся карту. Для этого откройте новый терминал и выполните:
```
cd Qualification-task-for-StarLine-hackathon
source /opt/ros/humble/setup.bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```
Карта будет сохранена в папке Qualification-task-for-StarLine-hackathon.


