# Квалификационное задание для хакатона “StarLine Беспилотный”

## Описание проекта
Данный репозиторий содержит решение квалификационного задания для хакатона StarLine, реализованное на ROS2 Humble. Проект включает систему одновременной
локализации и построения карты (SLAM) с детекцией маркеров и визуализацией результатов.

## Требования
- **Операционная система**: Linux Ubuntu 22.04 
- **ROS 2**: дистрибутив Humble  
- **Python**: 3.10 или выше

## Установка
Все команды выполняются в терминале Linux.
**Клонирование репозитория**
```
git clone https://github.com/l-e-n-aa/Qualification-task-for-StarLine-hackathon.git
cd Qualification-task-for-StarLine-hackathon
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
**Настройкао окрузжения рабочего пространства**
```
source install/setup.bash
```

## Запуск
Выполните команду:
```
ros2 launch slam pointcloud_to_laserscan_launch.py
```
Данная команда открывает окно rviz2, в котором запустится визуализация построения 2D карты, с отображением центров маркеров.
