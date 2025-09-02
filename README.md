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
**Настройкао окрузжения рабочего пространства**
```
source install/setup.bash
```

## Запуск
Выполните команду:
```
ros2 launch slam pointcloud_to_laserscan_launch.py
```
Данная команда открывает окно rviz2, в котором запустится визуализация построения 2D карты, с отображением центров маркеров. Кроеме того будут осуществляться вычесления для построения 3D карты, но визуализировать 3D карту, во избежание зависания rviz2, мы рекомендуем только после того, как в терминале выведется надпись "ICP ended". Для того, чтобы визуализировать 3D карту отобразите PointCloud2, поставив напротив него галочку в поле Displays.

## Сохранение Карты
После того как в терминале, в котором вы запускали `pointcloud_to_laserscan_launch.py`, отобразиться надпись "process has finished cleanly", можно сохранить получившуюся карту. Для этого откройте новый терминал и выполните:
```
source /opt/ros/humble/setup.bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```
Карта будет сохранена в папке Qualification-task-for-StarLine-hackathon.


