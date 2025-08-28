Пакет для обнаружения светоотражающих маркеров.
Перейдите в корень рабочего пространства и выполните команду `ros2 launch odom_tf test_launch.py` для запуска лаунч файла. 
Далее в другом терминале перейдите в корень рабочего пространства и запутите детектор маркеров `ros2 run marker_detection marker_detector`.
Координаты обнаруженных маркеров сохраняются в: `~/comp/src/marker_detection/marker_detection/marker_coordinates.csv`.
