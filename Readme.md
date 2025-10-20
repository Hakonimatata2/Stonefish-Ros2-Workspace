# Hei

Trykk **Ctrl + Shift + v** for å vise en finere readme.


# Bygg
```
cd ~/stonefish_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Dersom man ønsker å fjerne det som er bygget:

```
cd ~/stonefish_ros2_ws
rm -rf build install log
```

# Kjør 
Kjør simulator node med:
```
ros2 launch simulator sim.py
```

Keyboard kontroll kjøres med noden:
```
ros2 run uuv_teleop keyboard_control
```

# Les DVL data
```
ros2 launch listener dvl_logger.launch.py
```
# ROS2 tips

Kjør rosviz

```
rviz2
```

Sjekk alle topics med:

```
ros2 topic list
```

Konverter fra ros1 .bag til ros2:


# Jobbe med ros data

Ros 1 .bag filer er opptak av ulike topics. De må konverteres til ros2-bag:

```
rosbags-convert \
  --src <source.bag> \
  --dst <new_destination_folder/> \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_humble
```

Source ROS2 og spill av
```
source /opt/ros/humble/setup.bash
ros2 bag play sintef_dataset_ros2/video_bag --clock
```


List alle topics ved å kjøre:

```
python3 list_topics.py
```

# Bag to .csv

Script for å konvertere til .csv. Eksempel på bruk:

```
python3 bag2csv.py --bag sintef_dataset_ros2/data_bag --topic /sensor/imu --out imu.csv --time-unit s
```

Eksempel med custom meldingstype:

```
python3 bag2csv.py \
  --bag recorded_data/simulation/rosbag2_2025_10_20-07_48_42/ \
  --topic /dvl \
  --out dvl_sim.csv \
  --time-unit s \
  --msg-dir /root/stonefish_ros2_ws/src/stonefish_ros2/msg \
  --dep-pkg std_msgs --dep-pkg sensor_msgs --dep-pkg geometry_msgs --dep-pkg builtin_interfaces
```


# Recorde data

Til Erik:

ros2 bag record /imu /ground_truth /center_camera/camera_info /center_camera/image_color
