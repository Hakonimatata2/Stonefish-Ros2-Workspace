# Hei

Trykk **Ctrl + Shift + v** for å vise en finere readme.


# Bygg
```
cd ~/stonefish_ros2_ws
colcon build --symlink-install
source install/setup.bash
```
# Kjør 
Kjør simulator node med:
```
ros2 launch stonefish_bluerov2 sim.py
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

List alle topics ved å kjøre:

```
python3 list_topics.py
```

Eksempel på å konvertere til .csv:

```
python3 bag2csv.py --bag sintef_dataset_ros2/data_bag --topic /sensor/imu --out imu.csv --time-unit s
```
