## Bygg

cd ~/stonefish_ros2_ws
colcon build --symlink-install
source install/setup.bash

## Kjør Simulator

ros2 launch stonefish_bluerov2 sim.py

## Keyboard kontroll

ros2 run uuv_teleop keyboard_control


## Les DVL data

ros2 launch listener dvl_logger.launch.py


# ROS2 tips

### Kjør rosviz

rviz2

### Sjekk alle topics

ros2 topic list

