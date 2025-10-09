
## Bygg

cd ~/stonefish_ros2_ws
colcon build --symlink-install
source install/setup.bash


## Kjør Simulator

ros2 launch stonefish_bluerov2 sim.py

## Keyboard kontroll

ros2 run uuv_teleop keyboard_control

## Kjør rosviz

rviz2
