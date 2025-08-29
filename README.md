README.md

cd ~/perahive/ws
colcon build --packages-select shadow
source install/setup.bash
ros2 launch shadow shadow.launch.py
