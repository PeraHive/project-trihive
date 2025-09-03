README.md

cd ~/perahive/ws && source install/setup.bash
colcon build && source install/setup.bash

ros2 launch shadow mavros.launch.py uavs:="1,2"
ros2 launch shadow simulator.launch.py uavs:=1,2 base_port:=14550 bind_ip:=0.0.0.0.

cd ~/perahive/ws
colcon build --packages-select shadow && source install/setup.bash

ros2 launch shadow shadow.launch.py

or

ros2 run shadow shadow_leader --ros-args -r __ns:=/leader

ros2 run shadow shadow_follower --ros-args -r __ns:=/follower_right \
    -p follower_ns:=/uav2 \
    -p side:=right \

ros2 run shadow shadow_follower --ros-args -r __ns:=/follower_left \
    -p follower_ns:=/uav3 \
    -p side:=left \




# Fox Glove

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

sudo systemctl daemon-reload
sudo systemctl enable foxglove_bridge.service
sudo systemctl start foxglove_bridge.service
sudo systemctl restart foxglove_bridge.service