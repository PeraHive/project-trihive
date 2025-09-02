README.md

cd ~/perahive/ws && source install/setup.bash
colcon build && source install/setup.bash

ros2 launch shadow mavros.launch.py uavs:="1,2"
ros2 launch shadow simulator.launch.py uavs:=1,2 base_port:=14550 bind_ip:=0.0.0.0.

cd ~/perahive/ws
colcon build --packages-select shadow && source install/setup.bash

ros2 launch shadow shadow.launch.py


ros2 service call /uav1/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 245, message_rate: 1.0}"


# Fox Glove

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

sudo systemctl daemon-reload
sudo systemctl enable foxglove_bridge.service
sudo systemctl start foxglove_bridge.service


