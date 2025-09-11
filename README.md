# Shadow Drone Swarm

This package provides a **leader–follower swarm control setup** using MAVROS and ROS 2.  
It supports running multiple UAVs (simulated or real) with flexible namespace and parameter configurations.  

---

## 📦 Build & Setup

```bash
cd ws
colcon build
source install/setup.bash
```

For selective builds:

```bash
colcon build --packages-select shadow
source install/setup.bash
```

---

## 🚀 Launch Instructions

### 1. Run MAVROS for Multiple UAVs
```bash
ros2 launch shadow mavros.launch.py uavs:="1,2"
```

### 2. Start the Simulator
```bash
ros2 launch shadow simulator.launch.py uavs:=1,2 base_port:=14550 bind_ip:=0.0.0.0
```


### 3. Launch the Full Shadow Swarm
```bash
ros2 launch shadow shadow.launch.py
```

---

## ▶️ Run Nodes Individually

### Leader
```bash
ros2 run shadow shadow_leader --ros-args -r __ns:=/leader
```

### Follower (Right Side)
```bash
ros2 run shadow shadow_follower --ros-args -r __ns:=/follower_right     -p follower_ns:=/uav2     -p side:=right
```

### Follower (Left Side)
```bash
ros2 run shadow shadow_follower --ros-args -r __ns:=/follower_left     -p follower_ns:=/uav3     -p side:=left
```

---

## 📊 Visualization with Foxglove

### Install Foxglove Bridge
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

### Launch Foxglove Bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Run as a System Service
```bash
sudo systemctl daemon-reload
sudo systemctl enable foxglove_bridge.service
sudo systemctl start foxglove_bridge.service
sudo systemctl restart foxglove_bridge.service
```

---

## 📌 Notes
- Make sure your UAVs (simulated or real) are properly connected with **MAVROS**.
- Default namespaces are `/uav1`, `/uav2`, `/uav3`.  
- The **leader** takes `/leader` namespace; **followers** can be assigned sides (`left`, `right`) and follow offsets.  
- Foxglove provides a live 3D visualization of UAV positions and telemetry.

---

## ✅ Example Workflow

```bash
# Build the workspace
cd ws
colcon build && source install/setup.bash

# Start MAVROS for 2 UAVs
ros2 launch shadow mavros.launch.py uavs:="1,2"

# Run simulator with 2 UAVs
ros2 launch shadow simulator.launch.py uavs:="1,2" base_port:=14550 bind_ip:=127.0.0.1

# Start the swarm control
ros2 launch shadow shadow.launch.py

# (Optional) View data in Foxglove
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Run Simulation

```
source ~/Projects/swarm_drone/venv/bin/activate
```

### UAV1 (SYSID 1) → send to 14550 (GCS?) and 14551 (MAVROS uav1)
```
python3 ~/Projects/ardupilot_ws/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -I0 --sysid 1 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
```

### UAV2 (SYSID 2) → send to 14550 (GCS?) and 14551+1=14551? NO — use 14551 for uav1, so uav2 must use 14551+1=14552
```
python3 ~/Projects/ardupilot_ws/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -I1 --sysid 2 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14552
```

### UAV3 (SYSID 3) → send to 14550 (GCS?) and 14551+1=14551? NO — use 14551 for uav1, so uav2 must use 14551+1=14553
```
python3 ~/Projects/ardupilot_ws/Tools/autotest/sim_vehicle.py \
  -v ArduCopter -I2 --sysid 3 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14553
```


# Position-related (GPS + global position, incl. rel_alt)
ros2 service call /uav1/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_POSITION', value: {integer: 3}}"

# Local pose & attitude group (covers LOCAL_POSITION_NED used by /local_position/pose)
ros2 service call /uav1/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_EXTRA1', value: {integer: 15}}"

# Basic status (nice to have; your /state already works but keep 1 Hz)
ros2 service call /uav1/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_EXT_STAT', value: {integer: 1}}"

ros2 service call /uav2/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_POSITION', value: {integer: 3}}"

ros2 service call /uav2/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_EXTRA1', value: {integer: 15}}"

ros2 service call /uav2/param/set mavros_msgs/srv/ParamSet \
  "{param_id: 'SR2_EXT_STAT', value: {integer: 1}}"




