```bash
sudo apt update
cd ~/crazyflie_ws/src/
git clone https://github.com/NoeFBou/projet_ros2_crazyflie.git
sudo apt install -y ros-humble-desktop ros-humble-octomap ros-humble-octomap-server ros-humble-octomap-rviz-plugins ros-humble-tf-transformations ros-humble-py-trees ros-humble-py-trees-ros ros-humble-interactive-markers
sudo apt install python3-pip
pip3 install numpy scipy transforms3d
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

autres packages necessaires (pas sur pour tous)
```bash
git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git
git clone https://github.com/knmcguire/ros_gz_crazyflie
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
git clone https://github.com/bitcraze/crazyflie-simulation.git
```

lancer la navigation 3d:
```bash
cd ~/crazyflie_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch navigation3d launchtest.launch.py
```


rebuild propre :
```bash
rm -rf build install log
colcon build --symlink-install
```

juste nav3d
```bash
colcon build --symlink-instal --packages-select navigation3d
```

commande pour zoomer et suivre le drone : 
```bash
ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "crazyflie"'
ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean --timeout 2000 --req 'x: -0.2, y: 0.0, z: 0.1'
```

commande pour kills les process:
```bash
pkill -f ros2
pkill -f gz
pkill -f ruby
pkill -f sim_server
pkill -f parameter_bridge
pkill -f rviz2
pkill -9 gzserver
pkill -f gzclient
pkill -f robot_state_publisher
killall -9 _ros2_daemon 2>/dev/null
```

teleop pour debbug(sinon juste "tirer/deplacer" le drone sur gazebo):
```bash
cd ~/crazyflie_ws/
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/crazyflie/cmd_vel
```