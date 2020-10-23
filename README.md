# P9-SLAM-RL
### Ubuntu 20.04
### ROS Noetic
### Ignition Dome
### Stable Baselines3
---------------
## Prerequisites
### Install Ignition Dome
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-dome
```

### Install gmapping
```bash
sudo apt-get install ros-noetic-gmapping
```

### Install Stable Baselines3
```bash
pip install stable-baselines3[extra]
```


