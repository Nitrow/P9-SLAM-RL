# P9-SLAM-RL #
### Ubuntu 20.04 ###
### ROS Noetic ###
### Ignition Dome ###
### Stable Baselines3 ###
---------------
## Prerequisites ##
### Install Ignition Dome ###
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-dome
```

### Install gmapping ##
```bash
sudo apt-get install ros-noetic-gmapping
```

### Install Stable Baselines3 ###
```bash
pip3 install stable-baselines3[extra]
```

## Installation ##


### Git clone the repository into your ROS /src folder and catkin_make it ###

```bash
$ git clone https://github.com/Nitrow/P9-SLAM-RL .

$ cd ..

$ catkin_make
```

### How to install the gym environment ###
The repository contains the following gym environment:

* P9-RL-env

To install the environment, go to the `P9-RL-env` folder and run the pip install command:

```bash
$ cd custom_gym/scripts/P9-RL-env

$ sudo pip3 install -e .
```

## Run learning ##

### Run the simulation ###

```bash
$ roslaunch simulation lidar.launch
```

### Run the navigation ###

```bash
$ roslaunch navigation navigation.launch 
```

### Run the trainer ###

```bash
$ roslaunch custom_gym RL_learner.launch 

```


