# P9-SLAM-RL #
### Ubuntu 20.04 ###
### ROS Noetic ###
### Gazebo 11.3.0 ###
---------------
## Prerequisites ##

### Install moce_base planner ##
```bash
sudo apt install ros-noetic-move-base
```

### Install global planner ##
```bash
sudo apt install ros-noetic-global-planner
```

### Install local planner ##
```bash
sudo apt install ros-noetic-teb-local-planner
```

## Installation ##


### Git clone the repository into your ROS /src folder and catkin_make it ###

```bash
$ git clone https://github.com/Nitrow/P9-SLAM-RL .

$ cd ..

$ catkin_make -DCMAKE_BUILD_TYPE=Release
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
$ roslaunch simulation gazebo.launch
```

### Run the navigation ###

```bash
$ roslaunch navigation navigation.launch 
```

### Run the trainer ###

```bash
$ cd /home/asger/P9/src/custom_gym/scripts/master
$ python3 master.py

```
### Performance after 2000 epsiodes ###

![cartpole](graphics/DuelingDDQNAgent_RLSLAM_lr0.0001_2000games.png

