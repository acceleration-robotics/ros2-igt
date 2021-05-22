# Igt One

## Installation

```bash

./install.sh

cd colcon_ws

colcon build --symlink-install

source install/setup.bash
```
## Progress

>RVIZ 2

```bash
ros2 launch tycrawler_description display.launch.py
```

<img src="./images/display.png"/>

>Gazebo 11


You might need to source it

. /usr/share/gazebo-11/setup.sh 

```bash
ros2 launch tycrawler_description gazebo.launch.py
```

<img src="./images/gazebo.png"/>



### Installation dependencies
xacro

joint-state-publisher

joint-state-controller

pip3 install pyserial

sudo usermod -a -G tty $USER

sudo usermod -a -G dialout $USER


## Real Robot Launch files

### Dependencies 
- **nav2_bringup**

- for cartographer mapping
```bash
ros2 launch tycrawler_nav cartographer.launch.py
```

- for navigation2 stack
```bash
ros2 launch tycrawler_nav navigation2.launch.py
```