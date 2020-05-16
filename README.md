# RISE LAB Controllers for multirotor vehicles

## Installation

1. Install [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [git](https://help.ubuntu.com/lts/serverguide/git.html.en)
2. Since we will use the 'catkin build' commmand, install catkin build tools:

```bash
sudo apt-get install python-catkin-tools
```

3. Create a catkin workspace, clone our repository and build. For this, in the folder of your preference:

```bash
mkdir -p ~/fast_uav_ws/src
cd ~/fast_uav_ws/src
git clone --recurse-submodules https://github.com/juanmed/riseq_fast_uav .
cd ..
catkin build
source ~/riseq_ws/devel/setup.bash
source ~/.bashrc
```



