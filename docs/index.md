# ICRA RoboMaster AI Challenge Simulator ——ShanghaiTech University
[![Build Status](https://travis-ci.com/Privilger/RoboMaster-Simulator.svg?token=1NvSjpoBdozy9gnxsTP8&branch=melodic-devel)](https://travis-ci.com/github/Privilger/RoboMaster-Simulator)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/10f617bd7cac445f96d0d626e00a22cf)](https://www.codacy.com?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=Privilger/RoboMaster-Simulator&amp;utm_campaign=Badge_Grade)![platform](https://img.shields.io/badge/platform-ubuntu-lightgrey.svg)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

> RM simulator made by Living Machine Lab, ShanghaiTech University

## Background

This project is aimed to build a simulator for ICRA RoboMaster AI Challenge. Through the simulator, a lot of Deep Reinforcement Learning algorithms can be tested and train a good policy for the competition. It may also require transfer learning to overcome the gap between sim to real.


This repository contains:


1. Latest 2020 arena map with [RoboMaster Visual Tag](https://github.com/Privilger/robomaster_visualtag_gazebo).
2. Robot Model
    - Full scale 2019 RoboMaster AI robot 
    - A camera is mounted on the rotatable turret.
    - [Omni-directional chassis](https://github.com/ridgeback/ridgeback_simulator) combine with [RoboRTS](https://github.com/RoboMaster/RoboRTS).
2. Compatible with the [RoboRTS](https://github.com/RoboMaster/RoboRTS) package.
1. Multi-agent navigation based on [multi-jackle](https://github.com/NicksSimulationsROS/multi_jackal) ROS package.
5. [Referee system](./src/rm_simulator/README.md)
    - bunos and supply function.
    - also handle the shooting function(whether the robot shoot the enemy successfully).
2. Connect gazebo with OpenAi gym framework by [openai_ros](http://wiki.ros.org/openai_ros) ROS package.

## Preview
### Screenshot
![Arena](arena.jpg)

### Videos
<iframe src="//player.bilibili.com/player.html?aid=91834595&bvid=BV1g7411K7ZY&cid=156809807&page=1&high_quality=1&danmaku=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="min-height:600px;width:100%;height:100%;"> 
</iframe>

<iframe src="https://player.bilibili.com/player.html?aid=327787645&bvid=BV1tA411b7kF&cid=180251953&page=1&high_quality=1&danmaku=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" style="min-height:600px;width:100%;height:100%;"> </iframe>

## Install
### Dependencies
First, install the ROS and the melodic version is recommended (http://wiki.ros.org/melodic/Installation).

```sh
$ sudo apt install protobuf-compiler
$ sudo apt install libprotoc-dev
$ sudo apt install ros-melodic-libg2o
$ sudo apt install ros-melodic-controller-manager
$ sudo apt install ros-melodic-realtime-tools 
$ sudo apt install ros-melodic-rosserial-server
$ sudo apt install ros-melodic-serial
$ sudo apt install ros-melodic-robot-localization
$ sudo apt install libgoogle-glog0v5 libgoogle-glog-dev
$ sudo apt install ros-melodic-move-base
$ sudo apt install ros-melodic-interactive-marker-twist-server
$ sudo apt install ros-melodic-gazebo-ros-control
$ sudo apt install ros-melodic-hector-gazebo-plugins
$ sudo apt install ros-melodic-joint-state-controller 
$ sudo apt install ros-melodic-joint-trajectory-controller
$ sudo apt install ros-melodic-lms1xx
$ sudo apt install libarmadillo-dev
$ sudo apt install ros-melodic-map-server
```

The robot gazebo model contains pointgrey cameras. So it is required to install pointgrey camera driver in your computer. The driver file is named [flycapture2-2.11.3.121-amd64-pkg.tgz](https://github.com/Privilger/RoboMaster-Simulator/blob/melodic-devel/flycapture2-2.11.3.121-amd64-pkg.tgz). Extract this file and run the install script.

```sh
$ echo "y\n" | sudo ./install_flycapture_only_simulation.sh
```
If there is some dependency problem, you may use
```sh
$ sudo apt --fix-broken install
```

### Install RoboMaster Visual Tag
```
$ git submodule update --init --recursive
$ cp -R src/rm_simulator/models/robomaster_visualtag_gazebo/models/* ~/.gazebo/models/
```

### Modify lidar model
```sh
$ roscd lms1xx/urdf

$ sudo vim sick_lms1xx.urdf.xacro
```
Delete \<visual> tag (from line 42 to line 48).

## Usage

### Case1: Launch four robots navigation in a map.
```sh
$ roslaunch rm_simulator ridgeback_robots.launch
```

### Case2: Launch one robot navigation in a map.
```sh
$ roslaunch rm_simulator one_rm_robot.launch
```

### Case3: Only launch world
```sh
$ roslaunch rm_simulator arena.launch
```

Feel free to take a look at the launch files in `rm_simulator` pkg.


## Config Robot Model
### Add or remove camera and turret
if you want to add the camera and turret, modify the `arg` in the launch file:

`<arg name="config" value="our_side" />`

`<arg name="config" value="enemy_side" />`

else:

`<arg name="config" value="our_side_no_camera" />`

`<arg name="config" value="enemy_side_no_camera" />`

## Func
### Rotate the turret.
jackal0 is the robot ID, the unit of an angle is radian.
Eg.
```sh
$ rostopic pub /jackal0/turret_position std_msgs/Float32 "data: 1.0"
```

### Start new game by call this service.(Not use currently)
```sh
rosservice call /start_game "start: true"
```

### Activate the debuff layer(Not use currently)
jackal0 is the robot ID. data's valid input is index from 1 to 6, the index 0 is not used.
Eg.
```
rostopic pub /jackal0/debuff std_msgs/String "data: '0 1 1 0 1 1 1'"
```

## TODO
1. [Run various Gazebo instances](https://answers.gazebosim.org//question/15897/how-to-open-several-gazebos-in-a-linux-operating-system/)
2. Add stage simulator.


## Maintainers

[@Yizheng](https://github.com/Privilger).

## Contributing

Feel free to dive in! [Open an issue](https://github.com/Privilger/rm_ws/issues/new) or submit PRs.

<!-- ### Contributors

This project exists thanks to all the people who contribute. 
<a href="graphs/contributors"><img src="https://opencollective.com/standard-readme/contributors.svg?width=890&button=false" /></a> -->


## Copyright and License

[GPL-v3](LICENSE)
