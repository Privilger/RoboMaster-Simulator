# ICRA RoboMaster AI Challenge Simulator
[![Build Status](https://travis-ci.com/Privilger/rmai_ws.svg?token=1NvSjpoBdozy9gnxsTP8&branch=master)](https://travis-ci.com/Privilger/rmai_ws)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/d4d16aa3a0664eecbd98516837f9d3f6)](https://www.codacy.com?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=Privilger/rmai_ws&amp;utm_campaign=Badge_Grade)
![platform](https://img.shields.io/badge/platform-ubuntu-lightgrey.svg)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

> RM simulator 

## Background

This project is aimed to build a simulator for ICRA RoboMaster AI Challenge. Through the simulator, a lot of Deep Reinforcement Learning algorithms can be tested and train a good policy for the competition. It may also require transfer learning to overcome the gap between sim to real.


This repository contains:


1. Latest 2020 arena map.
1. Multi-agent navigation based on [multi-jackle](https://github.com/NicksSimulationsROS/multi_jackal) ROS package.
2. [Omni-directional chassis](https://github.com/ridgeback/ridgeback_simulator) combine with RoboRTS.
2. Turret(simplified as a rotatable camera) on the robot model.
2. Connect gazebo with OpenAi gym framework by [openai_ros](http://wiki.ros.org/openai_ros) ROS package.

## TODO
1. ```resetSimulation``` function in openai_ros pkg doesn't look good. It need to be refactored. The function should put in the ```_env_setup``` function in the task env. Ref: openai_examples_projects -> Fetch Robot 
2. Speed up the gazebo simulation. 
3. Add bunos and supply function.
4. Add stage simulator.
5. Add referee system, maybe based on BlackBoard game design.

## Install
### Dependencies
First, install the ROS and the kinetic version is recommended (http://wiki.ros.org/kinetic/Installation).

```sh
$ sudo apt install protobuf-compiler
$ sudo apt install libprotoc-dev
$ sudo apt install ros-kinetic-libg2o
$ sudo apt install ros-kinetic-controller-manager
$ sudo apt install ros-kinetic-puma-motor-*
$ sudo apt install ros-kinetic-realtime-tools 
$ sudo apt install ros-kinetic-rosserial-server
$ sudo apt install ros-kinetic-robot-localization
$ sudo apt install libgoogle-glog0v5 libgoogle-glog-dev
$ sudo apt install ros-kinetic-move-base
$ sudo apt install ros-kinetic-interactive-marker-twist-server
$ sudo apt install ros-kinetic-gazebo-ros-control
$ sudo apt install ros-kinetic-hector-gazebo-plugins 
$ sudo apt install ros-kinetic-joint-state-controller 
$ sudo apt install ros-kinetic-lms1xx
$ sudo apt install libarmadillo-dev
```

The robot gazebo model contains pointgrey cameras. So it is required to install pointgrey camera driver in your computer. The driver file is named [flycapture2-2.11.3.121-amd64-pkg.tgz](https://github.com/Privilger/rm_ws/blob/master/flycapture2-2.11.3.121-amd64-pkg.tgz). Extract this file and run the install script.

```sh
$ ./install_flycapture.sh
```
If there is some dependency problem, you may use
```sh
$ sudo apt --fix-broken install
```

### Modify lidar model
```sh
$ roscd lms1xx/urdf

$ sudo vim sick_lms1xx.urdf.xacro
```
Delete \<visual> tag (from line 42 to line 48).

## Usage
### Launch four robots navigation in a map.
```sh
$ roslaunch rm_simulator ridgeback_robots.launch
```



## Maintainers

[@Yizheng](https://github.com/Privilger).

## Contributing

Feel free to dive in! [Open an issue](https://github.com/Privilger/rm_ws/issues/new) or submit PRs.

<!-- ### Contributors

This project exists thanks to all the people who contribute. 
<a href="graphs/contributors"><img src="https://opencollective.com/standard-readme/contributors.svg?width=890&button=false" /></a> -->


## Copyright and License

[GPL-v3](LICENSE)
