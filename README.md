# ICRA RoboMaster AI Challenge Simulator


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
2. ```resetSimulation``` function in openai_ros pkg doesn't look good. It need to be refactored. The function should put in the ```_env_setup``` function in the task env. Ref: openai_examples_projects -> Fetch Robot 
3. Speed up the gazebo simulation. 
4. Add bunos and supply function.
5. Add stage simulator.
6. Add referee system, maybe based on BlackBoard game design.

## Install
### Dependencies
```sh
$ sudo apt install protobuf-compiler
$ sudo apt install libprotoc-dev
$ sudo apt install ros-kinetic-libg2o
```

The robot gazebo model contains pointgrey cameras. So it is required to install pointgrey camera driver in your computer. The driver file is named [flycapture2-2.11.3.121-amd64-pkg.tgz](https://github.com/Privilger/rm_ws/blob/master/flycapture2-2.11.3.121-amd64-pkg.tgz). Extract this file and run the install script.

```sh
$ ./install_flycapture.sh
```

## Usage
### Launch two robots navigation in a map.
```sh
$ roslaunch rm_simulator autonomy_exist_map.launch
```
It will open 2 rviz, each rviz can send a goal to one robot.

### Launch one omni robot navigation in a map.
```sh
$ roslaunch rm_simulator ridgeback_exist_map.launch
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
