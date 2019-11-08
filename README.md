# ICRA RoboMaster AI Challenge Simulator


> RM simulator 

## Background

This project is aimed to build a simulator for ICRA RoboMaster AI Challenge. Through the simulator, a lot of Deep Reinforcement Learning algorithms can be tested and train a good policy for the competition. It may also require transfer learning to overcome the gap between sim to real.


This repository contains:

1. Multi-agent navigation based on [multi-jackle](https://github.com/NicksSimulationsROS/multi_jackal) ROS package.


## TODO
1. Add the 2020 Competition's map.
2. Connect it with OpenAi gym framework. It can be done by ref the [gym-gazebo2](https://github.com/AcutronicRobotics/gym-gazebo2) repository.
3. Speed up the gazebo simulation. 
4. Add bunos and supply function.
5. Add stage simulator.
6. Replace differential chassis with omni-directional chassis.

## Install

The robot gazebo model contains pointgrey cameras. So it is required to install pointgrey camera driver in your computer. The driver file is named [flycapture2-2.11.3.121-amd64-pkg.tgz](https://github.com/Privilger/rm_ws/blob/master/flycapture2-2.11.3.121-amd64-pkg.tgz). Extract this file and run the install script.

```sh
$ ./install_flycapture.sh
```

## Usage
Launch two robots navigation in a map.
```sh
$ roslaunch rm_simulator autonomy_exist_map.launch
```
It will open 2 rviz, each rviz can send a goal to one robot.


## Maintainers

[@Yizheng](https://github.com/Privilger).

## Contributing

Feel free to dive in! [Open an issue](https://github.com/Privilger/rm_ws/issues/new) or submit PRs.

<!-- ### Contributors

This project exists thanks to all the people who contribute. 
<a href="graphs/contributors"><img src="https://opencollective.com/standard-readme/contributors.svg?width=890&button=false" /></a> -->


## Copyright and License

[GPL-v3](LICENSE)
