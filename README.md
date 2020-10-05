### stonefish_ros package for ROS

This package delivers a ROS interface for the _Stonefish_ library. It also includes a standard simulator node, which loads the simulation scenario from a scenario description file (XML). The included parser extends the standard functionality of the _Stonefish_ library to enable ROS style search for files, resolution of arguments as well as a complete message interface. 

### Installation

1. Install the open-source [Stonefish](https://github.com/patrykcieslak/stonefish) library (*the same version as the ROS package!*).
2. Install the open-source ROS package [cola2_msgs](https://bitbucket.org/iquarobotics/cola2_msgs).
3. Clone the *stonefish_ros* package to your catkin workspace.
4. Compile the workspace.

### Launching

To run the standard simulator node you have to include the 'simulator.launch' file in your own launch file, overriding the default arguments.
This package does not include any examples of usage. The best way to learn how to use this package is to install the [cola2_stonefish](https://bitbucket.org/udg_cirs/cola2_stonefish) package, containing multiple, fully working examples of robots, developed by the CIRS Laboratory (University of Girona, Spain).

### Credits
This software was written and is continuously developed by Patryk Cieślak.

If you find this software useful in your research, please cite:

*Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France*
```
@inproceedings{stonefish,
   author = {Cie{\'s}lak, Patryk},
   booktitle = {OCEANS 2019 - Marseille},
   title = {{Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface}},
   month = jun,
   year = {2019},
   doi={10.1109/OCEANSE.2019.8867434}}
```
### Funding
This work was part of a project titled ”Force/position control system to enable compliant manipulation from a floating I-AUV”, which received funding from the European Community H2020 Programme, under the Marie Sklodowska-Curie grant agreement no. 750063. The work was continued under a project titled ”EU Marine Robots”, which received funding from the European Community H2020 Programme, grant agreement no. 731103.

### License
This is free software, published under the General Public License v3.0.
