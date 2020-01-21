### stonefish_ros package for ROS

This package delivers a ROS interface for the _Stonefish_ library. It also includes a standard simulator node, which loads the simulation scenario from a scenario description file (XML). The included parser extends the standard functionality of the _Stonefish_ library to enable ROS style search for files, resolution of arguments as well as a complete message interface. 

### Installation

1. Install the open-source [Stonefish](https://github.com/patrykcieslak/stonefish) library.
2. Install the open-source ROS package [cola2_msgs](https://bitbucket.org/iquarobotics/cola2_msgs).
3. Clone the *stonefish_ros* package to your catkin workspace.
4. Compile the workspace.

### Launching

This package does not include any examples of usage. The best way to learn how to use this package is to install the _stonefish_cirs_ package, containing multiple fully working examples of robots.
To launch the included standard simulator node the following lunch file tags should be used:
```
<launch>
    <arg name="simulation_data" default="..."/> <!-- path to the data directory -->
    <arg name="scenario_description" default="..."/> <!-- path to the scenario description -->
    <arg name="simulation_rate" default="500"/> <!-- number of simulation steps per second -->
    <arg name="graphics_resolution" default="1200 800"/> <!-- resolution of the simulation window -->
    <arg name="graphics_quality" default="medium"/> <!-- low, medium or high -->
    <node name="stonefish_simulator" pkg="stonefish_ros" type="parsed_simulator" args="$(arg simulation_data) $(arg scenario_description) $(arg simulation_rate) $(arg graphics_resolution) $(arg graphics_quality)"/>
</launch>
```

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
This work was part of a project titled ”Force/position control system to enable compliant manipulation from a floating I-AUV”, which received funding from the European Community H2020 Programme, under the Marie Sklodowska-Curie grant agreement no. 750063. The work is continued under a project titled ”EU Marine Robots”, which received funding from the European Community H2020 Programme, grant agreement no. 731103.

### License
This is free software, published under the General Public License v3.0.
