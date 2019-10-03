### stonefish_ros package for ROS

This package uses _Stonefish_ library (developed by Patryk Cieślak) to simulate dynamics, hydrodynamics, actuators and sensors of autonomous underwater vehicles (AUV) in a realistic virtual environment, including collision detection and high quality rendering. The simulators included in the package are based on robots used at the CIRS laboratory (University of Girona, Spain) and they depend on the COLA2 software architecture developed at the lab.

### Installation

1. Install the open-source [Stonefish](https://github.com/patrykcieslak/stonefish) library.
2. Install the open-source ROS packages from the CIRS laboratory:
- [cola2_core](https://bitbucket.org/iquarobotics/cola2_core)
- [cola2_lib](https://bitbucket.org/iquarobotics/cola2_lib)
- [cola2_msgs](https://bitbucket.org/iquarobotics/cola2_msgs) 
- [cola2_girona500](https://bitbucket.org/iquarobotics/cola2_girona500)
- [girona500_description](https://bitbucket.org/iquarobotics/girona500_description)
- [cola2_sparus2](https://bitbucket.org/iquarobotics/cola2_sparus2)
- [sparus2_description](https://bitbucket.org/iquarobotics/sparus2_description)
- [eca_5emicro_manipulator_description](https://bitbucket.org/udg_cirs/eca_5emicro_manipulator_description)

3. Clone the *stonefish_ros* package to your catkin workspace.
4. Compile the workspace.
5. Launch one of the examples.

### Launching

This package includes examples, showing how to use the combination of the _Stonefish_ library and the _stonefish_ros_ library to build custom robot simulators interfacing with ROS. Before launching the robot simulators it is best to connect a gamepad compatible with Logitech F310.

There are three ways to create a new simulator:
1. Launch the standard simulator node, taking as input an XML file descirbing the simulation scenario.
2. Subclass the empty simulation classes from the _Stonefish_ library, which unlocks extended functionality and custom handling of ROS communication.
3. Subclass the ROS interfaced simulation classes from the _stonefish_ros_ library, which gives a combination of solutions 1 and 2.

The first approach is presented by the following examples:
- GIRONA500 I-AUV, equipped with an ECA 5E Micro manipulator, with a force-torque sensor installed in the manipulator wrist and a simple two finger gripper
   `roslaunch stonefish_ros girona500_eca_parsed_simulator.launch`
- SPARUS II AUV, without payload
   `roslaunch stonefish_ros sparus2_parsed_simulator.launch`
   
The second approach is presented by the following examples:
- GIRONA500 AUV in console mode (only dynamics and non-vision sensors)
   `roslaunch stonefish_ros girona500_auv_dynamics.launch`
- GIRONA500 AUV with a down-looking camera
   `roslaunch stonefish_ros girona500_auv_simulator.launch`
- GIRONA500 I-AUV, equipped with an ECA 5E Micro manipulator, with a force-torque sensor installed in the manipulator wrist
   `roslaunch stonefish_ros girona500_eca_simulator.launch` 

### Credits
This software was written and is continuously developed by Patryk Cieślak.

If you find this software useful in your research, please cite:

*Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France*
```
@inproceedings{stonefish,
   author = {Cie{\'s}lak, Patryk},
   booktitle = {Proceedings of MTS/IEEE OCEANS 2019},
   title = {{Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface}},
   month = jun,
   year = {2019}}
```
### Funding
This work was part of a project titled ”Force/position control system to enable compliant manipulation from a floating I-AUV”, which received funding from the European Community H2020 Programme, under the Marie Sklodowska-Curie grant agreement no. 750063. The work is continued under a project titled ”EU Marine Robots”, which received funding from the European Community H2020 Programme, grant agreement no. 731103.

### License
This is free software, published under the General Public License v3.0.
