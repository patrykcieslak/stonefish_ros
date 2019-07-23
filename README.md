### stonefish_ros package for ROS

This package uses _Stonefish_ library (developed by Patryk Cieślak) to simulate dynamics, hydrodynamics, actuators and sensors of an I-AUV in a realistic virtual environment, including collision detection and high quality rendering.

Depends on the following open-source ROS packages:

- [cola2_core](https://bitbucket.org/iquarobotics/cola2_core)
- [cola2_lib](https://bitbucket.org/iquarobotics/cola2_lib)
- [cola2_msgs](https://bitbucket.org/iquarobotics/cola2_msgs) 
- [cola2_girona500](https://bitbucket.org/iquarobotics/cola2_girona500)
- [girona500_description](https://bitbucket.org/iquarobotics/girona500_description)
- [eca_5emicro_manipulator_description](https://bitbucket.org/udg_cirs/eca_5emicro_manipulator_description)

Depends on the following open-source libraries:

- [Stonefish](https://github.com/patrykcieslak/stonefish)

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
