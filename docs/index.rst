====================================
Welcome to ROS package for Stonefish
====================================

Stonefish: an advanced simulation tool developed for marine robotics
====================================================================

`Stonefish <https://github.com/patrykcieslak/stonefish>`_ is a C++ library combining a physics engine and a lightweight rendering pipeline. The physics engine is based on the core functionality of the `Bullet Physics <https://pybullet.org>`_ library, extended to deliver realistic simulation of marine robots. It is directed towards researchers in the field of marine robotics but can as well be used as a general purpose robot simulator.

*Stonefish* includes advanced hydrodynamics computation based on actual geometry of bodies to better approximate hydrodynamic forces and allow for effects not possible when using symbolic models. The rendering pipeline, developed from the ground up, delivers realistic rendering of atmosphere, ocean and underwater environment. Special focus was put on the latter, where effects of wavelength-dependent light absorption and scattering were considered.

*Stonefish* library can be used to create custom standalone simulators or combined with a `Robot Operating System (ROS) <https://www.ros.org>`_ package, called `stonefish_ros <https://github.com/patrykcieslak/stonefish_ros>`_, which delivers a convenient way to use the *Stonefish* potential in a ROS-oriented environment.

ROS package for Stonefish
=========================

The *stonefish_ros* package delivers an implementation of a standard simulator node. The standard simulator is a graphical type simulator, which loads the description of the simulation scenario from an XML file. 

The syntax of the scenario description files is explained in detail in the *Stonefish* library documentation. The *stonefish_ros* package extends this syntax, to include ROS specific definitions, through subclassing of the original XML parser class. The new parser is able to support finding files in other ROS packages, parsing parameters, as well as creating necessary publishers, subscribers and services automatically.

After the simulation scenario is constructed based on the description, the simulation automatically starts.
With each simulation step, the readings of the selected sensors are published using appropriate ROS topics and the setpoints of the robots' actuators are updated based on incoming ROS messages. The sensor publishing is defined through a set of functions, constituting a *Stonefish*-ROS interface. Moreover, some functions of the library are exposed through services.

Cite Me
=======

This software was written and is continuously developed by Patryk Cieślak. 

If you find this software useful in your research, please cite:

*Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France*

.. code-block:: bib

   @inproceedings{stonefish,
      author = "Cie{\'s}lak, Patryk",
      booktitle = "OCEANS 2019 - Marseille",
      title = "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface",
      month = "jun",
      year = "2019",
      doi = "10.1109/OCEANSE.2019.8867434"
   }

Funding
=======

This work was part of a project titled ”Force/position control system to enable compliant manipulation from a floating I-AUV”, which received funding from the European Community H2020 Programme, under the Marie Skłodowska-Curie grant agreement no. 750063. The work was continued under a project titled ”EU Marine Robots”, which received funding from the European Community H2020 Programme, grant agreement no. 731103.

.. toctree::
   :hidden:

   install
   running
   parser
   robots
   actuators
   sensors
   misc
   license