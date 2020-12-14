=======
Running
=======

Standard graphical simulator
============================

To run the standard simulator you have to execute the ``parsed_simulator`` node, passing the required command line arguments:

.. code-block:: console

    $ rosrun stonefish_ros parsed_simulator <data_path> <description_filename> <rate> <width> <height> <graphics_quality>

where:

* data_path - the path to the simulation data directory
* description_filename - the path to the scenario description file
* rate - the sampling rate of the simulation [Hz]
* width & height - the resolution of the main simulator window [px]
* graphics_quality - the quality of the rendering: low, medium or high.

Another option is to include the provided ``launch/simulator.launch`` file in your own launch file. It is necessary to override the default arguments. See an example below:

.. code-block:: xml

    <launch>
        <include file="$(find stonefish_ros)/launch/simulator.launch">
            <arg name="simulation_data" value="$(find my_package)/data"/>
            <arg name="scenario_description" value="$(find my_package)/scenarios/simulation.scn"/>
            <arg name="simulation_rate" value="300.0"/>
            <arg name="graphics_resolution" value="1200 800"/>
            <arg name="graphics_quality" value="high"/>
        </include>
    </launch>

.. note::

    This package does not include any examples of usage. The best way to learn how to use this package is to install the `cola2_stonefish <https://bitbucket.org/udg_cirs/cola2_stonefish>`_ package, containing multiple, fully working examples of robots, developed by the `CIRS Laboratory (University of Girona, Spain) <https://cirs.udg.edu>`_.

Implemented services
====================

Some functionality of the simulator node is available through ROS services:

* ``enable_currents`` - enable simulation of ocean currents (type *Trigger*)

* ``disable_currents`` - disable simulation of ocean currents (type *Trigger*)

