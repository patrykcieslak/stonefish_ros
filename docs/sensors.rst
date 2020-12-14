.. _sensors:

=======
Sensors
=======

The *Stonefish* library implements a wide range of sensor simulations. Each of the sensors produces a specific output, that can be retrieved. The *stonefish_ros* package implements automatic allocation of publishers and generation of appropriate messages, published when the new sensor data is available. The only definition that the user has to add to the standard definiton of a sensor, to use this functionality, is the name of the topic used by the publisher:

.. code-block:: xml

    <sensor name="..." type="...">
        <!-- standard sensor definitions -->
        <ros_publisher topic="/topic_name"/>
    </sensor>

For each sensor type a different message type is published. Some of the sensors need to publish on multiple topics. Specific information on all supported sensors is presented below.

Rotary encoder
==============

Message type: ``sensor_msgs::JointState``

Force-torque (6-axis)
=====================

Message type: ``geometry_msgs::WrenchStamped``

Odometry
========

Message type: ``nav_msgs::Odometry``

IMU
===

Message type: ``sensor_msgs::Imu``

GPS
===

Message type: ``sensor_msgs::NavSatFix``

Doppler velocity log (DVL)
==========================

Message type: ``cola2_msgs::DVL``

Second message type: ``sensor_msgs::Range``

Pressure
========

Message type: ``sensor_msgs::FluidPressure``

Multi-beam
==========

Message type: ``sensor_msgs::LaserScan``

Color camera
============

Message type: ``sensor_msgs::Image``

Second message type: ``sensor_msgs::CameraInfo``

Depth camera
============

Message type: ``sensor_msgs::Image``

Second message type: ``sensor_msgs::CameraInfo``

Forward-looking sonar (FLS)
===========================

Message type: ``sensor_msgs::Image``

Second message type: ``sensor_msgs::Image``

Mechanical scanning imaging sonar (MSIS)
========================================

Message type: ``sensor_msgs::Image``

Second message type: ``sensor_msgs::Image``

Side-scan sonar (SSS)
=====================

Message type: ``sensor_msgs::Image``

Second message type: ``sensor_msgs::Image``

