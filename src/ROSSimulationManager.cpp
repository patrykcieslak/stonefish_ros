/*    
    This file is a part of stonefish_ros.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROSSimulationManager.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 17/09/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSInterface.h"

#include <core/Robot.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/GPS.h>
#include <sensors/scalar/ForceTorque.h>
#include <sensors/scalar/RotaryEncoder.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/DepthCamera.h>
#include <actuators/Thruster.h>
#include <actuators/Propeller.h>
#include <actuators/Servo.h>
#include <utils/SystemUtil.hpp>

namespace sf
{

ROSSimulationManager::ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath) 
	: SimulationManager(stepsPerSecond, SolverType::SOLVER_SI, CollisionFilteringType::COLLISION_EXCLUSIVE, FluidDynamicsType::GEOMETRY_BASED), scnFilePath(scenarioFilePath)
{
}

ros::NodeHandle& ROSSimulationManager::getNodeHandle()
{
    return nh;
}

std::map<std::string, ros::Publisher>& ROSSimulationManager::getPublishers()
{
    return pubs;
}

std::map<std::string, ros::Subscriber>& ROSSimulationManager::getSubscribers()
{
    return subs;
}

void ROSSimulationManager::BuildScenario()
{
    ROSScenarioParser parser(this);
    parser.Parse(scnFilePath);
}

void ROSSimulationManager::AddROSRobot(ROSRobot* robot)
{
    rosRobots.push_back(robot);
}

void ROSSimulationManager::SimulationStepCompleted(Scalar timeStep)
{
	////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int sID = 0;
    Sensor* sensor;
    while((sensor = getSensor(sID++)) != NULL)
    {
        if(!sensor->isNewDataAvailable())
            continue;

        if(sensor->getType() != SensorType::SENSOR_VISION)
        {
            switch(((ScalarSensor*)sensor)->getScalarSensorType())
            {
                case ScalarSensorType::SENSOR_ODOM:
                    ROSInterface::PublishOdometry(pubs[sensor->getName()], (Odometry*)sensor);
                    break;

                case ScalarSensorType::SENSOR_IMU:
                    ROSInterface::PublishIMU(pubs[sensor->getName()], (IMU*)sensor);
                    break;

                case ScalarSensorType::SENSOR_DVL:
                    ROSInterface::PublishDVL(pubs[sensor->getName()], pubs[sensor->getName() + "/altitude"], (DVL*)sensor);
                    break;

                case ScalarSensorType::SENSOR_GPS:
                    ROSInterface::PublishGPS(pubs[sensor->getName()], (GPS*)sensor);
                    break;

                case ScalarSensorType::SENSOR_PRESSURE:
                    ROSInterface::PublishPressure(pubs[sensor->getName()], (Pressure*)sensor);
                    break;

                case ScalarSensorType::SENSOR_FT:
                    ROSInterface::PublishForceTorque(pubs[sensor->getName()], (ForceTorque*)sensor);
                    break;

                case ScalarSensorType::SENSOR_ENCODER:
                    ROSInterface::PublishEncoder(pubs[sensor->getName()], (RotaryEncoder*)sensor);
                    break;

                case ScalarSensorType::SENSOR_MULTIBEAM:
                    ROSInterface::PublishLaserScan(pubs[sensor->getName()], (Multibeam*)sensor);
                    break;

                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }   

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if (rosRobots[i]->publishBaseLinkTransform)
        {
            Transform transform = rosRobots[i]->robot->getTransform();
            transform.setRotation(transform.getRotation()*btQuaternion(0.0, 0.0, 1.5708));
            sf::ROSInterface::PublishTF(br, transform, ros::Time::now(), "world_ned", rosRobots[i]->robot->getName() + "/base_link");
        }
    }

    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(rosRobots[i]->servoSetpoints.size() == 0 
           || pubs.find(rosRobots[i]->robot->getName() + "/servos") == pubs.end())
            continue;

        unsigned int aID = 0;
        unsigned int sID = 0;
        Actuator* actuator;
        Servo* srv;

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = rosRobots[i]->robot->getName();
        msg.name.resize(rosRobots[i]->servoSetpoints.size());
        msg.position.resize(rosRobots[i]->servoSetpoints.size());
        msg.velocity.resize(rosRobots[i]->servoSetpoints.size());
        msg.effort.resize(rosRobots[i]->servoSetpoints.size());

        while((actuator = rosRobots[i]->robot->getActuator(aID++)) != NULL)
        {
            if(actuator->getType() == ActuatorType::ACTUATOR_SERVO)
            {
                srv = (Servo*)actuator;
                msg.name[sID] = srv->getJointName();
                msg.position[sID] = srv->getPosition();
                msg.velocity[sID] = srv->getVelocity();
                msg.effort[sID] = srv->getEffort();
                ++sID;

                if(sID == rosRobots[i]->servoSetpoints.size())
                    break;
            }
        }

        pubs[rosRobots[i]->robot->getName() + "/servos"].publish(msg);
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        
        while((actuator = rosRobots[i]->robot->getActuator(aID++)) != NULL)
        {
            switch(actuator->getType())
            {
                case ActuatorType::ACTUATOR_THRUSTER:
                    ((Thruster*)actuator)->setSetpoint(rosRobots[i]->thrusterSetpoints[thID++]);
                    break;

                case ActuatorType::ACTUATOR_PROPELLER:
                    ((Propeller*)actuator)->setSetpoint(rosRobots[i]->propellerSetpoints[propID++]);
                    break;

                case ActuatorType::ACTUATOR_SERVO:
                {
                    Scalar setpoint = rosRobots[i]->servoSetpoints.at(((Servo*)actuator)->getJointName());
                    
                    if(rosRobots[i]->servoVelocityMode)
                    {
                        ((Servo*)actuator)->setControlMode(ServoControlMode::VELOCITY_CTRL);
                        ((Servo*)actuator)->setDesiredVelocity(setpoint);
                    }
                    else
                    {
                        ((Servo*)actuator)->setControlMode(ServoControlMode::POSITION_CTRL);
                        ((Servo*)actuator)->setDesiredPosition(setpoint);
                    }
                }
                    break;

                case ActuatorType::ACTUATOR_VBS:
                {
                    std_msgs::Float64 msg;
                    msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                    pubs[actuator->getName()].publish(msg);
                }
                    break;

                default:
                    break;
            }
        }
    }

	ros::spinOnce();
}

void ROSSimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
	ROSInterface::PublishCamera(pubs[cam->getName()], pubs[cam->getName() + "/info"], cam);
}

void ROSSimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    ROSInterface::PublishPointCloud(pubs[cam->getName()], cam);
}

ThrustersCallback::ThrustersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void ThrustersCallback::operator()(const cola2_msgs::SetpointsConstPtr& msg)
{   
    if(msg->setpoints.size() != robot->thrusterSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of thruster setpoints for robot: " << robot->robot->getName());
        return;
    }

    for(size_t i=0; i<robot->thrusterSetpoints.size(); ++i)
        robot->thrusterSetpoints[i] = msg->setpoints[i];
}

PropellersCallback::PropellersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void PropellersCallback::operator()(const cola2_msgs::SetpointsConstPtr& msg)
{
    if(msg->setpoints.size() != robot->propellerSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of propeller setpoints for robot: " << robot->robot->getName());
        return;
    }

    for(size_t i=0; i<robot->propellerSetpoints.size(); ++i)
        robot->propellerSetpoints[i] = msg->setpoints[i];   
}

ServosCallback::ServosCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{

}

void ServosCallback::operator()(const sensor_msgs::JointStateConstPtr& msg)
{
    if(msg->name.size() == 0)
    {
        ROS_ERROR("Desired joint state message is missing joint names!");
        return;
    }

    if(msg->position.size() > 0)
    {
        robot->servoVelocityMode = false;
        for(unsigned int i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints.at(msg->name[i]) = msg->position[i];
            }
            catch(const std::out_of_range& e)
            {
                ROS_WARN_STREAM("Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        robot->servoVelocityMode = true;
        for(unsigned int i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot->servoSetpoints.at(msg->name[i]) = msg->velocity[i];
            }
            catch(const std::out_of_range& e)
            {
                ROS_WARN_STREAM("Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->effort.size() > 0)
    {
        ROS_ERROR("No effort control mode implemented in simulation!");
    }
}

VBSCallback::VBSCallback(VariableBuoyancy* act) : act(act)
{
}

void VBSCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{   
    act->setFlowRate(msg->data);
}

}
