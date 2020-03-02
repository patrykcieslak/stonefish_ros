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

#include <Stonefish/core/Robot.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/utils/SystemUtil.hpp>

namespace sf
{

ROSSimulationManager::ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath) 
	: SimulationManager(stepsPerSecond, SolverType::SOLVER_SI, CollisionFilteringType::COLLISION_EXCLUSIVE, FluidDynamicsType::GEOMETRY_BASED), scnFilePath(scenarioFilePath), nh("~")
{
    srvECurrents = nh.advertiseService("enable_currents", &ROSSimulationManager::EnableCurrents, this);
    srvDCurrents = nh.advertiseService("disable_currents", &ROSSimulationManager::DisableCurrents, this);
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
            if(pubs.find(sensor->getName()) == pubs.end())
                continue;

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

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    unsigned int cID = 0;
    Contact* cnt;
    while((cnt = getContact(cID++)) != NULL)
    {
        if(!cnt->isNewDataAvailable())
            continue;

        if(pubs.find(cnt->getName()) != pubs.end())
        {
            ROSInterface::PublishContact(pubs[cnt->getName()], cnt);
            cnt->MarkDataOld();
        }
    }   

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(rosRobots[i]->publishBaseLinkTransform)
            ROSInterface::PublishTF(br, rosRobots[i]->robot->getTransform(), ros::Time::now(), "world_ned", rosRobots[i]->robot->getName() + "/base_link");
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
                    //ROS_INFO("[Thruster %d] Setpoint: %1.3lf Omega: %1.3lf Thrust: %1.3lf", thID, ((Thruster*)actuator)->getSetpoint(), ((Thruster*)actuator)->getOmega(), ((Thruster*)actuator)->getThrust());
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
                    if(pubs.find(actuator->getName()) != pubs.end())
                    {
                        std_msgs::Float64 msg;
                        msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                        pubs[actuator->getName()].publish(msg);
                    }
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

void ROSSimulationManager::MultibeamScanReady(Multibeam2* mb)
{
    ROSInterface::PublishPointCloud(pubs[mb->getName()], mb);
}

void ROSSimulationManager::FLSScanReady(FLS* fls)
{
    ROSInterface::PublishFLS(pubs[fls->getName()], fls);
}

bool ROSSimulationManager::EnableCurrents(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    getOcean()->EnableCurrents();
    res.message = "Currents simulation enabled.";
    res.success = true;
    return true;
}

bool ROSSimulationManager::DisableCurrents(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    getOcean()->DisableCurrents();
    res.message = "Currents simulation disabled.";
    res.success = true;
    return true;
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
