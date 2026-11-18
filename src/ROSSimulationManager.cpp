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
//  Copyright (c) 2019-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSInterface.h"
#include "stonefish_ros/ROSSensorPluginInterface.h"
#include "stonefish_ros/ROSActuatorPluginInterface.h"
#include "stonefish_ros/ROSCommPluginInterface.h"
#include "stonefish_ros/ThrusterState.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Lidar.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/comms/OpticalModem.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/Light.h>
#include <Stonefish/joints/FixedJoint.h>
#include <Stonefish/utils/SystemUtil.hpp>

#include <ros/file_log.h>

namespace sf
{

ROSSimulationManager::ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath)
	: SimulationManager(stepsPerSecond, Solver::SI, CollisionFilter::EXCLUSIVE), scnFilePath_(scenarioFilePath), nh_("~"), it_(nh_), spinner_(4)
{
}

uint64_t ROSSimulationManager::GetSimulationClock() const
{
    return ros::Time::now().toNSec()/1000;
}

void ROSSimulationManager::SimulationClockSleep(uint64_t us)
{
    ros::Duration(0, (int32_t)us*1000).sleep();
}

ros::NodeHandle& ROSSimulationManager::GetNodeHandle()
{
    return nh_;
}

image_transport::ImageTransport& ROSSimulationManager::GetImageTransportHandle()
{
    return it_;
}

std::unordered_map<std::string, ros::ServiceServer>& ROSSimulationManager::GetServiceServers()
{
    return srvs_;
}

std::unordered_map<std::string, ros::Publisher>& ROSSimulationManager::GetPublishers()
{
    return pubs_;
}

std::unordered_map<std::string, image_transport::Publisher>& ROSSimulationManager::GetImagePublishers()
{
    return imgPubs_;
}

std::unordered_map<std::string, ros::Subscriber>& ROSSimulationManager::GetSubscribers()
{
    return subs_;
}

std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& ROSSimulationManager::GetCameraMsgPrototypes()
{
    return cameraMsgPrototypes_;
}

std::unordered_map<std::string, std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr>>& ROSSimulationManager::GetDualImageCameraMsgPrototypes()
{
    return dualImageCameraMsgPrototypes_;
}

std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& ROSSimulationManager::GetSonarMsgPrototypes()
{
    return sonarMsgPrototypes_;
}

std::vector<std::unique_ptr<ROSControlInterface>>& ROSSimulationManager::GetControlInterfaces()
{
    return controlIfs_;
}

void ROSSimulationManager::BuildScenario()
{
    //Run parser
    ROSScenarioParser parser(this);
    bool success = parser.Parse(scnFilePath_);

    //Save log
    std::string logFilePath = ros::file_log::getLogDirectory() + "/stonefish_ros_parser.log";
    bool success2 = parser.SaveLog(logFilePath);

    if(!success)
    {
        ROS_ERROR("Parsing of scenario file '%s' failed!", scnFilePath_.c_str());
        if(success2)
            ROS_ERROR("For more information check the parser log file '%s'.", logFilePath.c_str());
    }

    if(!success2)
        ROS_ERROR("Parser log file '%s' could not be saved!", logFilePath.c_str());

    srvECurrents_ = nh_.advertiseService("enable_currents", &ROSSimulationManager::EnableCurrents, this);
    srvDCurrents_ = nh_.advertiseService("disable_currents", &ROSSimulationManager::DisableCurrents, this);
    srvRespawn_ = nh_.advertiseService("respawn_robot", &ROSSimulationManager::RespawnRobot, this);
    spinner_.start();
}

void ROSSimulationManager::DestroyScenario()
{
    spinner_.stop();

    //Destroy ROS components
    for(auto it = srvs_.begin(); it != srvs_.end(); ++it)
        it->second.shutdown();
    for(auto it = pubs_.begin(); it != pubs_.end(); ++it)
        it->second.shutdown();
    for(auto it = imgPubs_.begin(); it != imgPubs_.end(); ++it)
        it->second.shutdown();
    for(auto it = subs_.begin(); it != subs_.end(); ++it)
        it->second.shutdown();
    srvs_.clear();
    pubs_.clear();
    imgPubs_.clear();
    subs_.clear();
    cameraMsgPrototypes_.clear();
    dualImageCameraMsgPrototypes_.clear();
    sonarMsgPrototypes_.clear();
    rosRobots_.clear();
    controlIfs_.clear();

    //Destroy simulation entities
    SimulationManager::DestroyScenario();
}

void ROSSimulationManager::AddROSRobot(std::unique_ptr<ROSRobot> robot)
{
    rosRobots_.push_back(std::move(robot));
}

bool ROSSimulationManager::RespawnROSRobot(std::string_view robotName, const Transform& origin)
{
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->robot->GetName() == robotName)
        {
            rosRobots_[i]->respawnOrigin = origin;
            rosRobots_[i]->respawnRequested = true;
            return true;
        }
    }
    return false;
}

void ROSSimulationManager::SimulationStepCompleted(Scalar timeStep)
{
	////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = GetSensor(id++)) != nullptr)
    {
        if(!sensor->IsNewDataAvailable())
            continue;

        if(sensor->GetType() != SensorType::VISION)
        {
            if(pubs_.find(sensor->GetName()) == pubs_.end())
                continue;

            switch(static_cast<ScalarSensor*>(sensor)->GetScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    ROSInterface::PublishAccelerometer(pubs_.at(sensor->GetName()), (Accelerometer*)sensor);
                    break;
                case ScalarSensorType::GYRO:
                    ROSInterface::PublishGyroscope(pubs_.at(sensor->GetName()), (Gyroscope*)sensor);
                    break;
                case ScalarSensorType::IMU:
                    ROSInterface::PublishIMU(pubs_.at(sensor->GetName()), (IMU*)sensor);
                    break;
                case ScalarSensorType::ODOM:
                    ROSInterface::PublishOdometry(pubs_.at(sensor->GetName()), (Odometry*)sensor);
                    break;
                case ScalarSensorType::DVL:
                {
                    ROSInterface::PublishDVL(pubs_.at(sensor->GetName()), (DVL*)sensor);
                    if(pubs_.find(sensor->GetName() + "/altitude") != pubs_.end())
                        ROSInterface::PublishDVLAltitude(pubs_.at(sensor->GetName() + "/altitude"), (DVL*)sensor);
                }
                    break;
                case ScalarSensorType::INS:
                {
                    ROSInterface::PublishINS(pubs_.at(sensor->GetName()), (INS*)sensor);
                    if(pubs_.find(sensor->GetName() + "/odometry") != pubs_.end())
                        ROSInterface::PublishINSOdometry(pubs_.at(sensor->GetName() + "/odometry"), (INS*)sensor);
                }
                    break;
                case ScalarSensorType::GPS:
                    ROSInterface::PublishGPS(pubs_.at(sensor->GetName()), (GPS*)sensor);
                    break;
                case ScalarSensorType::PRESSURE:
                    ROSInterface::PublishPressure(pubs_.at(sensor->GetName()), (Pressure*)sensor);
                    break;
                case ScalarSensorType::FT:
                    ROSInterface::PublishForceTorque(pubs_.at(sensor->GetName()), (ForceTorque*)sensor);
                    break;
                case ScalarSensorType::ENCODER:
                    ROSInterface::PublishEncoder(pubs_.at(sensor->GetName()), (RotaryEncoder*)sensor);
                    break;
                case ScalarSensorType::MULTIBEAM:
                {
                    ROSInterface::PublishMultibeam(pubs_.at(sensor->GetName()), (Multibeam*)sensor);
                    if(pubs_.find(sensor->GetName() + "/pcl") != pubs_.end())
                        ROSInterface::PublishMultibeamPCL(pubs_.at(sensor->GetName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;
                case ScalarSensorType::PROFILER:
                    ROSInterface::PublishProfiler(pubs_.at(sensor->GetName()), (Profiler*)sensor);
                    break;
                case ScalarSensorType::PLUGIN:
                {
                    ROSSensorPluginInterface* iface = dynamic_cast<ROSSensorPluginInterface*>(sensor);
                    if (iface != nullptr)
                        iface->Publish(nh_, pubs_);
                }
                    break;
                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }

    ///////////////////////////////////////COMMS///////////////////////////////////////////////////
    id = 0;
    Comm* comm;
    while((comm = GetComm(id++)) != nullptr)
    {
        if(pubs_.find(comm->GetName()) == pubs_.end())
            continue;

        switch(comm->GetType())
        {
            case CommType::ACOUSTIC:
            {
                std_msgs::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    pubs_.at(comm->GetName()).publish(msg);
                }
            }
                break;
            case CommType::USBL:
            {
                if(comm->IsNewDataAvailable())
                {
                    ROSInterface::PublishUSBL(pubs_.at(comm->GetName()), pubs_.at(comm->GetName() + "/beacon_info"), (USBL*)comm);
                    comm->MarkDataOld();
                }

                std_msgs::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    pubs_.at(comm->GetName() + "/received_data").publish(msg);
                }
            }
                break;
            case CommType::OPTICAL:
            {
                std_msgs::Float64 msg;
                msg.data = static_cast<OpticalModem*>(comm)->GetReceptionQuality();
                pubs_.at(comm->GetName()).publish(msg);

                std_msgs::String msg2;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg2.data = std::string(message->data.begin(), message->data.end());
                    pubs_.at(comm->GetName() + "/received_data").publish(msg2);
                }
            }
                break;
            case CommType::PLUGIN:
            {
                ROSCommPluginInterface* iface = dynamic_cast<ROSCommPluginInterface*>(comm);
                if (iface != nullptr)
                    iface->Publish(nh_, pubs_);
            }
                break;
            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = GetEntity(id++)) != nullptr)
    {
        if(ent->GetType() == EntityType::ANIMATED)
        {
            if(pubs_.find(ent->GetName() + "/odometry") == pubs_.end())
                continue;

            ROSInterface::PublishTrajectoryState(pubs_.at(ent->GetName() + "/odometry"), pubs_.at(ent->GetName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = GetContact(id++)) != nullptr)
    {
        if(!cnt->IsNewDataAvailable())
            continue;

        if(pubs_.find(cnt->GetName()) != pubs_.end())
        {
            ROSInterface::PublishContact(pubs_[cnt->GetName()], cnt);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->publishBaseLinkTransform)
            ROSInterface::PublishTF(br_, rosRobots_[i]->robot->GetTransform(), ros::Time(GetSimulationTime(true)), "world_ned", rosRobots_[i]->robot->GetName() + "/base_link");
    }

    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot->GetName() + "/motors") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots_[i]->robot->GetName();
            
            while((actuator = rosRobots_[i]->robot->GetActuator(aID++)) != nullptr && actuator->GetType() == ActuatorType::JOINT)
            {
                if(static_cast<JointActuator*>(actuator)->GetJointActuatorType() == JointActuatorType::MOTOR)
                {
                    Motor* mtr = static_cast<Motor*>(actuator);
                    msg.name.push_back(mtr->GetJointName());
                    msg.position.push_back(mtr->GetAngle());
                    msg.velocity.push_back(mtr->GetAngularVelocity());
                    msg.effort.push_back(mtr->GetTorque());
                }
            }
            if(msg.name.size() > 0)
                pubs_.at(rosRobots_[i]->robot->GetName() + "/motors").publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot->GetName() + "/servos") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots_[i]->robot->GetName();
            
            while((actuator = rosRobots_[i]->robot->GetActuator(aID++)) != nullptr && actuator->GetType() == ActuatorType::JOINT)
            {
                if(static_cast<JointActuator*>(actuator)->GetJointActuatorType() == JointActuatorType::SERVO)
                {
                    Servo* srv = static_cast<Servo*>(actuator);
                    msg.name.push_back(srv->GetJointName());
                    msg.position.push_back(srv->GetPosition());
                    msg.velocity.push_back(srv->GetVelocity());
                    msg.effort.push_back(srv->GetEffort());
                }
            }
            if(msg.name.size() > 0)
                pubs_.at(rosRobots_[i]->robot->GetName() + "/servos").publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot->GetName() + "/rudders") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots_[i]->robot->GetName();
            
            while((actuator = rosRobots_[i]->robot->GetActuator(aID++)) != nullptr && actuator->GetType() == ActuatorType::LINK)
            {
                if(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType() == LinkActuatorType::RUDDER)
                {
                    Rudder* rudder = static_cast<Rudder*>(actuator);
                    msg.name.push_back(rudder->GetName());
                    msg.position.push_back(rudder->GetSetpoint());
                    msg.velocity.push_back(0.0);
                    msg.effort.push_back(0.0);
                }
            }
            if(msg.name.size() > 0)
                pubs_.at(rosRobots_[i]->robot->GetName() + "/rudders").publish(msg);
        }

        if(rosRobots_[i]->thrusterSetpoints.size() != 0
           && pubs_.find(rosRobots_[i]->robot->GetName() + "/thrusters") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros::ThrusterState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots_[i]->robot->GetName();
            msg.setpoint.resize(rosRobots_[i]->thrusterSetpoints.size());
            msg.rpm.resize(rosRobots_[i]->thrusterSetpoints.size());
            msg.thrust.resize(rosRobots_[i]->thrusterSetpoints.size());
            msg.torque.resize(rosRobots_[i]->thrusterSetpoints.size());

            while((actuator = rosRobots_[i]->robot->GetActuator(aID++)) != nullptr && actuator->GetType() == ActuatorType::LINK)
            {
                if(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType() == LinkActuatorType::THRUSTER)
                {
                    Thruster* th = static_cast<Thruster*>(actuator);
                    msg.setpoint[thID] = th->GetSetpoint();
                    msg.rpm[thID] = th->GetOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->GetThrust();
                    msg.torque[thID] = th->GetTorque();
                    ++thID;    
                }
                else if(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType() == LinkActuatorType::SIMPLE_THRUSTER)
                {
                    SimpleThruster* th = static_cast<SimpleThruster*>(actuator);
                    msg.setpoint[thID] = th->GetThrust();
                    msg.rpm[thID] = Scalar(0);
                    msg.thrust[thID] = th->GetThrust();
                    msg.torque[thID] = th->GetTorque();
                    ++thID;
                }

                if(thID == rosRobots_[i]->thrusterSetpoints.size())
                        break;
            }
            pubs_.at(rosRobots_[i]->robot->GetName() + "/thrusters").publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;

        while((actuator = rosRobots_[i]->robot->GetActuator(aID++)) != nullptr)
        {
            if (actuator->GetType() == ActuatorType::LINK)
            {
                switch(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType())
                {
                    case LinkActuatorType::PUSH:
                    {
                        Push* push = static_cast<Push*>(actuator);
                        if (rosRobots_[i]->thrusterSetpointsChanged)
                            push->SetForce(rosRobots_[i]->thrusterSetpoints[thID++]);

                        auto it = pubs_.find(actuator->GetName());
                        if(it != pubs_.end())
                        {
                            geometry_msgs::WrenchStamped msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = push->GetName();
                            msg.wrench.force.x = push->GetForce();
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::SIMPLE_THRUSTER:
                    {
                        SimpleThruster* th = static_cast<SimpleThruster*>(actuator);
                        if (rosRobots_[i]->thrusterSetpointsChanged)
                            th->SetSetpoint(rosRobots_[i]->thrusterSetpoints[thID++], Scalar(0));

                        auto it = pubs_.find(actuator->GetName()+"/wrench");
                        if(it != pubs_.end())
                        {
                            geometry_msgs::WrenchStamped msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = th->GetName();
                            msg.wrench.force.x = th->GetThrust();
                            msg.wrench.torque.x = th->GetTorque();
                            it->second.publish(msg);
                        }            

                        it = pubs_.find(actuator->GetName()+"/joint_state");
                        if(it != pubs_.end())
                        {
                            //Publish propeller rotation for visualization
                            sensor_msgs::JointState msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = th->GetName();
                            msg.name.push_back(th->GetName()+"/propeller");
                            msg.position.push_back(th->GetAngle());  
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::THRUSTER:
                    {
                        Thruster* th = static_cast<Thruster*>(actuator);
                        if (rosRobots_[i]->thrusterSetpointsChanged)
                            th->SetSetpoint(rosRobots_[i]->thrusterSetpoints[thID++]);

                        auto it = pubs_.find(actuator->GetName()+"/wrench");
                        if(it != pubs_.end())
                        {
                            geometry_msgs::WrenchStamped msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = th->GetName();
                            msg.wrench.force.x = th->GetThrust();
                            msg.wrench.torque.x = th->GetTorque();
                            it->second.publish(msg);
                        }            

                        it = pubs_.find(actuator->GetName()+"/joint_state");
                        if(it != pubs_.end())
                        {
                            //Publish propeller rotation for visualization
                            sensor_msgs::JointState msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = th->GetName();
                            msg.name.push_back(th->GetName()+"/propeller");
                            msg.position.push_back(th->GetAngle());  
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::PROPELLER:
                    {
                        Propeller* prop = static_cast<Propeller*>(actuator);
                        if (rosRobots_[i]->propellerSetpointsChanged)
                            prop->SetSetpoint(rosRobots_[i]->propellerSetpoints[propID++]);

                        auto it = pubs_.find(actuator->GetName());
                        if(it != pubs_.end())
                        {
                            geometry_msgs::WrenchStamped msg;
                            msg.header.stamp = ros::Time(GetSimulationTime(true));
                            msg.header.frame_id = prop->GetName();
                            msg.wrench.force.x = prop->GetThrust();
                            msg.wrench.torque.x = prop->GetTorque();
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::RUDDER:
                    {
                        if (rosRobots_[i]->rudderSetpointsChanged)
                            static_cast<Rudder*>(actuator)->SetSetpoint(rosRobots_[i]->rudderSetpoints[rudderID++]);
                    }
                        break;
                    case LinkActuatorType::VBS:
                    {
                        auto it = pubs_.find(actuator->GetName());
                        if(it != pubs_.end())
                        {
                            std_msgs::Float64 msg;
                            msg.data = static_cast<VariableBuoyancy*>(actuator)->GetLiquidVolume();
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::SUCTION_CUP:
                    {
                        auto it = pubs_.find(actuator->GetName());
                        if(it != pubs_.end())
                        {
                            std_msgs::Bool msg;
                            msg.data = static_cast<SuctionCup*>(actuator)->GetPump();
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::PLUGIN:
                    {
                        ROSActuatorPluginInterface* iface = dynamic_cast<ROSActuatorPluginInterface*>(actuator);
                        if (iface != nullptr)
                            iface->Publish(nh_, pubs_);
                    }
                        break;
                    default:
                        break;
                }
            }
            else if (actuator->GetType() == ActuatorType::JOINT)
            {
                switch (static_cast<JointActuator*>(actuator)->GetJointActuatorType())
                {
                    case JointActuatorType::MOTOR:
                    {
                        if(rosRobots_[i]->servoSetpoints.size() == 0)
                            continue;

                        Motor* mtr = static_cast<Motor*>(actuator);
                        auto it = rosRobots_[i]->servoSetpoints.find(mtr->GetJointName());
                        if(it != rosRobots_[i]->servoSetpoints.end())
                        {
                            if(it->second.first == ServoControlMode::TORQUE)
                            {
                                mtr->SetTorque(it->second.second);
                            }
                        }
                    }
                        break;
                    case JointActuatorType::SERVO:
                    {
                        if(rosRobots_[i]->servoSetpoints.size() == 0)
                            continue;

                        Servo* srv = static_cast<Servo*>(actuator);

                        auto it = rosRobots_[i]->servoSetpoints.find(srv->GetJointName());
                        if(it != rosRobots_[i]->servoSetpoints.end())
                        {
                            if(it->second.first == ServoControlMode::VELOCITY)
                            {
                                srv->SetControlMode(ServoControlMode::VELOCITY);
                                srv->SetDesiredVelocity(it->second.second);
                            }
                            else if(it->second.first == ServoControlMode::POSITION)
                            {
                                srv->SetControlMode(ServoControlMode::POSITION);
                                srv->SetDesiredPosition(it->second.second);
                            }
                        }
                    }
                        break;
                    case JointActuatorType::PLUGIN:
                    {
                        ROSActuatorPluginInterface* iface = dynamic_cast<ROSActuatorPluginInterface*>(actuator);
                        if (iface != nullptr)
                            iface->Publish(nh_, pubs_);
                    }
                        break;
                    default:
                        break;
                }
            }
        }

        //Reset change flags
        rosRobots_[i]->thrusterSetpointsChanged = false;
        rosRobots_[i]->propellerSetpointsChanged = false;
        rosRobots_[i]->rudderSetpointsChanged = false;
    }

    ////////////////////////////////////// ROS CONTROL ////////////////////////////////////////////
    for(size_t i=0; i<controlIfs_.size(); ++i)
    {
        controlIfs_[i]->read();
        controlIfs_[i]->update(ros::Time(GetSimulationTime(true)), ros::Duration(timeStep));
        controlIfs_[i]->write();
    }

    /////////////////////////////////// RESPAWN REQUESTS ///////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->respawnRequested)
        {
            rosRobots_[i]->robot->Respawn(this, rosRobots_[i]->respawnOrigin);
            rosRobots_[i]->respawnRequested = false;
        }
    }    

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot->GetName() + "/debug/physics") != pubs_.end())
        {
            Robot* r = rosRobots_[i]->robot;
            size_t lID = 0;
            SolidEntity* link;
            stonefish_ros::DebugPhysics msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            
            auto& debugPub = pubs_.at(r->GetName() + "/debug/physics");

            while((link = r->GetLink(lID++)) != nullptr)
            {
                msg.header.frame_id = link->GetName();
                
                msg.mass = link->GetMass();
                msg.volume = link->GetVolume();
                msg.surface = link->GetSurface();
                Vector3 inertia = link->GetInertia();
                msg.inertia.x = inertia.getX();
                msg.inertia.y = inertia.getY();
                msg.inertia.z = inertia.getZ();

                Vector3 cog = -link->GetCG2OTransform().getOrigin();
                msg.cog.x = cog.getX();
                msg.cog.y = cog.getY();
                msg.cog.z = cog.getZ();
                
                Vector3 cob = link->GetCG2OTransform() * link->GetCb();
                msg.cob.x = cob.getX();
                msg.cob.y = cob.getY();
                msg.cob.z = cob.getZ();
                
                Matrix3 toOrigin = link->GetOTransform().getBasis().inverse();

                Vector3 vel = toOrigin * link->GetLinearVelocity();
                Vector3 avel = toOrigin * link->GetAngularVelocity();
                msg.velocity.linear.x = vel.getX();
                msg.velocity.linear.y = vel.getY();
                msg.velocity.linear.z = vel.getZ();
                msg.velocity.angular.x = avel.getX();
                msg.velocity.angular.y = avel.getY();
                msg.velocity.angular.z = avel.getZ();

                Vector3 Fb, Tb, Fd, Td, Ff, Tf;
                link->GetHydrodynamicForces(Fb, Tb, Fd, Td, Ff, Tf);            
                Vector3 Cd, Cf;
                link->GetHydrodynamicCoefficients(Cd, Cf);
                msg.damping_coeff.x = Cd.getX();
                msg.damping_coeff.y = Cd.getY();
                msg.damping_coeff.z = Cd.getZ();
                msg.skin_friction_coeff.x = Cf.getX();
                msg.skin_friction_coeff.y = Cf.getY();
                msg.skin_friction_coeff.z = Cf.getZ();

                // Fb = toOrigin * Fb;
                // Tb = toOrigin * Tb;
                msg.buoyancy.force.x = Fb.getX();
                msg.buoyancy.force.y = Fb.getY();
                msg.buoyancy.force.z = Fb.getZ();
                msg.buoyancy.torque.x = Tb.getX();
                msg.buoyancy.torque.y = Tb.getY();
                msg.buoyancy.torque.z = Tb.getZ();

                Fd = toOrigin * Fd;
                Td = toOrigin * Td;
                msg.damping.force.x = Fd.getX();
                msg.damping.force.y = Fd.getY();
                msg.damping.force.z = Fd.getZ();
                msg.damping.torque.x = Td.getX();
                msg.damping.torque.y = Td.getY();
                msg.damping.torque.z = Td.getZ();

                Ff = toOrigin * Ff;
                Tf = toOrigin * Tf;
                msg.skin_friction.force.x = Ff.getX();
                msg.skin_friction.force.y = Ff.getY();
                msg.skin_friction.force.z = Ff.getZ();
                msg.skin_friction.torque.x = Tf.getX();
                msg.skin_friction.torque.y = Tf.getY();
                msg.skin_friction.torque.z = Tf.getZ();

                msg.wetted_surface = link->GetWettedSurface();
                msg.submerged_volume = link->GetSubmergedVolume();

                debugPub.publish(msg);
            }
        }
    }
}

void ROSSimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes_[cam->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), (uint8_t*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes_[cam->GetName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    pubs_.at(cam->GetName() + "/info").publish(info);
}


void ROSSimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes_[cam->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes_[cam->GetName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    pubs_.at(cam->GetName() + "/info").publish(info);
}

void ROSSimulationManager::ThermalCameraImageReady(ThermalCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    pubs_.at(cam->GetName() + "/info").publish(info);
    imgPubs_.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::OpticalFlowCameraImageReady(OpticalFlowCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    pubs_.at(cam->GetName() + "/info").publish(info);
    imgPubs_.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::SegmentationCameraImageReady(SegmentationCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint16_t*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    pubs_.at(cam->GetName() + "/info").publish(info);
    imgPubs_.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::EventBasedCameraOutputReady(EventBasedCamera* cam)
{
    ROSInterface::PublishEventBasedCamera(pubs_.at(cam->GetName()), cam);
}

void ROSSimulationManager::LidarScanReady(Lidar* lidar)
{
    ROSInterface::PublishLidar(pubs_.at(lidar->GetName()), lidar);
}

void ROSSimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes_[fls->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), fls->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes_[fls->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), fls->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(fls->GetName()).publish(img);
    imgPubs_.at(fls->GetName() + "/display").publish(disp);
}

void ROSSimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes_[sss->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), sss->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes_[sss->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), sss->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(sss->GetName()).publish(img);
    imgPubs_.at(sss->GetName() + "/display").publish(disp);
}

void ROSSimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes_[msis->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), msis->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes_[msis->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), msis->GetDisplayDataPointer(), disp->step * disp->height);

    //Fill in the laser scan message for the last beam
    Scalar currentAngle = (msis->GetCurrentRotationStep() * msis->GetRotationStepAngle()) * M_PI / 180.0;
    unsigned int currentBeamIndex = msis->GetCurrentBeamIndex();

    sensor_msgs::LaserScan laserscan;
    laserscan.header.stamp = img->header.stamp;
    laserscan.header.frame_id = msis->GetName();
    laserscan.angle_min = currentAngle;
    laserscan.angle_max = currentAngle;
    laserscan.angle_increment = 0.0;
    laserscan.time_increment = 0.0;
    laserscan.range_min = msis->GetRangeMin();
    laserscan.range_max = msis->GetRangeMax();
    laserscan.ranges.resize(img->height);
    laserscan.intensities.resize(img->height);

    for(unsigned int i=0; i<img->height; ++i)
    {
        laserscan.ranges[i] = (laserscan.range_max - laserscan.range_min) * (img->height-1-i)/Scalar(img->height-1) + laserscan.range_min;
        laserscan.intensities[i] = img->data[img->step * i + currentBeamIndex];
    }

    //Publish messages
    imgPubs_.at(msis->GetName()).publish(img);
    imgPubs_.at(msis->GetName() + "/display").publish(disp);
    pubs_.at(msis->GetName() + "/beam").publish(laserscan);
}
        
bool ROSSimulationManager::EnableCurrents(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    GetOcean()->EnableVelocityFields();
    res.message = "Currents simulation enabled.";
    res.success = true;
    return true;
}

bool ROSSimulationManager::DisableCurrents(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    GetOcean()->DisableVelocityFields();
    res.message = "Currents simulation disabled.";
    res.success = true;
    return true;
}

bool ROSSimulationManager::RespawnRobot(stonefish_ros::Respawn::Request& req, stonefish_ros::Respawn::Response& res)
{
    Vector3 p(req.origin.position.x, req.origin.position.y, req.origin.position.z);
    Quaternion q(req.origin.orientation.x, req.origin.orientation.y, req.origin.orientation.z, req.origin.orientation.w);
    Transform origin(q, p);
    
    if(RespawnROSRobot(req.name, origin))
    {
        res.message = "Robot respawned.";
        res.success = true;
    }
    else    
    {
        res.message = "Robot not found!";
        res.success = false;
    }
    return res.success;
}

UniformVFCallback::UniformVFCallback(Uniform* vf) : vf_(vf)
{
}

void UniformVFCallback::operator()(const geometry_msgs::Vector3ConstPtr& msg)
{
    vf_->SetVelocity(Vector3(msg->x, msg->y, msg->z));
}

JetVFCallback::JetVFCallback(Jet* vf) : vf_(vf)
{
}

void JetVFCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    vf_->SetOutletVelocity(msg->data);
}

ActuatorOriginCallback::ActuatorOriginCallback(Actuator* act) : act_(act)
{
}

void ActuatorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    if(act_->GetType() == ActuatorType::LINK)
        static_cast<LinkActuator*>(act_)->SetRelativeActuatorFrame(T);
}

TrajectoryCallback::TrajectoryCallback(ManualTrajectory* tr) : tr_(tr)
{
}

void TrajectoryCallback::operator()(const nav_msgs::OdometryConstPtr& msg)
{
    Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr_->SetTransform(Transform(q, p));
    tr_->SetLinearVelocity(v);
    tr_->SetAngularVelocity(omega);
}

PushCallback::PushCallback(Push* act) : act_(act)
{
}

void PushCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act_->SetForce(msg->data);
}

SimpleThrusterCallback::SimpleThrusterCallback(SimpleThruster* act) : act_(act)
{
}

void SimpleThrusterCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act_->SetSetpoint(msg->data, Scalar(0));
}

ThrusterCallback::ThrusterCallback(Thruster* act) : act_(act)
{
}

void ThrusterCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act_->SetSetpoint(msg->data);
}

PropellerCallback::PropellerCallback(Propeller* act) : act_(act)
{
}

void PropellerCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act_->SetSetpoint(msg->data);
}

VBSCallback::VBSCallback(VariableBuoyancy* act) : act_(act)
{
}

void VBSCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act_->SetFlowRate(msg->data);
}

SensorOriginCallback::SensorOriginCallback(Sensor* sens) : sens_(sens)
{
}

void SensorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens_->GetType())
    {
        case SensorType::LINK:
            static_cast<LinkSensor*>(sens_)->SetRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            static_cast<VisionSensor*>(sens_)->SetRelativeSensorFrame(T);
            break;

        default:
            ROS_WARN_STREAM("Live update of origin frame of sensor '" << sens_->GetName() << "' not supported!");
            break;
    }
}

CommCallback::CommCallback(Comm* comm) : comm_(comm)
{
}

void CommCallback::operator()(const std_msgs::StringConstPtr& msg)
{
    comm_->SendMessage(msg->data);
}

SensorService::SensorService(Sensor* sens) : sens_(sens)
{
}

bool SensorService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    sens_->SetEnabled(req.data);
    if(req.data)
        res.message = "Sensor turned on.";
    else
        res.message = "Sensor turned off.";
    res.success = true;  
    return true;
}

FLSService::FLSService(FLS* fls) : fls_(fls)
{
}

bool FLSService::operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res)
{
    if(req.range_min <= 0 || req.range_max <= 0 || req.gain <= 0 || req.range_min >= req.range_max)
    {
        res.success = false;
        res.message = "Wrong sonar settings!";
    }
    else
    {
        fls_->SetRangeMax(req.range_max);
        fls_->SetRangeMin(req.range_min);
        fls_->SetGain(req.gain);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

SSSService::SSSService(SSS* sss) : sss_(sss)
{
}

bool SSSService::operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res)
{
    if(req.range_min <= 0 || req.range_max <= 0 || req.gain <= 0 || req.range_min >= req.range_max)
    {
        res.success = false;
        res.message = "Wrong sonar settings!";
    }
    else
    {
        sss_->SetRangeMax(req.range_max);
        sss_->SetRangeMin(req.range_min);
        sss_->SetGain(req.gain);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

MSISService::MSISService(MSIS* msis) : msis_(msis)
{
}

bool MSISService::operator()(stonefish_ros::SonarSettings2::Request& req, stonefish_ros::SonarSettings2::Response& res)
{
    if(req.range_min <= 0 || req.range_max <= 0 || req.gain <= 0
       || req.range_min >= req.range_max
       || req.rotation_min < -180.0
       || req.rotation_max > 180.0
       || req.rotation_min >= req.rotation_max)
    {
        res.success = false;
        res.message = "Wrong sonar settings!";
    }
    else
    {
        msis_->SetRangeMax(req.range_max);
        msis_->SetRangeMin(req.range_min);
        msis_->SetGain(req.gain);
        msis_->SetRotationLimits(req.rotation_min, req.rotation_max);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

SuctionCupService::SuctionCupService(SuctionCup* suction) : suction_(suction)
{
}

bool SuctionCupService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    suction_->SetPump(req.data);
    if(req.data)
        res.message = "Pump turned on.";
    else 
        res.message = "Pump turned off.";
    res.success = true;
    return true;
}

ThrustersCallback::ThrustersCallback(ROSRobot* robot) : robot_(robot)
{
}

void ThrustersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot_->thrusterSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of thruster setpoints for robot: " << robot_->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot_->thrusterSetpoints.size(); ++i)
        robot_->thrusterSetpoints[i] = msg->data[i];
    robot_->thrusterSetpointsChanged = true;
}

PropellersCallback::PropellersCallback(ROSRobot* robot) : robot_(robot)
{
}

void PropellersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot_->propellerSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of propeller setpoints for robot: " << robot_->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot_->propellerSetpoints.size(); ++i)
        robot_->propellerSetpoints[i] = msg->data[i];
    robot_->propellerSetpointsChanged = true;
}

RuddersCallback::RuddersCallback(ROSRobot* robot) : robot_(robot)
{
}

void RuddersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot_->rudderSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of rudder setpoints for robot: " << robot_->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot_->rudderSetpoints.size(); ++i)
        robot_->rudderSetpoints[i] = msg->data[i];
    robot_->rudderSetpointsChanged = true;
}

ServosCallback::ServosCallback(ROSRobot* robot) : robot_(robot)
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
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot_->servoSetpoints.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
            }
            catch(const std::out_of_range& e)
            {
                ROS_WARN_STREAM("Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        for(size_t i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot_->servoSetpoints.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
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

JointGroupCallback::JointGroupCallback(ROSRobot* robot, ServoControlMode mode, const std::vector<std::string>& jointNames)
    : robot_(robot), mode_(mode), jointNames_(jointNames)
{
    //Generate setpoint placeholders
    for(size_t i=0; i<jointNames_.size(); ++i)
        robot_->servoSetpoints[jointNames_[i]] = std::make_pair(mode_, Scalar(0));
}

void JointGroupCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != jointNames_.size())
    {
        ROS_ERROR_STREAM("Wrong size of joint group message! Required: " << jointNames_.size() << " Received: " << msg->data.size());
        return;
    }

    for(size_t i=0; i<jointNames_.size(); ++i)
    {
        try
        {
            robot_->servoSetpoints.at(jointNames_[i]) = std::make_pair(mode_, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            ROS_WARN_STREAM("Invalid joint name: " << jointNames_[i]);
        }
    }
}

JointCallback::JointCallback(ROSRobot* robot, ServoControlMode mode, const std::string& jointName)
    : robot_(robot), mode_(mode), jointName_(jointName)
{
    //Generate setpoint placeholder
    robot->servoSetpoints[jointName_] = std::make_pair(mode_, Scalar(0));
}

void JointCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    try
    {
        robot_->servoSetpoints.at(jointName_) = std::make_pair(mode_, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        ROS_WARN_STREAM("Invalid joint name: " << jointName_);
    }
}

GlueService::GlueService(ROSSimulationManager* sm, FixedJoint* fix) : sm_(sm), fix_(fix)
{
}

bool GlueService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    if(req.data)
    {
        fix_->RemoveFromSimulation(sm_);
        fix_->UpdateDefinition();
        fix_->AddToSimulation(sm_);
        res.message = "Glue activated.";
    }
    else
    { 
        fix_->RemoveFromSimulation(sm_);  
        res.message = "Glue deactivated.";
    }
    res.success = true;
    return true;
}

LightService::LightService(Light* l) : l_(l)
{
}

bool LightService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    l_->Switch(req.data);
    if(req.data)
        res.message = "Light turned on.";
    else
        res.message = "Light turned off.";
    res.success = true;
    return true;
}

}
