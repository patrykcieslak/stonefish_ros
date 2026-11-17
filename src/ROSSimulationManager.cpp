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
	: SimulationManager(stepsPerSecond, Solver::SI, CollisionFilter::EXCLUSIVE), scnFilePath(scenarioFilePath), nh("~"), it(nh), spinner(4)
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
    return nh;
}

image_transport::ImageTransport& ROSSimulationManager::GetImageTransportHandle()
{
    return it;
}

std::unordered_map<std::string, ros::ServiceServer>& ROSSimulationManager::GetServiceServers()
{
    return srvs;
}

std::unordered_map<std::string, ros::Publisher>& ROSSimulationManager::GetPublishers()
{
    return pubs;
}

std::unordered_map<std::string, image_transport::Publisher>& ROSSimulationManager::GetImagePublishers()
{
    return imgPubs;
}

std::unordered_map<std::string, ros::Subscriber>& ROSSimulationManager::GetSubscribers()
{
    return subs;
}

std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& ROSSimulationManager::GetCameraMsgPrototypes()
{
    return cameraMsgPrototypes;
}

std::unordered_map<std::string, std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr>>& ROSSimulationManager::GetDualImageCameraMsgPrototypes()
{
    return dualImageCameraMsgPrototypes;
}

std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& ROSSimulationManager::GetSonarMsgPrototypes()
{
    return sonarMsgPrototypes;
}

std::vector<std::unique_ptr<ROSControlInterface>>& ROSSimulationManager::GetControlInterfaces()
{
    return controlIfs;
}

void ROSSimulationManager::BuildScenario()
{
    //Run parser
    ROSScenarioParser parser(this);
    bool success = parser.Parse(scnFilePath);

    //Save log
    std::string logFilePath = ros::file_log::getLogDirectory() + "/stonefish_ros_parser.log";
    bool success2 = parser.SaveLog(logFilePath);

    if(!success)
    {
        ROS_ERROR("Parsing of scenario file '%s' failed!", scnFilePath.c_str());
        if(success2)
            ROS_ERROR("For more information check the parser log file '%s'.", logFilePath.c_str());
    }

    if(!success2)
        ROS_ERROR("Parser log file '%s' could not be saved!", logFilePath.c_str());

    srvECurrents = nh.advertiseService("enable_currents", &ROSSimulationManager::EnableCurrents, this);
    srvDCurrents = nh.advertiseService("disable_currents", &ROSSimulationManager::DisableCurrents, this);
    srvRespawn = nh.advertiseService("respawn_robot", &ROSSimulationManager::RespawnRobot, this);
    spinner.start();
}

void ROSSimulationManager::DestroyScenario()
{
    spinner.stop();

    //Destroy ROS components
    for(auto it = srvs.begin(); it != srvs.end(); ++it)
        it->second.shutdown();
    for(auto it = pubs.begin(); it != pubs.end(); ++it)
        it->second.shutdown();
    for(auto it = imgPubs.begin(); it != imgPubs.end(); ++it)
        it->second.shutdown();
    for(auto it = subs.begin(); it != subs.end(); ++it)
        it->second.shutdown();
    srvs.clear();
    pubs.clear();
    imgPubs.clear();
    subs.clear();
    cameraMsgPrototypes.clear();
    dualImageCameraMsgPrototypes.clear();
    sonarMsgPrototypes.clear();
    rosRobots.clear();
    controlIfs.clear();

    //Destroy simulation entities
    SimulationManager::DestroyScenario();
}

void ROSSimulationManager::AddROSRobot(std::unique_ptr<ROSRobot> robot)
{
    rosRobots.push_back(std::move(robot));
}

bool ROSSimulationManager::RespawnROSRobot(std::string_view robotName, const Transform& origin)
{
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(rosRobots[i]->robot->GetName() == robotName)
        {
            rosRobots[i]->respawnOrigin = origin;
            rosRobots[i]->respawnRequested = true;
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
            if(pubs.find(sensor->GetName()) == pubs.end())
                continue;

            switch(static_cast<ScalarSensor*>(sensor)->GetScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    ROSInterface::PublishAccelerometer(pubs.at(sensor->GetName()), (Accelerometer*)sensor);
                    break;
                case ScalarSensorType::GYRO:
                    ROSInterface::PublishGyroscope(pubs.at(sensor->GetName()), (Gyroscope*)sensor);
                    break;
                case ScalarSensorType::IMU:
                    ROSInterface::PublishIMU(pubs.at(sensor->GetName()), (IMU*)sensor);
                    break;
                case ScalarSensorType::ODOM:
                    ROSInterface::PublishOdometry(pubs.at(sensor->GetName()), (Odometry*)sensor);
                    break;
                case ScalarSensorType::DVL:
                {
                    ROSInterface::PublishDVL(pubs.at(sensor->GetName()), (DVL*)sensor);
                    if(pubs.find(sensor->GetName() + "/altitude") != pubs.end())
                        ROSInterface::PublishDVLAltitude(pubs.at(sensor->GetName() + "/altitude"), (DVL*)sensor);
                }
                    break;
                case ScalarSensorType::INS:
                {
                    ROSInterface::PublishINS(pubs.at(sensor->GetName()), (INS*)sensor);
                    if(pubs.find(sensor->GetName() + "/odometry") != pubs.end())
                        ROSInterface::PublishINSOdometry(pubs.at(sensor->GetName() + "/odometry"), (INS*)sensor);
                }
                    break;
                case ScalarSensorType::GPS:
                    ROSInterface::PublishGPS(pubs.at(sensor->GetName()), (GPS*)sensor);
                    break;
                case ScalarSensorType::PRESSURE:
                    ROSInterface::PublishPressure(pubs.at(sensor->GetName()), (Pressure*)sensor);
                    break;
                case ScalarSensorType::FT:
                    ROSInterface::PublishForceTorque(pubs.at(sensor->GetName()), (ForceTorque*)sensor);
                    break;
                case ScalarSensorType::ENCODER:
                    ROSInterface::PublishEncoder(pubs.at(sensor->GetName()), (RotaryEncoder*)sensor);
                    break;
                case ScalarSensorType::MULTIBEAM:
                {
                    ROSInterface::PublishMultibeam(pubs.at(sensor->GetName()), (Multibeam*)sensor);
                    if(pubs.find(sensor->GetName() + "/pcl") != pubs.end())
                        ROSInterface::PublishMultibeamPCL(pubs.at(sensor->GetName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;
                case ScalarSensorType::PROFILER:
                    ROSInterface::PublishProfiler(pubs.at(sensor->GetName()), (Profiler*)sensor);
                    break;
                case ScalarSensorType::PLUGIN:
                {
                    ROSSensorPluginInterface* iface = dynamic_cast<ROSSensorPluginInterface*>(sensor);
                    if (iface != nullptr)
                        iface->Publish(nh, pubs);
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
        if(pubs.find(comm->GetName()) == pubs.end())
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
                    pubs.at(comm->GetName()).publish(msg);
                }
            }
                break;

            case CommType::USBL:
            {
                if(comm->IsNewDataAvailable())
                {
                    ROSInterface::PublishUSBL(pubs.at(comm->GetName()), pubs.at(comm->GetName() + "/beacon_info"), (USBL*)comm);
                    comm->MarkDataOld();
                }

                std_msgs::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    pubs.at(comm->GetName() + "/received_data").publish(msg);
                }
            }
                break;

            case CommType::OPTICAL:
            {
                std_msgs::Float64 msg;
                msg.data = static_cast<OpticalModem*>(comm)->GetReceptionQuality();
                pubs.at(comm->GetName()).publish(msg);

                std_msgs::String msg2;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg2.data = std::string(message->data.begin(), message->data.end());
                    pubs.at(comm->GetName() + "/received_data").publish(msg2);
                }
            }
                break;
            case CommType::PLUGIN:
            {
                ROSCommPluginInterface* iface = dynamic_cast<ROSCommPluginInterface*>(comm);
                if (iface != nullptr)
                    iface->Publish(nh, pubs);
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
            if(pubs.find(ent->GetName() + "/odometry") == pubs.end())
                continue;

            ROSInterface::PublishTrajectoryState(pubs.at(ent->GetName() + "/odometry"), pubs.at(ent->GetName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = GetContact(id++)) != nullptr)
    {
        if(!cnt->IsNewDataAvailable())
            continue;

        if(pubs.find(cnt->GetName()) != pubs.end())
        {
            ROSInterface::PublishContact(pubs[cnt->GetName()], cnt);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(rosRobots[i]->publishBaseLinkTransform)
            ROSInterface::PublishTF(br, rosRobots[i]->robot->GetTransform(), ros::Time(GetSimulationTime(true)), "world_ned", rosRobots[i]->robot->GetName() + "/base_link");
    }

    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(pubs.find(rosRobots[i]->robot->GetName() + "/servos") != pubs.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            Servo* srv;
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots[i]->robot->GetName();
            
            while((actuator = rosRobots[i]->robot->GetActuator(aID++)) != nullptr)
            {
                if(actuator->GetType() == ActuatorType::JOINT 
                    && static_cast<JointActuator*>(actuator)->GetJointActuatorType() == JointActuatorType::SERVO)
                {
                    srv = static_cast<Servo*>(actuator);
                    msg.name.push_back(srv->GetJointName());
                    msg.position.push_back(srv->GetPosition());
                    msg.velocity.push_back(srv->GetVelocity());
                    msg.effort.push_back(srv->GetEffort());
                }
            }
            if(msg.name.size() > 0)
                pubs.at(rosRobots[i]->robot->GetName() + "/servos").publish(msg);
        }

        if(rosRobots[i]->thrusterSetpoints.size() != 0
           && pubs.find(rosRobots[i]->robot->GetName() + "/thrusters") != pubs.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros::ThrusterState msg;
            msg.header.stamp = ros::Time(GetSimulationTime(true));
            msg.header.frame_id = rosRobots[i]->robot->GetName();
            msg.setpoint.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.rpm.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.thrust.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.torque.resize(rosRobots[i]->thrusterSetpoints.size());

            while((actuator = rosRobots[i]->robot->GetActuator(aID++)) != nullptr)
            {
                if(actuator->GetType() == ActuatorType::LINK)
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

                    if(thID == rosRobots[i]->thrusterSetpoints.size())
                            break;
                }
            }
            pubs.at(rosRobots[i]->robot->GetName() + "/thrusters").publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;

        while((actuator = rosRobots[i]->robot->GetActuator(aID++)) != nullptr)
        {
            if (actuator->GetType() == ActuatorType::LINK)
            {
                switch(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType())
                {
                    case LinkActuatorType::PUSH:
                        static_cast<Push*>(actuator)->SetForce(rosRobots[i]->thrusterSetpoints[thID++]);
                        break;
                    case LinkActuatorType::SIMPLE_THRUSTER:
                        static_cast<SimpleThruster*>(actuator)->SetSetpoint(rosRobots[i]->thrusterSetpoints[thID++], Scalar(0));
                        break;
                    case LinkActuatorType::THRUSTER:
                        static_cast<Thruster*>(actuator)->SetSetpoint(rosRobots[i]->thrusterSetpoints[thID++]);
                        break;
                    case LinkActuatorType::PROPELLER:
                        static_cast<Propeller*>(actuator)->SetSetpoint(rosRobots[i]->propellerSetpoints[propID++]);
                        break;
                    case LinkActuatorType::RUDDER:
                        static_cast<Rudder*>(actuator)->SetSetpoint(rosRobots[i]->rudderSetpoints[rudderID++]);
                        break;
                    case LinkActuatorType::VBS:
                    {
                        auto it = pubs.find(actuator->GetName());
                        if(it != pubs.end())
                        {
                            std_msgs::Float64 msg;
                            msg.data = static_cast<VariableBuoyancy*>(actuator)->GetLiquidVolume();
                            it->second.publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::SUCTION_CUP:
                    {
                        auto it = pubs.find(actuator->GetName());
                        if(it != pubs.end())
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
                            iface->Publish(nh, pubs);
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
                        if(rosRobots[i]->servoSetpoints.size() == 0)
                            continue;

                        Motor* mtr = static_cast<Motor*>(actuator);
                        auto it = rosRobots[i]->servoSetpoints.find(mtr->GetJointName());
                        if(it != rosRobots[i]->servoSetpoints.end())
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
                        if(rosRobots[i]->servoSetpoints.size() == 0)
                            continue;

                        Servo* srv = static_cast<Servo*>(actuator);

                        auto it = rosRobots[i]->servoSetpoints.find(srv->GetJointName());
                        if(it != rosRobots[i]->servoSetpoints.end())
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
                            iface->Publish(nh, pubs);
                    }
                        break;
                    default:
                        break;
                }
            }
        }
    }

    ////////////////////////////////////// ROS CONTROL ////////////////////////////////////////////
    for(size_t i=0; i<controlIfs.size(); ++i)
    {
        controlIfs[i]->read();
        controlIfs[i]->update(ros::Time(GetSimulationTime(true)), ros::Duration(timeStep));
        controlIfs[i]->write();
    }

    /////////////////////////////////// RESPAWN REQUESTS ///////////////////////////////
    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        if(rosRobots[i]->respawnRequested)
        {
            rosRobots[i]->robot->Respawn(this, rosRobots[i]->respawnOrigin);
            rosRobots[i]->respawnRequested = false;
        }
    }    

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    auto hydroLambda = [](Robot* r)
    {
        std::stringstream output;
        size_t lID = 0;
        SolidEntity* link;
        Vector3 Fb, Tb, Fd, Td, Fs, Ts;
    
        while((link = r->GetLink(lID++)) != nullptr)
        {
            link->GetHydrodynamicForces(Fb, Tb, Fd, Td, Fs, Ts);
            output << "[" << link->GetName() << "]" << std::endl;
            output << "Buoyancy force[N]: " << Fb.x() << ", " << Fb.y() << ", " << Fb.z() << std::endl;
            output << "Damping force[N]: " << Fd.x() << ", " << Fd.y() << ", " << Fd.z() << std::endl;
            output << "Skin friction[N]: " << Fs.x() << ", " << Fs.y() << ", " << Fs.z() << std::endl;
        }
        return output.str();
    };

    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        Robot* r = rosRobots[i]->robot;
        ROS_DEBUG_STREAM_NAMED("Hydrodynamics", "===== Hydrodynamics for " << r->GetName() << " =====\n" << hydroLambda(r));
    }
}

void ROSSimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes[cam->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), (uint8_t*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes[cam->GetName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs.at(cam->GetName()).publish(img);
    pubs.at(cam->GetName() + "/info").publish(info);
}


void ROSSimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes[cam->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes[cam->GetName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs.at(cam->GetName()).publish(img);
    pubs.at(cam->GetName() + "/info").publish(info);
}

void ROSSimulationManager::ThermalCameraImageReady(ThermalCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs.at(cam->GetName()).publish(img);
    pubs.at(cam->GetName() + "/info").publish(info);
    imgPubs.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::OpticalFlowCameraImageReady(OpticalFlowCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs.at(cam->GetName()).publish(img);
    pubs.at(cam->GetName() + "/info").publish(info);
    imgPubs.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::SegmentationCameraImageReady(SegmentationCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = std::get<0>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint16_t*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = std::get<1>(dualImageCameraMsgPrototypes[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::ImagePtr img2 = std::get<2>(dualImageCameraMsgPrototypes[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs.at(cam->GetName()).publish(img);
    pubs.at(cam->GetName() + "/info").publish(info);
    imgPubs.at(cam->GetName()+"/display").publish(img2);
}

void ROSSimulationManager::EventBasedCameraOutputReady(EventBasedCamera* cam)
{
    ROSInterface::PublishEventBasedCamera(pubs.at(cam->GetName()), cam);
}

void ROSSimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[fls->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), fls->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[fls->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), fls->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs.at(fls->GetName()).publish(img);
    imgPubs.at(fls->GetName() + "/display").publish(disp);
}

void ROSSimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[sss->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), sss->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[sss->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), sss->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs.at(sss->GetName()).publish(img);
    imgPubs.at(sss->GetName() + "/display").publish(disp);
}

void ROSSimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[msis->GetName()].first;
    img->header.stamp = ros::Time(GetSimulationTime(true));
    memcpy(img->data.data(), msis->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[msis->GetName()].second;
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
    imgPubs.at(msis->GetName()).publish(img);
    imgPubs.at(msis->GetName() + "/display").publish(disp);
    pubs.at(msis->GetName() + "/beam").publish(laserscan);
}

void ROSSimulationManager::LidarScanReady(Lidar* lidar)
{
    ROSInterface::PublishLidar(pubs.at(lidar->GetName()), lidar);
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

UniformVFCallback::UniformVFCallback(Uniform* vf) : vf(vf)
{
}

void UniformVFCallback::operator()(const geometry_msgs::Vector3ConstPtr& msg)
{
    vf->SetVelocity(Vector3(msg->x, msg->y, msg->z));
}

JetVFCallback::JetVFCallback(Jet* vf) : vf(vf)
{
}

void JetVFCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    vf->SetOutletVelocity(msg->data);
}

ThrustersCallback::ThrustersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void ThrustersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot->thrusterSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of thruster setpoints for robot: " << robot->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot->thrusterSetpoints.size(); ++i)
        robot->thrusterSetpoints[i] = msg->data[i];
}

PropellersCallback::PropellersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void PropellersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot->propellerSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of propeller setpoints for robot: " << robot->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot->propellerSetpoints.size(); ++i)
        robot->propellerSetpoints[i] = msg->data[i];
}

RuddersCallback::RuddersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void RuddersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot->rudderSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of rudder setpoints for robot: " << robot->robot->GetName());
        return;
    }

    for(size_t i=0; i<robot->rudderSetpoints.size(); ++i)
        robot->rudderSetpoints[i] = msg->data[i];
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
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
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
                robot->servoSetpoints.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
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

JointGroupCallback::JointGroupCallback(ROSSimulationManager* sm, ROSRobot* robot, ServoControlMode mode, const std::vector<std::string>& jointNames)
    : sm(sm), robot(robot), mode(mode), jointNames(jointNames)
{
    //Generate setpoint placeholders
    for(size_t i=0; i<jointNames.size(); ++i)
        robot->servoSetpoints[jointNames[i]] = std::make_pair(mode, Scalar(0));
}

void JointGroupCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != jointNames.size())
    {
        ROS_ERROR_STREAM("Wrong size of joint group message! Required: " << jointNames.size() << " Received: " << msg->data.size());
        return;
    }

    for(size_t i=0; i<jointNames.size(); ++i)
    {
        try
        {
            robot->servoSetpoints.at(jointNames[i]) = std::make_pair(mode, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            ROS_WARN_STREAM("Invalid joint name: " << jointNames[i]);
        }
    }
}

JointCallback::JointCallback(ROSSimulationManager* sm, ROSRobot* robot, ServoControlMode mode, const std::string& jointName)
    : sm(sm), robot(robot), mode(mode), jointName(jointName)
{
    //Generate setpoint placeholder
    robot->servoSetpoints[jointName] = std::make_pair(mode, Scalar(0));
}

void JointCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    try
    {
        robot->servoSetpoints.at(jointName) = std::make_pair(mode, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        ROS_WARN_STREAM("Invalid joint name: " << jointName);
    }
}

PushCallback::PushCallback(Push* act) : act(act)
{
}

void PushCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->SetForce(msg->data);
}

SimpleThrusterCallback::SimpleThrusterCallback(SimpleThruster* act) : act(act)
{
}

void SimpleThrusterCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->SetSetpoint(msg->data, Scalar(0));
}

ThrusterCallback::ThrusterCallback(Thruster* act) : act(act)
{
}

void ThrusterCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->SetSetpoint(msg->data);
}

PropellerCallback::PropellerCallback(Propeller* act) : act(act)
{
}

void PropellerCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->SetSetpoint(msg->data);
}

VBSCallback::VBSCallback(VariableBuoyancy* act) : act(act)
{
}

void VBSCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->SetFlowRate(msg->data);
}

TrajectoryCallback::TrajectoryCallback(ManualTrajectory* tr) : tr(tr)
{
}

void TrajectoryCallback::operator()(const nav_msgs::OdometryConstPtr& msg)
{
    Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr->SetTransform(Transform(q, p));
    tr->SetLinearVelocity(v);
    tr->SetAngularVelocity(omega);
}

ActuatorOriginCallback::ActuatorOriginCallback(Actuator* act) : act(act)
{
}

void ActuatorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    if(act->GetType() == ActuatorType::LINK)
        static_cast<LinkActuator*>(act)->SetRelativeActuatorFrame(T);
}

SensorOriginCallback::SensorOriginCallback(Sensor* sens) : sens(sens)
{
}

void SensorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->GetType())
    {
        case SensorType::LINK:
            static_cast<LinkSensor*>(sens)->SetRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            static_cast<VisionSensor*>(sens)->SetRelativeSensorFrame(T);
            break;

        default:
            ROS_WARN_STREAM("Live update of origin frame of sensor '" << sens->GetName() << "' not supported!");
            break;
    }
}

CommCallback::CommCallback(Comm* comm) : comm(comm)
{
}

void CommCallback::operator()(const std_msgs::StringConstPtr& msg)
{
    comm->SendMessage(msg->data);
}

SensorService::SensorService(Sensor* sens) : sens(sens)
{
}

bool SensorService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    sens->SetEnabled(req.data);
    if(req.data)
        res.message = "Sensor turned on.";
    else
        res.message = "Sensor turned off.";
    res.success = true;  
    return true;
}

FLSService::FLSService(FLS* fls) : fls(fls)
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
        fls->SetRangeMax(req.range_max);
        fls->SetRangeMin(req.range_min);
        fls->SetGain(req.gain);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

SSSService::SSSService(SSS* sss) : sss(sss)
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
        sss->SetRangeMax(req.range_max);
        sss->SetRangeMin(req.range_min);
        sss->SetGain(req.gain);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

MSISService::MSISService(MSIS* msis) : msis(msis)
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
        msis->SetRangeMax(req.range_max);
        msis->SetRangeMin(req.range_min);
        msis->SetGain(req.gain);
        msis->SetRotationLimits(req.rotation_min, req.rotation_max);
        res.success = true;
        res.message = "New sonar settings applied.";
    }
    return true;
}

SuctionCupService::SuctionCupService(SuctionCup* suction) : suction(suction)
{
}

bool SuctionCupService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    suction->SetPump(req.data);
    if(req.data)
        res.message = "Pump turned on.";
    else 
        res.message = "Pump turned off.";
    res.success = true;
    return true;
}

LightService::LightService(Light* l) : l(l)
{
}

bool LightService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    l->Switch(req.data);
    if(req.data)
        res.message = "Light turned on.";
    else
        res.message = "Light turned off.";
    res.success = true;
    return true;
}

GlueService::GlueService(ROSSimulationManager* sm, FixedJoint* fix) : sm(sm), fix(fix)
{
}

bool GlueService::operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    if(req.data)
    {
        fix->RemoveFromSimulation(sm);
        fix->UpdateDefinition();
        fix->AddToSimulation(sm);
        res.message = "Glue activated.";
    }
    else
    { 
        fix->RemoveFromSimulation(sm);  
        res.message = "Glue deactivated.";
    }
    res.success = true;
    return true;
}

}
