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
//  Copyright (c) 2019-2023 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSInterface.h"
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
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/utils/SystemUtil.hpp>

#include <ros/file_log.h>

namespace sf
{

ROSSimulationManager::ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath)
	: SimulationManager(stepsPerSecond, SolverType::SOLVER_SI, CollisionFilteringType::COLLISION_EXCLUSIVE), scnFilePath(scenarioFilePath), nh("~"), it(nh), spinner(4)
{
    srvECurrents = nh.advertiseService("enable_currents", &ROSSimulationManager::EnableCurrents, this);
    srvDCurrents = nh.advertiseService("disable_currents", &ROSSimulationManager::DisableCurrents, this);
}

ROSSimulationManager::~ROSSimulationManager()
{
}

uint64_t ROSSimulationManager::getSimulationClock() const
{
    return ros::Time::now().toNSec()/1000;
}

void ROSSimulationManager::SimulationClockSleep(uint64_t us)
{
    ros::Duration(0, (int32_t)us*1000).sleep();
}

ros::NodeHandle& ROSSimulationManager::getNodeHandle()
{
    return nh;
}

image_transport::ImageTransport& ROSSimulationManager::getImageTransportHandle()
{
    return it;
}

std::map<std::string, ros::ServiceServer>& ROSSimulationManager::getServiceServers()
{
    return srvs;
}

std::map<std::string, ros::Publisher>& ROSSimulationManager::getPublishers()
{
    return pubs;
}

std::map<std::string, image_transport::Publisher>& ROSSimulationManager::getImagePublishers()
{
    return imgPubs;
}

std::map<std::string, ros::Subscriber>& ROSSimulationManager::getSubscribers()
{
    return subs;
}

std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& ROSSimulationManager::getCameraMsgPrototypes()
{
    return cameraMsgPrototypes;
}

std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& ROSSimulationManager::getSonarMsgPrototypes()
{
    return sonarMsgPrototypes;
}

std::vector<ROSControlInterface*>& ROSSimulationManager::getControlInterfaces()
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
    sonarMsgPrototypes.clear();
    for(size_t i = 0; i<rosRobots.size(); ++i)
        delete rosRobots[i];
    rosRobots.clear();
    for(size_t i = 0; i<controlIfs.size(); ++i)
        delete controlIfs[i];
    controlIfs.clear();

    //Destroy simulation entities
    SimulationManager::DestroyScenario();
}

void ROSSimulationManager::AddROSRobot(ROSRobot* robot)
{
    rosRobots.push_back(robot);
}

void ROSSimulationManager::SimulationStepCompleted(Scalar timeStep)
{
	////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = getSensor(id++)) != nullptr)
    {
        if(!sensor->isNewDataAvailable())
            continue;

        if(sensor->getType() != SensorType::VISION)
        {
            if(pubs.find(sensor->getName()) == pubs.end())
                continue;

            switch(((ScalarSensor*)sensor)->getScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    ROSInterface::PublishAccelerometer(pubs.at(sensor->getName()), (Accelerometer*)sensor);
                    break;

                case ScalarSensorType::GYRO:
                    ROSInterface::PublishGyroscope(pubs.at(sensor->getName()), (Gyroscope*)sensor);
                    break;

                case ScalarSensorType::IMU:
                    ROSInterface::PublishIMU(pubs.at(sensor->getName()), (IMU*)sensor);
                    break;

                case ScalarSensorType::ODOM:
                    ROSInterface::PublishOdometry(pubs.at(sensor->getName()), (Odometry*)sensor);
                    break;

                case ScalarSensorType::DVL:
                {
                    ROSInterface::PublishDVL(pubs.at(sensor->getName()), (DVL*)sensor);
                    if(pubs.find(sensor->getName() + "/altitude") != pubs.end())
                        ROSInterface::PublishDVLAltitude(pubs.at(sensor->getName() + "/altitude"), (DVL*)sensor);
                }
                    break;

                case ScalarSensorType::INS:
                {
                    ROSInterface::PublishINS(pubs.at(sensor->getName()), (INS*)sensor);
                    if(pubs.find(sensor->getName() + "/odometry") != pubs.end())
                        ROSInterface::PublishINSOdometry(pubs.at(sensor->getName() + "/odometry"), (INS*)sensor);
                }
                    break;

                case ScalarSensorType::GPS:
                    ROSInterface::PublishGPS(pubs.at(sensor->getName()), (GPS*)sensor);
                    break;

                case ScalarSensorType::PRESSURE:
                    ROSInterface::PublishPressure(pubs.at(sensor->getName()), (Pressure*)sensor);
                    break;

                case ScalarSensorType::FT:
                    ROSInterface::PublishForceTorque(pubs.at(sensor->getName()), (ForceTorque*)sensor);
                    break;

                case ScalarSensorType::ENCODER:
                    ROSInterface::PublishEncoder(pubs.at(sensor->getName()), (RotaryEncoder*)sensor);
                    break;

                case ScalarSensorType::MULTIBEAM:
                {
                    ROSInterface::PublishMultibeam(pubs.at(sensor->getName()), (Multibeam*)sensor);
                    if(pubs.find(sensor->getName() + "/pcl") != pubs.end())
                        ROSInterface::PublishMultibeamPCL(pubs.at(sensor->getName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;

                case ScalarSensorType::PROFILER:
                    ROSInterface::PublishProfiler(pubs.at(sensor->getName()), (Profiler*)sensor);
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
    while((comm = getComm(id++)) != nullptr)
    {
        if(!comm->isNewDataAvailable())
            continue;

        if(pubs.find(comm->getName()) == pubs.end())
            continue;

        switch(comm->getType())
        {
            case CommType::USBL:
                ROSInterface::PublishUSBL(pubs.at(comm->getName()), pubs.at(comm->getName() + "/beacon_info"), (USBL*)comm);
                comm->MarkDataOld();
                break;

            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = getEntity(id++)) != nullptr)
    {
        if(ent->getType() == EntityType::ANIMATED)
        {
            if(pubs.find(ent->getName() + "/odometry") == pubs.end())
                continue;

            ROSInterface::PublishTrajectoryState(pubs.at(ent->getName() + "/odometry"), pubs.at(ent->getName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = getContact(id++)) != nullptr)
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
        if(pubs.find(rosRobots[i]->robot->getName() + "/servos") != pubs.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            Servo* srv;
            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = rosRobots[i]->robot->getName();
            
            while((actuator = rosRobots[i]->robot->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::SERVO)
                {
                    srv = (Servo*)actuator;
                    msg.name.push_back(srv->getJointName());
                    msg.position.push_back(srv->getPosition());
                    msg.velocity.push_back(srv->getVelocity());
                    msg.effort.push_back(srv->getEffort());
                }
            }
            if(msg.name.size() > 0)
                pubs.at(rosRobots[i]->robot->getName() + "/servos").publish(msg);
        }

        if(rosRobots[i]->thrusterSetpoints.size() != 0
           && pubs.find(rosRobots[i]->robot->getName() + "/thrusters") != pubs.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros::ThrusterState msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = rosRobots[i]->robot->getName();
            msg.setpoint.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.rpm.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.thrust.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.torque.resize(rosRobots[i]->thrusterSetpoints.size());

            while((actuator = rosRobots[i]->robot->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::THRUSTER)
                {
                    Thruster* th = (Thruster*)actuator;
                    msg.setpoint[thID] = th->getSetpoint();
                    msg.rpm[thID] = th->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;    
                }
                else if(actuator->getType() == ActuatorType::SIMPLE_THRUSTER)
                {
                    SimpleThruster* th = (SimpleThruster*)actuator;
                    msg.setpoint[thID] = th->getThrust();
                    msg.rpm[thID] = Scalar(0);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;
                }

                if(thID == rosRobots[i]->thrusterSetpoints.size())
                        break;
            }
            pubs.at(rosRobots[i]->robot->getName() + "/thrusters").publish(msg);
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

        while((actuator = rosRobots[i]->robot->getActuator(aID++)) != nullptr)
        {
            switch(actuator->getType())
            {
                case ActuatorType::SIMPLE_THRUSTER:
                    ((SimpleThruster*)actuator)->setSetpoint(rosRobots[i]->thrusterSetpoints[thID++], Scalar(0));
                    break;

                case ActuatorType::THRUSTER:
                    ((Thruster*)actuator)->setSetpoint(rosRobots[i]->thrusterSetpoints[thID++]);
                    break;

                case ActuatorType::PROPELLER:
                    ((Propeller*)actuator)->setSetpoint(rosRobots[i]->propellerSetpoints[propID++]);
                    break;

                case ActuatorType::RUDDER:
                    ((Rudder*)actuator)->setSetpoint(rosRobots[i]->rudderSetpoints[rudderID++]);
                    break;

                case ActuatorType::SERVO:
                {
                    if(rosRobots[i]->servoSetpoints.size() == 0)
                        continue;

                    auto it = rosRobots[i]->servoSetpoints.find(((Servo*)actuator)->getJointName());
                    if(it != rosRobots[i]->servoSetpoints.end())
                    {
                        if(it->second.first == ServoControlMode::VELOCITY)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::VELOCITY);
                            ((Servo*)actuator)->setDesiredVelocity(it->second.second);
                        }
                        else if(it->second.first == ServoControlMode::POSITION)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::POSITION);
                            ((Servo*)actuator)->setDesiredPosition(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::VBS:
                {
                    auto it = pubs.find(actuator->getName());
                    if(it != pubs.end())
                    {
                        std_msgs::Float64 msg;
                        msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                        it->second.publish(msg);
                    }
                }
                    break;

                case ActuatorType::SUCTION_CUP:
                {
                    auto it = pubs.find(actuator->getName());
                    if(it != pubs.end())
                    {
                        std_msgs::Bool msg;
                        msg.data = ((SuctionCup*)actuator)->getPump();
                        it->second.publish(msg);
                    }
                }
                    break;

                default:
                    break;
            }
        }
    }

    ////////////////////////////////////// ROS CONTROL ////////////////////////////////////////////
    for(size_t i=0; i<controlIfs.size(); ++i)
    {
        controlIfs[i]->read();
        controlIfs[i]->update(ros::Time::now(), ros::Duration(timeStep));
        controlIfs[i]->write();
    }

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    auto hydroLambda = [](Robot* r)
    {
        std::stringstream output;
        size_t lID = 0;
        SolidEntity* link;
        Vector3 Fb, Tb, Fd, Td, Fs, Ts;
    
        while((link = r->getLink(lID++)) != nullptr)
        {
            link->getHydrodynamicForces(Fb, Tb, Fd, Td, Fs, Ts);
            output << "[" << link->getName() << "]" << std::endl;
            output << "Buoyancy force[N]: " << Fb.x() << ", " << Fb.y() << ", " << Fb.z() << std::endl;
            output << "Damping force[N]: " << Fd.x() << ", " << Fd.y() << ", " << Fd.z() << std::endl;
            output << "Skin friction[N]: " << Fs.x() << ", " << Fs.y() << ", " << Fs.z() << std::endl;
        }
        return output.str();
    };

    for(size_t i=0; i<rosRobots.size(); ++i)
    {
        Robot* r = rosRobots[i]->robot;
        ROS_DEBUG_STREAM_NAMED("Hydrodynamics", "===== Hydrodynamics for " << r->getName() << " =====\n" << hydroLambda(r));
    }
}

void ROSSimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes[cam->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint8_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs.at(cam->getName()).publish(img);
    pubs.at(cam->getName() + "/info").publish(info);
}


void ROSSimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes[cam->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs.at(cam->getName()).publish(img);
    pubs.at(cam->getName() + "/info").publish(info);
}

void ROSSimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[fls->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint8_t*)fls->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[fls->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)fls->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs.at(fls->getName()).publish(img);
    imgPubs.at(fls->getName() + "/display").publish(disp);
}

void ROSSimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[sss->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint8_t*)sss->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[sss->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)sss->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs.at(sss->getName()).publish(img);
    imgPubs.at(sss->getName() + "/display").publish(disp);
}

void ROSSimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::ImagePtr img = sonarMsgPrototypes[msis->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint8_t*)msis->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::ImagePtr disp = sonarMsgPrototypes[msis->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)msis->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs.at(msis->getName()).publish(img);
    imgPubs.at(msis->getName() + "/display").publish(disp);
}

void ROSSimulationManager::Multibeam2ScanReady(Multibeam2* mb)
{
    ROSInterface::PublishMultibeam2(pubs.at(mb->getName()), mb);
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

UniformVFCallback::UniformVFCallback(Uniform* vf) : vf(vf)
{
}

void UniformVFCallback::operator()(const geometry_msgs::Vector3ConstPtr& msg)
{
    vf->setVelocity(Vector3(msg->x, msg->y, msg->z));
}

JetVFCallback::JetVFCallback(Jet* vf) : vf(vf)
{
}

void JetVFCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    vf->setOutletVelocity(msg->data);
}

ThrustersCallback::ThrustersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void ThrustersCallback::operator()(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != robot->thrusterSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of thruster setpoints for robot: " << robot->robot->getName());
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
        ROS_ERROR_STREAM("Wrong number of propeller setpoints for robot: " << robot->robot->getName());
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
        ROS_ERROR_STREAM("Wrong number of rudder setpoints for robot: " << robot->robot->getName());
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

VBSCallback::VBSCallback(VariableBuoyancy* act) : act(act)
{
}

void VBSCallback::operator()(const std_msgs::Float64ConstPtr& msg)
{
    act->setFlowRate(msg->data);
}

ActuatorOriginCallback::ActuatorOriginCallback(Actuator* act) : act(act)
{
}

void ActuatorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(act->getType())
    {
        case ActuatorType::PUSH:
        case ActuatorType::SIMPLE_THRUSTER:
        case ActuatorType::THRUSTER:
        case ActuatorType::PROPELLER:
        case ActuatorType::VBS:
        case ActuatorType::LIGHT:
            ((LinkActuator*)act)->setRelativeActuatorFrame(T);
            break;

        default:
            ROS_WARN_STREAM("Live update of origin frame of actuator '" << act->getName() << "' not supported!");
            break;
    }
}

SensorOriginCallback::SensorOriginCallback(Sensor* sens) : sens(sens)
{
}

void SensorOriginCallback::operator()(const geometry_msgs::TransformConstPtr& msg)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->getType())
    {
        case SensorType::LINK:
            ((LinkSensor*)sens)->setRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            ((VisionSensor*)sens)->setRelativeSensorFrame(T);
            break;

        default:
            ROS_WARN_STREAM("Live update of origin frame of sensor '" << sens->getName() << "' not supported!");
            break;
    }
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
    tr->setTransform(Transform(q, p));
    tr->setLinearVelocity(v);
    tr->setAngularVelocity(omega);
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
        fls->setRangeMax(req.range_max);
        fls->setRangeMin(req.range_min);
        fls->setGain(req.gain);
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
        sss->setRangeMax(req.range_max);
        sss->setRangeMin(req.range_min);
        sss->setGain(req.gain);
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
        msis->setRangeMax(req.range_max);
        msis->setRangeMin(req.range_min);
        msis->setGain(req.gain);
        msis->setRotationLimits(req.rotation_min, req.rotation_max);
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
    suction->setPump(req.data);
    if(req.data)
        res.message = "Pump turned on.";
    else 
        res.message = "Pump turned off.";
    res.success = true;
    return true;
}


}
