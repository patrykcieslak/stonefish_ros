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
//  Copyright (c) 2019-2021 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSInterface.h"
#include "stonefish_ros/ThrusterState.h"

#include <Stonefish/core/Robot.h>
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
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/utils/SystemUtil.hpp>

#include <ros/file_log.h>

namespace sf
{

ROSSimulationManager::ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath) 
	: SimulationManager(stepsPerSecond, SolverType::SOLVER_SI, CollisionFilteringType::COLLISION_EXCLUSIVE), scnFilePath(scenarioFilePath), nh("~")
{
    srvECurrents = nh.advertiseService("enable_currents", &ROSSimulationManager::EnableCurrents, this);
    srvDCurrents = nh.advertiseService("disable_currents", &ROSSimulationManager::DisableCurrents, this);
}

ROSSimulationManager::~ROSSimulationManager()
{
}

uint64_t ROSSimulationManager::getSimulationClock()
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

std::map<std::string, ros::ServiceServer>& ROSSimulationManager::getServiceServers()
{
    return srvs;
}

std::map<std::string, ros::Publisher>& ROSSimulationManager::getPublishers()
{
    return pubs;
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
    while((sensor = getSensor(id++)) != NULL)
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
                    ROSInterface::PublishAccelerometer(pubs[sensor->getName()], (Accelerometer*)sensor);
                    break;

                case ScalarSensorType::GYRO:
                    ROSInterface::PublishGyroscope(pubs[sensor->getName()], (Gyroscope*)sensor);
                    break;

                case ScalarSensorType::IMU:
                    ROSInterface::PublishIMU(pubs[sensor->getName()], (IMU*)sensor);
                    break;

                case ScalarSensorType::ODOM:
                    ROSInterface::PublishOdometry(pubs[sensor->getName()], (Odometry*)sensor);
                    break;

                case ScalarSensorType::DVL:
                    ROSInterface::PublishDVL(pubs[sensor->getName()], pubs[sensor->getName() + "/altitude"], (DVL*)sensor);
                    break;

                case ScalarSensorType::GPS:
                    ROSInterface::PublishGPS(pubs[sensor->getName()], (GPS*)sensor);
                    break;

                case ScalarSensorType::PRESSURE:
                    ROSInterface::PublishPressure(pubs[sensor->getName()], (Pressure*)sensor);
                    break;

                case ScalarSensorType::FT:
                    ROSInterface::PublishForceTorque(pubs[sensor->getName()], (ForceTorque*)sensor);
                    break;

                case ScalarSensorType::ENCODER:
                    ROSInterface::PublishEncoder(pubs[sensor->getName()], (RotaryEncoder*)sensor);
                    break;

                case ScalarSensorType::MULTIBEAM:
                    ROSInterface::PublishMultibeam(pubs[sensor->getName()], (Multibeam*)sensor);
                    break;

                case ScalarSensorType::PROFILER:
                    ROSInterface::PublishProfiler(pubs[sensor->getName()], (Profiler*)sensor);
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
    while((comm = getComm(id++)) != NULL)
    {
        if(!comm->isNewDataAvailable())
            continue;

        if(pubs.find(comm->getName()) == pubs.end())
            continue;

        switch(comm->getType())
        {
            case CommType::USBL:
                ROSInterface::PublishUSBL(pubs[comm->getName()], (USBL*)comm);
                comm->MarkDataOld();
                break;
            
            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = getEntity(id++)) != NULL)
    {
        if(ent->getType() == EntityType::ANIMATED)
        {
            if(pubs.find(ent->getName() + "/odometry") == pubs.end())
                continue;

            ROSInterface::PublishTrajectoryState(pubs[ent->getName() + "/odometry"], pubs[ent->getName() + "/iteration"], (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = getContact(id++)) != NULL)
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
        if(rosRobots[i]->servoSetpoints.size() != 0 
           && pubs.find(rosRobots[i]->robot->getName() + "/servos") != pubs.end())
        {
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
                if(actuator->getType() == ActuatorType::SERVO)
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

        if(rosRobots[i]->thrusterSetpoints.size() != 0 
           && pubs.find(rosRobots[i]->robot->getName() + "/thrusters") != pubs.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            Thruster* th;

            stonefish_ros::ThrusterState msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = rosRobots[i]->robot->getName();
            msg.setpoint.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.rpm.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.thrust.resize(rosRobots[i]->thrusterSetpoints.size());
            msg.torque.resize(rosRobots[i]->thrusterSetpoints.size());

            while((actuator = rosRobots[i]->robot->getActuator(aID++)) != NULL)
            {
                if(actuator->getType() == ActuatorType::THRUSTER)
                {
                    th = (Thruster*)actuator;
                    msg.setpoint[thID] = th->getSetpoint();
                    msg.rpm[thID] = th->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots[i]->thrusterSetpoints.size())
                        break;
                }
            }

            pubs[rosRobots[i]->robot->getName() + "/thrusters"].publish(msg);
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
        
        while((actuator = rosRobots[i]->robot->getActuator(aID++)) != NULL)
        {
            switch(actuator->getType())
            {
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

                case ActuatorType::VBS:
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
    //Fill in the image message
    sensor_msgs::ImagePtr img = cameraMsgPrototypes[cam->getName()].first;
    img->header.stamp = ros::Time::now();
    memcpy(img->data.data(), (uint8_t*)cam->getImageDataPointer(), img->step * img->height);
    
    //Fill in the info message
    sensor_msgs::CameraInfoPtr info = cameraMsgPrototypes[cam->getName()].second;
    info->header.stamp = img->header.stamp;
    
    //Publish messages
    pubs[cam->getName()].publish(img);
    pubs[cam->getName() + "/info"].publish(info);
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
    pubs[cam->getName()].publish(img);
    pubs[cam->getName() + "/info"].publish(info);
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
    pubs[fls->getName()].publish(img);
    pubs[fls->getName() + "/display"].publish(disp);
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
    pubs[sss->getName()].publish(img);
    pubs[sss->getName() + "/display"].publish(disp);
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
    pubs[msis->getName()].publish(img);
    pubs[msis->getName() + "/display"].publish(disp);
}

void ROSSimulationManager::Multibeam2ScanReady(Multibeam2* mb)
{
    ROSInterface::PublishPointCloud(pubs[mb->getName()], mb);
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

RuddersCallback::RuddersCallback(ROSSimulationManager* sm, ROSRobot* robot) : sm(sm), robot(robot)
{
}

void RuddersCallback::operator()(const cola2_msgs::SetpointsConstPtr& msg)
{
    if(msg->setpoints.size() != robot->rudderSetpoints.size())
    {
        ROS_ERROR_STREAM("Wrong number of rudder setpoints for robot: " << robot->robot->getName());
        return;
    }

    for(size_t i=0; i<robot->rudderSetpoints.size(); ++i)
        robot->rudderSetpoints[i] = msg->setpoints[i];   
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

}
