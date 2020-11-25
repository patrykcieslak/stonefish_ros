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
//  ROSScenarioParser.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 17/09/19.
//  Copyright (c) 2019-2020 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSInterface.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/sensors/ScalarSensor.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/comms/Comm.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cola2_msgs/DVL.h>
#include <cola2_msgs/Setpoints.h>
#include <stonefish_ros/ThrusterState.h>
#include <stonefish_ros/Int32Stamped.h>

#include <ros/package.h>

namespace sf
{

ROSScenarioParser::ROSScenarioParser(ROSSimulationManager* sm) : ScenarioParser(sm)
{
}

std::string ROSScenarioParser::SubstituteROSVars(const std::string& value)
{
    std::string replacedValue;

    size_t currentPos = 0;
    size_t startPos, endPos;
    while ((startPos = value.find("$(", currentPos)) != std::string::npos && (endPos = value.find(")", startPos+2)) != std::string::npos)
    {
        replacedValue += value.substr(currentPos, startPos - currentPos);

        std::string arguments = value.substr(startPos+2, endPos-startPos-2);
        std::istringstream iss(arguments);
        std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                         std::istream_iterator<std::string>());
        if (results.size() != 2)
        {
            ROS_ERROR("Scenario parser(ROS): substitution args need to be 2, got: %s", arguments.c_str());
            continue;
        }

        if (results[0] == "find")
        {
            std::string packagePath = ros::package::getPath(results[1]);
            if (packagePath.empty())
            {
                ROS_ERROR("Scenario parser(ROS): could not find package '%s'!", results[1].c_str());
                return value;
            }
            replacedValue += packagePath;
        }
        else if (results[0] == "param")
        {
            std::string param;
            // to get private params, we need to prefix ~ it seems
            if (!results[1].empty() && results[1][0] != '~' && results[1][0] != '/')
            {
                results[1] = std::string("~") + results[1];
            }
            if (!ros::param::get(results[1], param))
            {
                ROS_ERROR("Scenario parser(ROS): could not find parameter '%s'!", results[1].c_str());
                return value;
            }
            replacedValue += param;
        }
        else //Command unsupported
        {
            return value;
        }

        currentPos = endPos + 1;
    }

    replacedValue += value.substr(currentPos, value.size() - currentPos);

    return replacedValue;
}

bool ROSScenarioParser::ReplaceROSVars(XMLNode* node)
{
    XMLElement* element = node->ToElement();
    if (element != nullptr)
    {
        for (const tinyxml2::XMLAttribute* attr = element->FirstAttribute(); attr != nullptr; attr = attr->Next())
        {
            std::string value = std::string(attr->Value());
            std::string substitutedValue = SubstituteROSVars(value);
            if (substitutedValue != value)
            {
                ROS_INFO("Scenario parser(ROS): replacing '%s' with '%s'.", value.c_str(), substitutedValue.c_str());
                element->SetAttribute(attr->Name(), substitutedValue.c_str());
            }

        }
    }

    for (tinyxml2::XMLNode* child = node->FirstChild(); child != nullptr; child = child->NextSibling())
    {
        if(!ReplaceROSVars(child))
            return false;
    }

    return true;
}

bool ROSScenarioParser::PreProcess(XMLNode* root, const std::map<std::string, std::string>& args)
{
    //First replace ROS variables to support them inside include arguments
    if(!ReplaceROSVars(root))
        return false;
    //Then process include arguments
    return ScenarioParser::PreProcess(root, args);
}

bool ROSScenarioParser::ParseRobot(XMLElement* element)
{
    if(!ScenarioParser::ParseRobot(element))
        return false;

    ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
    ros::NodeHandle& nh = sim->getNodeHandle();
    std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();
    std::map<std::string, ros::Subscriber>& subs = sim->getSubscribers();

    //Robot info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);
    Robot* robot = getSimulationManager()->getRobot(nameStr);

    //Check actuators
    unsigned int nThrusters = 0;
    unsigned int nPropellers = 0;
    unsigned int nServos = 0;

    unsigned int aID = 0;
    Actuator* act;
    while((act = robot->getActuator(aID++)) != NULL)
    {
        switch(act->getType())
        {
            case ActuatorType::THRUSTER:
                ++nThrusters;
                break;

            case ActuatorType::PROPELLER:
                ++nPropellers;
                break;

            case ActuatorType::SERVO:
                ++nServos;
                break;

            default:
                break;
        }
    }

    ROSRobot* rosRobot = new ROSRobot(robot, nThrusters, nPropellers);

    //Initialise servo setpoints
    if(nServos > 0)
    {
        aID = 0;
        while((act = robot->getActuator(aID++)) != NULL)
        {
            if(act->getType() == ActuatorType::SERVO)
                rosRobot->servoSetpoints[((Servo*)act)->getJointName()] = Scalar(0);
        }
    }

    //Check if we should publish world_ned -> base_link transform
    XMLElement* item;
    if((item = element->FirstChildElement("ros_base_link_transforms")) != nullptr)
        item->QueryBoolAttribute("publish", &rosRobot->publishBaseLinkTransform);

    //Save robot
    sim->AddROSRobot(rosRobot);

    //Generate subscribers
    if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
    {
        const char* topicThrust = nullptr;
        const char* topicProp = nullptr;
        const char* topicSrv = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThrust) == XML_SUCCESS)
            subs[robot->getName() + "/thrusters"] = nh.subscribe<cola2_msgs::Setpoints>(std::string(topicThrust), 10, ThrustersCallback(sim, rosRobot));
            
        if(nPropellers > 0 && item->QueryStringAttribute("propellers", &topicProp) == XML_SUCCESS)
            subs[robot->getName() + "/propellers"] = nh.subscribe<cola2_msgs::Setpoints>(std::string(topicProp), 10, PropellersCallback(sim, rosRobot));
        
        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            subs[robot->getName() + "/servos"] = nh.subscribe<sensor_msgs::JointState>(std::string(topicSrv), 10, ServosCallback(sim, rosRobot));
    }

    //Generate publishers
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        const char* topicThrust = nullptr;
        const char* topicSrv = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThrust) == XML_SUCCESS)
            pubs[robot->getName() + "/thrusters"] = nh.advertise<stonefish_ros::ThrusterState>(std::string(topicThrust), 10);

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            pubs[robot->getName() + "/servos"] = nh.advertise<sensor_msgs::JointState>(std::string(topicSrv), 10);
    }


    return true;
}

bool ROSScenarioParser::ParseAnimated(XMLElement* element)
{
    if(!ScenarioParser::ParseAnimated(element))
        return false;

    ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
    ros::NodeHandle& nh = sim->getNodeHandle();
    std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();
    std::map<std::string, ros::Subscriber>& subs = sim->getSubscribers();

    //Get name
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);

    //Get type of trajectory
    const char* type;
    XMLElement* item = element->FirstChildElement("trajectory");
    item->QueryStringAttribute("type", &type);
    std::string typeStr(type);

    if(typeStr == "manual") //Position of animated body set by a message
    {
        //Get topic group name
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_subscriber")) == nullptr 
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return true;
        }
        AnimatedEntity* anim = (AnimatedEntity*)sim->getEntity(nameStr);
        if(anim != nullptr)
            subs[nameStr + "/odometry"] = nh.subscribe<nav_msgs::Odometry>(std::string(topic), 1, TrajectoryCallback((ManualTrajectory*)anim->getTrajectory()));
    }
    else //State of the trajectory published with a message
    {
        //Get topic group name
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) == nullptr 
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return true;
        }
        pubs[nameStr + "/odometry"] = nh.advertise<nav_msgs::Odometry>(std::string(topic), 10);
        pubs[nameStr + "/iteration"] = nh.advertise<stonefish_ros::Int32Stamped>(std::string(topic) + "/iteration", 10);
    }
    
    return true;
}

bool ROSScenarioParser::ParseActuator(XMLElement* element, Robot* robot)
{
    if(!ScenarioParser::ParseActuator(element, robot))
        return false;

    ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
    ros::NodeHandle& nh = sim->getNodeHandle();
    std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();
    std::map<std::string, ros::Subscriber>& subs = sim->getSubscribers();

    //Actuator info
    const char* name = nullptr;
    const char* type = nullptr;
    element->QueryStringAttribute("name", &name);
    element->QueryStringAttribute("type", &type);
    std::string actuatorName = robot->getName() + "/" + std::string(name);
    std::string typeStr(type);

    //Publish and subscribe info
    if(typeStr == "vbs")
    {
        XMLElement* item;
        const char* pubTopic = nullptr;
        const char* subTopic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) != nullptr 
            && item->QueryStringAttribute("topic", &pubTopic) == XML_SUCCESS)
        {
            pubs[actuatorName] = nh.advertise<std_msgs::Float64>(std::string(pubTopic), 10);
        }
        if((item = element->FirstChildElement("ros_subscriber")) != nullptr
            && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
        {
            subs[actuatorName] = nh.subscribe<std_msgs::Float64>(std::string(subTopic), 1, VBSCallback((VariableBuoyancy*)robot->getActuator(actuatorName)));
        }
    }

    return true;
}

Sensor* ROSScenarioParser::ParseSensor(XMLElement* element, const std::string& namePrefix)
{
    Sensor* sens = ScenarioParser::ParseSensor(element, namePrefix);
    if(sens != nullptr)
    {
        ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
        ros::NodeHandle& nh = sim->getNodeHandle();
        std::map<std::string, ros::ServiceServer>& srvs = sim->getServiceServers();
        std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();
        std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& camMsgProto = sim->getCameraMsgPrototypes();
        std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& sonarMsgProto = sim->getSonarMsgPrototypes();
        std::string sensorName = sens->getName();

        //Publishing info
        XMLElement* item;
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) == nullptr 
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return sens;
        }

        std::string topicStr(topic);
        unsigned int queueSize = (unsigned int)ceil(sens->getUpdateFrequency());
    
        //Generate publishers for different sensor types
        switch(sens->getType())
        {
            case SensorType::JOINT:
            case SensorType::LINK:
            {
                switch(((ScalarSensor*)sens)->getScalarSensorType())
                {
                    case ScalarSensorType::IMU:
                        pubs[sensorName] = nh.advertise<sensor_msgs::Imu>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::DVL:
                    {
                        pubs[sensorName] = nh.advertise<cola2_msgs::DVL>(topicStr, queueSize);

                        //Second topic with altitude
                        std::string altTopicStr = topicStr + "/altitude"; 
                        const char* altTopic = nullptr; 
                        if(item->QueryStringAttribute("altitude_topic", &altTopic) == XML_SUCCESS)
                            altTopicStr = std::string(altTopic);
                        pubs[sensorName + "/altitude"] = nh.advertise<sensor_msgs::Range>(altTopicStr, queueSize);
                    }
                        break;

                    case ScalarSensorType::GPS:
                        pubs[sensorName] = nh.advertise<sensor_msgs::NavSatFix>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::PRESSURE:
                        pubs[sensorName] = nh.advertise<sensor_msgs::FluidPressure>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::ODOM:
                        pubs[sensorName] = nh.advertise<nav_msgs::Odometry>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::FT:
                        pubs[sensorName] = nh.advertise<geometry_msgs::WrenchStamped>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::ENCODER:
                        pubs[sensorName] = nh.advertise<sensor_msgs::JointState>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::MULTIBEAM:
                        pubs[sensorName] = nh.advertise<sensor_msgs::LaserScan>(topicStr, queueSize);
                        break;

                    default:
                        ROS_ERROR("Scenario parser(ROS): sensor '%s' not supported!", sensorName.c_str());
                        break;
                }
            }
                break;

            case SensorType::VISION:
            {
                switch(((VisionSensor*)sens)->getVisionSensorType())
                {
                    case VisionSensorType::COLOR_CAMERA:
                    {
                        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(topicStr + "/camera_info", queueSize);
                        ColorCamera* cam = (ColorCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::ColorCameraImageReady, sim, std::placeholders::_1));
                        camMsgProto[sensorName] = ROSInterface::GenerateCameraMsgPrototypes(cam, false);
                    }
                        break;

                    case VisionSensorType::DEPTH_CAMERA:
                    {
                        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image_depth", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(topicStr + "/camera_info", queueSize);
                        DepthCamera* cam = (DepthCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::DepthCameraImageReady, sim, std::placeholders::_1));
                        camMsgProto[sensorName] = ROSInterface::GenerateCameraMsgPrototypes(cam, true);
                    }
                        break;

                    case VisionSensorType::MULTIBEAM2:
                    {
                        pubs[sensorName] = nh.advertise<sensor_msgs::PointCloud2>(topicStr, queueSize);
                        Multibeam2* mb = (Multibeam2*)sens;
                        mb->InstallNewDataHandler(std::bind(&ROSSimulationManager::Multibeam2ScanReady, sim, std::placeholders::_1));
                    }
                        break;

                    case VisionSensorType::FLS:
                    {
                        FLS* fls = (FLS*)sens;
                        fls->InstallNewDataHandler(std::bind(&ROSSimulationManager::FLSScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateFLSMsgPrototypes(fls);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings::Request, stonefish_ros::SonarSettings::Response>(topicStr + "/settings", FLSService(fls));
                        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image", queueSize);
                        pubs[sensorName + "/display"] = nh.advertise<sensor_msgs::Image>(topicStr + "/display", queueSize);
                    }
                        break;

                    case VisionSensorType::SSS:
                    {
                        SSS* sss = (SSS*)sens;
                        sss->InstallNewDataHandler(std::bind(&ROSSimulationManager::SSSScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateSSSMsgPrototypes(sss);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings::Request, stonefish_ros::SonarSettings::Response>(topicStr + "/settings", SSSService(sss));
                        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image", queueSize);
                        pubs[sensorName + "/display"] = nh.advertise<sensor_msgs::Image>(topicStr + "/display", queueSize);
                    }
                        break;

                    case VisionSensorType::MSIS:
                    {
                        MSIS* msis = (MSIS*)sens;
                        msis->InstallNewDataHandler(std::bind(&ROSSimulationManager::MSISScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateMSISMsgPrototypes(msis);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings2::Request, stonefish_ros::SonarSettings2::Response>(topicStr + "/settings", MSISService(msis));
                        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image", queueSize);
                        pubs[sensorName + "/display"] = nh.advertise<sensor_msgs::Image>(topicStr + "/display", queueSize);
                    }
                        break;

                    default:
                        ROS_ERROR("Scenario parser(ROS): sensor '%s' not supported!", sensorName.c_str());
                        break;
                }
            }
                break;

            default:
                ROS_ERROR("Scenario parser(ROS): sensor '%s' not supported!", sensorName.c_str());
                break;
        }
    }
    return sens;
}

Comm* ROSScenarioParser::ParseComm(XMLElement* element, const std::string& namePrefix)
{
    Comm* comm = ScenarioParser::ParseComm(element, namePrefix);
    if(comm != nullptr)
    {
        ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
        ros::NodeHandle& nh = sim->getNodeHandle();
        std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();
        std::string commName = comm->getName();
        
        //Publishing info
        XMLElement* item;
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) == nullptr 
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return comm;
        }
        
        std::string topicStr(topic);

        //Generate publishers for different comm types
        switch(comm->getType())
        {
            case CommType::USBL:
                pubs[commName] = nh.advertise<visualization_msgs::MarkerArray>(topicStr, 10);
                break;

            default:
                break;   
        }
    }
    return comm;
}

bool ROSScenarioParser::ParseContact(XMLElement* element)
{
    if(!ScenarioParser::ParseContact(element))
        return false;
    
    ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
    ros::NodeHandle& nh = sim->getNodeHandle();
    std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();

    //Contact info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string contactName = std::string(name);
    
    //Publishing info
    XMLElement* item;
    const char* topic = nullptr;
    if((item = element->FirstChildElement("ros_publisher")) == nullptr 
        || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        return true;
    std::string topicStr(topic);
    
    pubs[contactName] = nh.advertise<visualization_msgs::Marker>(topicStr, 10);
    
    return true;
}

}
