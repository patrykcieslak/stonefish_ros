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
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSSimulationManager.h"

#include <core/Robot.h>
#include <actuators/Actuator.h>
#include <actuators/Servo.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/DepthCamera.h>
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
#include <cola2_msgs/DVL.h>
#include <cola2_msgs/Setpoints.h>

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
        else 
        {
            ROS_ERROR("Scenario parser(ROS): substitution command '%s' not currently supported!", results[0].c_str());
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

bool ROSScenarioParser::PreProcess(XMLNode* root)
{
    return ReplaceROSVars(root);
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
            case ActuatorType::ACTUATOR_THRUSTER:
                ++nThrusters;
                break;

            case ActuatorType::ACTUATOR_PROPELLER:
                ++nPropellers;
                break;

            case ActuatorType::ACTUATOR_SERVO:
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
            if(act->getType() == ActuatorType::ACTUATOR_SERVO)
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
            subs[robot->getName() + "/thrusters"] = nh.subscribe<cola2_msgs::Setpoints>(std::string(topicThrust), 1, ThrustersCallback(sim, rosRobot));
            
        if(nPropellers > 0 && item->QueryStringAttribute("propellers", &topicProp) == XML_SUCCESS)
            subs[robot->getName() + "/propellers"] = nh.subscribe<cola2_msgs::Setpoints>(std::string(topicProp), 1, PropellersCallback(sim, rosRobot));
        
        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            subs[robot->getName() + "/servos"] = nh.subscribe<sensor_msgs::JointState>(std::string(topicSrv), 1, ServosCallback(sim, rosRobot));
    }

    //Generate publishers
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        const char* topicSrv = nullptr;

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            pubs[robot->getName() + "/servos"] = nh.advertise<sensor_msgs::JointState>(std::string(topicSrv), 2);
    }


    return true;
}

bool ROSScenarioParser::ParseSensor(XMLElement* element, Robot* robot)
{
    if(!ScenarioParser::ParseSensor(element, robot))
        return false;

    ROSSimulationManager* sim = (ROSSimulationManager*)getSimulationManager();
    ros::NodeHandle& nh = sim->getNodeHandle();
    std::map<std::string, ros::Publisher>& pubs = sim->getPublishers();

    //Sensor info
    const char* name = nullptr;
    const char* type = nullptr;
    element->QueryStringAttribute("name", &name);
    element->QueryStringAttribute("type", &type);
    std::string sensorName = robot->getName() + "/" + std::string(name);
    std::string typeStr(type);

    //Publishing info
    XMLElement* item;
    const char* topic = nullptr;
    if((item = element->FirstChildElement("ros_publisher")) == nullptr 
        || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        return true;
    std::string topicStr(topic);
    
    //Generate publishers for different sensor types
    if(typeStr == "imu")
        pubs[sensorName] = nh.advertise<sensor_msgs::Imu>(topicStr, 2);
    else if(typeStr == "dvl")
    {
        pubs[sensorName] = nh.advertise<cola2_msgs::DVL>(topicStr, 2);

        //Second topic with altitude
        std::string altTopicStr = topicStr + "/altitude"; 
        const char* altTopic = nullptr; 
        if(item->QueryStringAttribute("altitude_topic", &altTopic) == XML_SUCCESS)
            altTopicStr = std::string(altTopic);
        pubs[sensorName + "/altitude"] = nh.advertise<sensor_msgs::Range>(altTopicStr, 2);
    }
    else if(typeStr == "gps")
        pubs[sensorName] = nh.advertise<sensor_msgs::NavSatFix>(topicStr, 2);
    else if(typeStr == "pressure")
        pubs[sensorName] = nh.advertise<sensor_msgs::FluidPressure>(topicStr, 2);
    else if(typeStr == "odometry")
        pubs[sensorName] = nh.advertise<nav_msgs::Odometry>(topicStr, 2);
    else if(typeStr == "forcetorque")
        pubs[sensorName] = nh.advertise<geometry_msgs::WrenchStamped>(topicStr, 2);
    else if(typeStr == "encoder")
        pubs[sensorName] = nh.advertise<sensor_msgs::JointState>(topicStr, 2);
    else if(typeStr == "multibeam1d")
        pubs[sensorName] = nh.advertise<sensor_msgs::LaserScan>(topicStr, 2);
    else if(typeStr == "camera")
    {
        pubs[sensorName] = nh.advertise<sensor_msgs::Image>(topicStr + "/image_color", 2);
        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(topicStr + "/camera_info", 2);
        ColorCamera* cam = (ColorCamera*)robot->getSensor(sensorName);
        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::ColorCameraImageReady, sim, std::placeholders::_1));
    }
    else if(typeStr == "depthcamera")
    {
        pubs[sensorName] = nh.advertise<sensor_msgs::PointCloud2>(topicStr, 2);
        DepthCamera* cam = (DepthCamera*)robot->getSensor(sensorName);
        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::DepthCameraImageReady, sim, std::placeholders::_1));
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
            pubs[actuatorName] = nh.advertise<std_msgs::Float64>(std::string(pubTopic), 2);
        }
        if((item = element->FirstChildElement("ros_subscriber")) != nullptr
            && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
        {
            subs[actuatorName] = nh.subscribe<std_msgs::Float64>(std::string(subTopic), 1, VBSCallback((VariableBuoyancy*)robot->getActuator(actuatorName)));
        }
    }

    return true;
}

}
