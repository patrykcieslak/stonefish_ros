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
//  Copyright (c) 2019-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSScenarioParser.h"
#include "stonefish_ros/ROSSimulationManager.h"
#include "stonefish_ros/ROSInterface.h"
#include "stonefish_ros/ROSSensorPluginInterface.h"
#include "stonefish_ros/ROSActuatorPluginInterface.h"
#include "stonefish_ros/ROSCommPluginInterface.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Light.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/sensors/ScalarSensor.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/vision/Lidar.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/comms/Comm.h>
#include <Stonefish/joints/FixedJoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 
#include <stonefish_ros/DVL.h>
#include <stonefish_ros/INS.h>
#include <stonefish_ros/ThrusterState.h>
#include <stonefish_ros/Int32Stamped.h>
#include <stonefish_ros/BeaconInfo.h>
#include <stonefish_ros/Event.h>
#include <stonefish_ros/EventArray.h>

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
            log.Print(MessageType::ERROR, "[ROS] Argument substitution requires 2 values to be provided, got: '%s'!", arguments.c_str());
            ROS_ERROR("Scenario parser: Argument substitution failed '%s'!", arguments.c_str());
            continue;
        }

        if (results[0] == "find")
        {
            std::string packagePath = ros::package::getPath(results[1]);
            if (packagePath.empty())
            {
                log.Print(MessageType::ERROR, "[ROS] Could not find package '%s'!", results[1].c_str());
                ROS_ERROR("Scenario parser: Could not find package '%s'!", results[1].c_str());
                return value;
            }
            replacedValue += packagePath;
        }
        else if (results[0] == "param")
        {
            XmlRpc::XmlRpcValue param;
            std::string paramStr;
            // to get private params, we need to prefix ~ it seems
            if (!results[1].empty() && results[1][0] != '~' && results[1][0] != '/')
            {
                results[1] = std::string("~") + results[1];
            }
            if (!ros::param::get(results[1], param))
            {
                log.Print(MessageType::ERROR, "[ROS] Could not find parameter '%s'!", results[1].c_str());
                ROS_ERROR("Scenario parser: Could not find parameter '%s'!", results[1].c_str());
                return value;
            }
            switch(param.getType())
            {
                case XmlRpc::XmlRpcValue::TypeDouble:
                    paramStr = std::to_string(static_cast<double>(param));
                    break;

                case XmlRpc::XmlRpcValue::TypeInt:
                    paramStr = std::to_string(static_cast<int>(param));
                    break;

                case XmlRpc::XmlRpcValue::TypeString:
                    paramStr = static_cast<std::string>(param);
                    break;

                default:
                    log.Print(MessageType::ERROR, "[ROS] Found parameter '%s' of unsupported type!", results[1].c_str());
                    ROS_ERROR("Scenario parser: Found parameter '%s' of unsupported type!", results[1].c_str());
                    return value;
            }
            replacedValue += paramStr;
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
                log.Print(MessageType::INFO, "[ROS] Replacing '%s' with '%s'.", value.c_str(), substitutedValue.c_str());
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

std::unique_ptr<VelocityField> ROSScenarioParser::ParseVelocityField(XMLElement* element)
{
    std::unique_ptr<VelocityField> vf = ScenarioParser::ParseVelocityField(element);
    if(vf != nullptr)
    {
        ROSSimulationManager* sim = static_cast<ROSSimulationManager*>((GetSimulationManager()));
        ros::NodeHandle& nh = sim->GetNodeHandle();
        std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
        XMLElement* item;

        switch(vf->GetType())
        {
            case VelocityFieldType::UNIFORM:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                   && item->QueryStringAttribute("velocity", &subTopic) == XML_SUCCESS)
                {
                    subs["vf"+std::to_string(ros::Time::now().toNSec())]  //Unique subscriber key string
                        = nh.subscribe<geometry_msgs::Vector3>(std::string(subTopic), 1, UniformVFCallback(static_cast<Uniform*>(vf.get())));
                }
            }
                break;

            case VelocityFieldType::JET:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                   && item->QueryStringAttribute("outlet_velocity", &subTopic) == XML_SUCCESS)
                {
                    subs["vf"+std::to_string(ros::Time::now().toNSec())]  //Unique subscriber key string
                        = nh.subscribe<std_msgs::Float64>(std::string(subTopic), 1, JetVFCallback(static_cast<Jet*>(vf.get())));
                }
            }
                break;

            default:
                break;
        }
    }
    return vf;
}

bool ROSScenarioParser::ParseRobot(XMLElement* element)
{
    if(!ScenarioParser::ParseRobot(element))
        return false;

    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>((GetSimulationManager()));
    ros::NodeHandle& nh = sim->GetNodeHandle();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
    std::vector<std::unique_ptr<ROSControlInterface>>& cifs = sim->GetControlInterfaces();

    //Robot info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);
    Robot* robot = (GetSimulationManager())->GetRobot(nameStr);

    //Check actuators
    unsigned int nThrusters = 0;
    unsigned int nPropellers = 0;
    unsigned int nServos = 0;
    unsigned int nRudders = 0;
    unsigned int nMotors = 0;

    unsigned int aID = 0;
    Actuator* act;

    while((act = robot->GetActuator(aID++)) != nullptr)
    {
        if (act->GetType() == ActuatorType::LINK)
        {
            LinkActuator* linkAct = static_cast<LinkActuator*>(act);
            switch(linkAct->GetLinkActuatorType())
            {
                case LinkActuatorType::SIMPLE_THRUSTER:
                case LinkActuatorType::THRUSTER:
                    ++nThrusters;
                    break;

                case LinkActuatorType::PROPELLER:
                    ++nPropellers;
                    break;

                case LinkActuatorType::RUDDER:
                    ++nRudders;
                    break;

                default:
                    break;
            }
        }
        else if (act->GetType() == ActuatorType::JOINT)
        {
            JointActuator* jointAct = static_cast<JointActuator*>(act);
            switch(jointAct->GetJointActuatorType())
            {
                case JointActuatorType::SERVO:
                    ++nServos;
                    break;

                case JointActuatorType::MOTOR:
                    ++nMotors;
                    break;

                default:
                    break;
            }
        }
    }

    std::unique_ptr<ROSRobot> rosRobot = std::make_unique<ROSRobot>(robot, nThrusters, nPropellers, nRudders);

    //Check if we should publish world_ned -> base_link transform
    XMLElement* item;
    if((item = element->FirstChildElement("ros_base_link_transforms")) != nullptr)
        item->QueryBoolAttribute("publish", &rosRobot->publishBaseLinkTransform);

    //Generate subscribers
    if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
    {
        const char* topicThrust = nullptr;
        const char* topicProp = nullptr;
        const char* topicRudder = nullptr;
        const char* topicSrv = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThrust) == XML_SUCCESS)
            subs[robot->GetName() + "/thrusters"] = nh.subscribe<std_msgs::Float64MultiArray>(std::string(topicThrust), 10, ThrustersCallback(sim, rosRobot.get()));

        if(nPropellers > 0 && item->QueryStringAttribute("propellers", &topicProp) == XML_SUCCESS)
            subs[robot->GetName() + "/propellers"] = nh.subscribe<std_msgs::Float64MultiArray>(std::string(topicProp), 10, PropellersCallback(sim, rosRobot.get()));

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRudder) == XML_SUCCESS)
            subs[robot->GetName() + "/rudders"] = nh.subscribe<std_msgs::Float64MultiArray>(std::string(topicRudder), 10, RuddersCallback(sim, rosRobot.get()));

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
        {
            subs[robot->GetName() + "/servos"] = nh.subscribe<sensor_msgs::JointState>(std::string(topicSrv), 10, ServosCallback(sim, rosRobot.get()));
            //Generate setpoint placeholders for the whole system
            aID = 0;
            while((act = robot->GetActuator(aID++)) != nullptr)
            {
                if (act->GetType() == ActuatorType::JOINT && static_cast<JointActuator*>(act)->GetJointActuatorType() == JointActuatorType::SERVO)
                    rosRobot->servoSetpoints[static_cast<Servo*>(act)->GetJointName()] = std::pair(ServoControlMode::VELOCITY, Scalar(0));
            }
        }
    }

    //Parse all defined joint groups, single joint subscribers and ros control interfaces
    if(nServos > 0 || nMotors > 0)
    {
        //Joint group subscribers
        const char* jgTopic = nullptr;
        const char* jgMode = nullptr;
        unsigned int jg = 0;
        ServoControlMode mode;

        for(item = element->FirstChildElement("ros_joint_group_subscriber"); item != nullptr; item = item->NextSiblingElement("ros_joint_group_subscriber"))
        {
            if(item->QueryStringAttribute("topic", &jgTopic) == XML_SUCCESS && item->QueryStringAttribute("control_mode", &jgMode) == XML_SUCCESS)
            {
                std::string modeStr(jgMode);
                if(modeStr == "velocity")
                    mode = ServoControlMode::VELOCITY;
                else if(modeStr == "position")
                    mode = ServoControlMode::POSITION;
                else if(modeStr == "torque")
                    mode = ServoControlMode::TORQUE;
                else
                    continue; //Skip joint group -> missing parameters
            }
            else
                continue;

            std::vector<std::string> jointNames;
            for(XMLElement* joint = item->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint"))
            {
                const char* jgJointName = nullptr;
                if(joint->QueryStringAttribute("name", &jgJointName) == XML_SUCCESS)
                    jointNames.push_back(robot->GetName() + "/" + std::string(jgJointName));
            }

            if(jointNames.size() > 0) // Any joints defined?
            {
                ROS_INFO_STREAM("Creating joint group subscriber (" << std::string(jgMode) << ") " << std::string(jgTopic));
                subs[robot->GetName() + "/joint_group" + std::to_string(jg)] = nh.subscribe<std_msgs::Float64MultiArray>(std::string(jgTopic), 10, JointGroupCallback(sim, rosRobot.get(), mode, jointNames));
                ++jg;
            }
        }

        //Single joint subscribers
        const char* jointName = nullptr;
        jg = 0;
        for(item = element->FirstChildElement("ros_joint_subscriber"); item != nullptr; item = item->NextSiblingElement("ros_joint_subscriber"))
        {
            if(item->QueryStringAttribute("joint_name", &jointName) == XML_SUCCESS
               && item->QueryStringAttribute("topic", &jgTopic) == XML_SUCCESS
               && item->QueryStringAttribute("control_mode", &jgMode) == XML_SUCCESS)
            {
                std::string modeStr(jgMode);
                if(modeStr == "velocity")
                    mode = ServoControlMode::VELOCITY;
                else if(modeStr == "position")
                    mode = ServoControlMode::POSITION;
                else if(modeStr == "torque")
                    mode = ServoControlMode::TORQUE;
                else
                    continue; //Skip joint group -> missing parameters
                ROS_INFO_STREAM("Creating joint subscriber (" << modeStr << ") " << std::string(jgTopic));
                subs[robot->GetName() + "/joint" + std::to_string(jg)] = 
                    nh.subscribe<std_msgs::Float64>(std::string(jgTopic), 10, JointCallback(sim, rosRobot.get(), mode, robot->GetName() + "/" + std::string(jointName)));
                ++jg;
            }
        }

        //ROSControl interfaces
        const char* rcifNamespace = nullptr;
        const char* rcifType = nullptr;

        for(item = element->FirstChildElement("ros_control_interface"); item != nullptr; item = item->NextSiblingElement("ros_control_interface"))
        {
            if(item->QueryStringAttribute("namespace", &rcifNamespace) == XML_SUCCESS && item->QueryStringAttribute("type", &rcifType) == XML_SUCCESS)
            {
                std::string typeStr(rcifType);
                if(typeStr == "velocity")
                    mode = ServoControlMode::VELOCITY;
                else if(typeStr == "position")
                    mode = ServoControlMode::POSITION;
                else
                    continue; //Skip joint group -> missing parameters
            }
            else
                continue;

            std::vector<std::string> jointNames;
            for(XMLElement* joint = item->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint"))
            {
                const char* rcifJointName = nullptr;
                if(joint->QueryStringAttribute("name", &rcifJointName) == XML_SUCCESS)
                    jointNames.push_back(robot->GetName() + "/" + std::string(rcifJointName));
            }

            if(jointNames.size() > 0) // Any joints defined?
            {
                ROS_INFO_STREAM("Creating ros control interface (" << std::string(rcifType) << ") " << std::string(rcifNamespace));
                cifs.push_back(std::make_unique<ROSControlInterface>(robot, jointNames, mode, std::string(rcifNamespace)));
            }
        }
    }

    //Generate publishers
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        const char* topicThrust = nullptr;
        const char* topicSrv = nullptr;
        const char* topicMtr = nullptr;
        const char* topicRud = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThrust) == XML_SUCCESS)
            pubs[robot->GetName() + "/thrusters"] = nh.advertise<stonefish_ros::ThrusterState>(std::string(topicThrust), 10);

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            pubs[robot->GetName() + "/servos"] = nh.advertise<sensor_msgs::JointState>(std::string(topicSrv), 10);

        if(nMotors > 0 && item->QueryStringAttribute("motors", &topicMtr) == XML_SUCCESS)
            pubs[robot->GetName() + "/motors"] = nh.advertise<sensor_msgs::JointState>(std::string(topicMtr), 10);

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRud) == XML_SUCCESS)
            pubs[robot->GetName() + "/servos"] = nh.advertise<sensor_msgs::JointState>(std::string(topicRud), 10);
    }

    //Save robot
    sim->AddROSRobot(std::move(rosRobot));

    return true;
}

bool ROSScenarioParser::ParseAnimated(XMLElement* element)
{
    if(!ScenarioParser::ParseAnimated(element))
        return false;

    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>((GetSimulationManager()));
    ros::NodeHandle& nh = sim->GetNodeHandle();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();

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
        AnimatedEntity* anim = static_cast<AnimatedEntity*>(sim->GetEntity(nameStr));
        if(anim != nullptr)
            subs[nameStr + "/odometry"] = nh.subscribe<nav_msgs::Odometry>(std::string(topic), 1, 
                                            TrajectoryCallback(static_cast<ManualTrajectory*>(anim->GetTrajectory())));
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

std::unique_ptr<Actuator, ActuatorDeleter> ROSScenarioParser::ParseActuator(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Actuator, ActuatorDeleter> act = ScenarioParser::ParseActuator(element, namePrefix);
    if (act == nullptr)
        return act;

    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
    ros::NodeHandle& nh = sim->GetNodeHandle();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
    std::unordered_map<std::string, ros::ServiceServer>& srvs = sim->GetServiceServers();
    std::string actuatorName = act->GetName();
    XMLElement* item;

    //Get subscriber topics
    std::unordered_map<std::string, std::string> subTopics;
    if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
    {
        for (const XMLAttribute* attr = item->FirstAttribute(); attr != nullptr; attr = attr->Next()) 
        {
            std::string name(attr->Name());
            std::string value(attr->Value());
            if (value != "")
                subTopics[name] = std::string(attr->Value());
        }
    }

    //Get publisher topics
    std::unordered_map<std::string, std::string> pubTopics;
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        for (const XMLAttribute* attr = item->FirstAttribute(); attr != nullptr; attr = attr->Next()) 
        {
            std::string name(attr->Name());
            std::string value(attr->Value());
            if (value != "")
                pubTopics[name] = std::string(attr->Value());
        }
    }    
    
    if (subTopics.empty() && pubTopics.empty()) // No ROS messaging enabled
        return act;

    //Actuator specific handling
    if (act->GetType() == ActuatorType::LINK)
    {
        switch(static_cast<LinkActuator*>(act.get())->GetLinkActuatorType())
        {
            case LinkActuatorType::SIMPLE_THRUSTER:
            {
                if (auto it = subTopics.find("topic"); it != subTopics.end())
                    subs[actuatorName] = nh.subscribe<std_msgs::Float64>(it->second, 1, SimpleThrusterCallback(static_cast<SimpleThruster*>(act.get())));

                if (auto it = pubTopics.find("wrench_topic"); it != pubTopics.end())
                    pubs[actuatorName + "/wrench"] = nh.advertise<geometry_msgs::WrenchStamped>(it->second, 10);

                if (auto it = pubTopics.find("joint_state_topic"); it != pubTopics.end())
                    pubs[actuatorName + "/joint_state"] = nh.advertise<sensor_msgs::JointState>(it->second, 10);
            }
                break;
            case LinkActuatorType::THRUSTER:
            {
                if (auto it = subTopics.find("topic"); it != subTopics.end())
                    subs[actuatorName] = nh.subscribe<std_msgs::Float64>(it->second, 1, ThrusterCallback(static_cast<Thruster*>(act.get())));

                if (auto it = pubTopics.find("wrench_topic"); it != pubTopics.end())
                    pubs[actuatorName + "/wrench"] = nh.advertise<geometry_msgs::WrenchStamped>(it->second, 10);

                if (auto it = pubTopics.find("joint_state_topic"); it != pubTopics.end())
                    pubs[actuatorName + "/joint_state"] = nh.advertise<sensor_msgs::JointState>(it->second, 10);
            }
                break;
            case LinkActuatorType::PROPELLER:
            {
                if (auto it = subTopics.find("topic"); it != subTopics.end())
                    subs[actuatorName] = nh.subscribe<std_msgs::Float64>(it->second, 1, PropellerCallback(static_cast<Propeller*>(act.get())));

                if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    pubs[actuatorName] = nh.advertise<geometry_msgs::WrenchStamped>(it->second, 10);
            }
                break;
            case LinkActuatorType::PUSH:
            {
                if (auto it = subTopics.find("topic"); it != subTopics.end())
                    subs[actuatorName] = nh.subscribe<std_msgs::Float64>(it->second, 1, PushCallback(static_cast<Push*>(act.get())));

                if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    pubs[actuatorName] = nh.advertise<geometry_msgs::WrenchStamped>(it->second, 10);
            }
                break;
            case LinkActuatorType::VBS:
            {
                if (auto it = subTopics.find("topic"); it != subTopics.end())
                    subs[actuatorName] = nh.subscribe<std_msgs::Float64>(it->second, 1, VBSCallback(static_cast<VariableBuoyancy*>(act.get())));

                if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    pubs[actuatorName] = nh.advertise<std_msgs::Float64>(it->second, 10);
            }
                break;
            case LinkActuatorType::SUCTION_CUP:
            {
                const char* srvTopic = nullptr;
                if((item = element->FirstChildElement("ros_service")) != nullptr && item->QueryStringAttribute("topic", &srvTopic) == XML_SUCCESS)
                {
                    SuctionCup* suction = static_cast<SuctionCup*>(act.get());
                    srvs[actuatorName] = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(std::string(srvTopic), SuctionCupService(suction));
                }

                if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    pubs[actuatorName] = nh.advertise<std_msgs::Bool>(it->second, 10);
            }
                break;
            case LinkActuatorType::PLUGIN:
            {
                ROSActuatorPluginInterface* iface = dynamic_cast<ROSActuatorPluginInterface*>(act.get());
                if (iface != nullptr)
                {
                    iface->CreatePublishers(nh, pubs, pubTopics);   
                    iface->CreateSubscribers(nh, subs, subTopics);
                }
            }
                break;
            default:
                return act;
        }

        //Handling of online origin updates
        if (auto it = subTopics.find("origin"); it != subTopics.end())
        {
            switch(static_cast<LinkActuator*>(act.get())->GetLinkActuatorType())
            {
                case LinkActuatorType::PUSH:
                case LinkActuatorType::SIMPLE_THRUSTER:
                case LinkActuatorType::THRUSTER:
                case LinkActuatorType::PROPELLER:
                case LinkActuatorType::VBS:
                    subs[actuatorName] = nh.subscribe<geometry_msgs::Transform>(it->second, 1, ActuatorOriginCallback(act.get()));
                    break;
                default:
                    break;
            }
        }
    }
    else if (act->GetType() == ActuatorType::JOINT)
    {
        if(static_cast<JointActuator*>(act.get())->GetJointActuatorType() == JointActuatorType::PLUGIN)
        {
            ROSActuatorPluginInterface* iface = dynamic_cast<ROSActuatorPluginInterface*>(act.get());
            if (iface != nullptr)
            {
                iface->CreatePublishers(nh, pubs, pubTopics);   
                iface->CreateSubscribers(nh, subs, subTopics);
            }
        }
    }
    return act;
}

std::unique_ptr<Sensor, SensorDeleter> ROSScenarioParser::ParseSensor(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Sensor, SensorDeleter> sens = ScenarioParser::ParseSensor(element, namePrefix);
    if(sens == nullptr)
        return sens;

    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
    ros::NodeHandle& nh = sim->GetNodeHandle();
    image_transport::ImageTransport& imgTrans = sim->GetImageTransportHandle();
    std::unordered_map<std::string, ros::ServiceServer>& srvs = sim->GetServiceServers();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, image_transport::Publisher>& imgPubs = sim->GetImagePublishers();
    std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
    std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& camMsgProto = sim->GetCameraMsgPrototypes();
    std::unordered_map<std::string, std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr>>& dualCamMsgProto = sim->GetDualImageCameraMsgPrototypes();
    std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& sonarMsgProto = sim->GetSonarMsgPrototypes();
    std::string sensorName = sens->GetName();
    XMLElement* item;

    //Get publisher topics
    std::unordered_map<std::string, std::string> pubTopics;
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        for (const XMLAttribute* attr = item->FirstAttribute(); attr != nullptr; attr = attr->Next()) 
        {
            std::string name(attr->Name());
            std::string value(attr->Value());
            if (value != "")
                pubTopics[name] = std::string(attr->Value());
        }
    }    
    else
    {
        return sens;
    }

    if (pubTopics.empty())
        return sens;

    //Sensor enable/disable service
    const char* serviceTopic = nullptr;
    if((item = element->FirstChildElement("ros_service")) != nullptr
        && item->QueryStringAttribute("set_enabled", &serviceTopic) == XML_SUCCESS)
    {
        srvs[sensorName + "/set_enabled"] = nh.advertiseService<std_srvs::SetBool::Request, 
            std_srvs::SetBool::Response>(std::string(serviceTopic), SensorService(sens.get()));
        sens->SetEnabled(false); // Sensor with an enable service starts as disabled
    }

    unsigned int queueSize = (unsigned int)ceil(sens->GetUpdateFrequency());

    //Generate publishers for different sensor types
    switch(sens->GetType())
    {
        case SensorType::JOINT:
        {
            switch(static_cast<ScalarSensor*>(sens.get())->GetScalarSensorType())
            {
                case ScalarSensorType::FT:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<geometry_msgs::WrenchStamped>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::ENCODER:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::JointState>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::PLUGIN:
                {
                    ROSSensorPluginInterface* iface = dynamic_cast<ROSSensorPluginInterface*>(sens.get());
                    if (iface != nullptr)
                        iface->CreatePublishers(nh, pubs, pubTopics);
                }
                    break;
                default:
                    log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                    ROS_ERROR("Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                    break;
            }
        }
            break;
        case SensorType::LINK:
        {
            switch(static_cast<ScalarSensor*>(sens.get())->GetScalarSensorType())
            {
                case ScalarSensorType::ACC:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<geometry_msgs::AccelWithCovarianceStamped>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::GYRO:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::IMU:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::Imu>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::DVL:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<stonefish_ros::DVL>(it->second, queueSize);
                    if (auto it = pubTopics.find("altitude_topic"); it != pubTopics.end())
                        pubs[sensorName + "/altitude"] = nh.advertise<sensor_msgs::Range>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::GPS:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::NavSatFix>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::PRESSURE:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::FluidPressure>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::INS:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<stonefish_ros::INS>(it->second, queueSize);
                    if (auto it = pubTopics.find("odometry_topic"); it != pubTopics.end())
                        pubs[sensorName + "/odometry"] = nh.advertise<nav_msgs::Odometry>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::ODOM:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<nav_msgs::Odometry>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::MULTIBEAM:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::LaserScan>(it->second, queueSize);
                    if (auto it = pubTopics.find("pcl_topic"); it != pubTopics.end())
                        pubs[sensorName + "/pcl"] = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::PROFILER:
                {
                    if (auto it = pubTopics.find("odometry_topic"); it != pubTopics.end())
                        pubs[sensorName] = nh.advertise<sensor_msgs::LaserScan>(it->second, queueSize);
                }
                    break;
                case ScalarSensorType::PLUGIN:
                {
                    ROSSensorPluginInterface* iface = dynamic_cast<ROSSensorPluginInterface*>(sens.get());
                    if (iface != nullptr)
                        iface->CreatePublishers(nh, pubs, pubTopics);
                }
                    break;
                default:
                    log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                    ROS_ERROR("Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                    break;
            }
            //Origin frame updates
            const char* originTopic = nullptr;
            if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
            {
                subs[sensorName] = nh.subscribe<geometry_msgs::Transform>(std::string(originTopic), queueSize, SensorOriginCallback(sens.get()));
            }
        }
            break;
        case SensorType::VISION:
        {
            switch(static_cast<VisionSensor*>(sens.get())->GetVisionSensorType())
            {
                case VisionSensorType::COLOR_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(it->second + "/camera_info", queueSize);
                        ColorCamera* cam = static_cast<ColorCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::ColorCameraImageReady, sim, std::placeholders::_1));
                        camMsgProto[sensorName] = ROSInterface::GenerateCameraMsgPrototypes(cam, false);
                    }
                }
                    break;
                case VisionSensorType::DEPTH_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image_depth", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(it->second + "/camera_info", queueSize);
                        DepthCamera* cam = static_cast<DepthCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::DepthCameraImageReady, sim, std::placeholders::_1));
                        camMsgProto[sensorName] = ROSInterface::GenerateCameraMsgPrototypes(cam, true);
                    }
                }
                    break;
                case VisionSensorType::THERMAL_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image_raw", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(it->second + "/camera_info", queueSize);
                        ThermalCamera* cam = static_cast<ThermalCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::ThermalCameraImageReady, sim, std::placeholders::_1));
                        dualCamMsgProto[sensorName] = ROSInterface::GenerateThermalCameraMsgPrototypes(cam); 
                    }
                }   
                    break;
                case VisionSensorType::OPTICAL_FLOW_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image_raw", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(it->second + "/camera_info", queueSize);
                        OpticalFlowCamera* cam = static_cast<OpticalFlowCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::OpticalFlowCameraImageReady, sim, std::placeholders::_1));
                        dualCamMsgProto[sensorName] = ROSInterface::GenerateOpticalFlowCameraMsgPrototypes(cam); 
                    }
                }   
                    break;
                case VisionSensorType::SEGMENTATION_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image_raw", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh.advertise<sensor_msgs::CameraInfo>(it->second + "/camera_info", queueSize);
                        SegmentationCamera* cam = static_cast<SegmentationCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::SegmentationCameraImageReady, sim, std::placeholders::_1));
                        dualCamMsgProto[sensorName] = ROSInterface::GenerateSegmentationCameraMsgPrototypes(cam); 
                    }
                }   
                    break;
                case VisionSensorType::EVENT_BASED_CAMERA:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        pubs[sensorName] = nh.advertise<stonefish_ros::EventArray>(it->second, queueSize);
                        EventBasedCamera* cam = static_cast<EventBasedCamera*>(sens.get());
                        cam->InstallNewDataHandler(std::bind(&ROSSimulationManager::EventBasedCameraOutputReady, sim, std::placeholders::_1));
                    }
                }
                    break;
                case VisionSensorType::LIDAR:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        pubs[sensorName] = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(it->second, queueSize);
                        Lidar* lidar = static_cast<Lidar*>(sens.get());
                        lidar->InstallNewDataHandler(std::bind(&ROSSimulationManager::LidarScanReady, sim, std::placeholders::_1));
                    }
                }
                    break;
                case VisionSensorType::FLS:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        FLS* fls = static_cast<FLS*>(sens.get());
                        fls->InstallNewDataHandler(std::bind(&ROSSimulationManager::FLSScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateFLSMsgPrototypes(fls);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings::Request, stonefish_ros::SonarSettings::Response>(it->second + "/settings", FLSService(fls));
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/display", queueSize);
                    }
                }
                    break;
                case VisionSensorType::SSS:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        SSS* sss = static_cast<SSS*>(sens.get());
                        sss->InstallNewDataHandler(std::bind(&ROSSimulationManager::SSSScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateSSSMsgPrototypes(sss);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings::Request, stonefish_ros::SonarSettings::Response>(it->second + "/settings", SSSService(sss));
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/display", queueSize);
                    }
                }
                    break;
                case VisionSensorType::MSIS:
                {
                    if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                    {
                        MSIS* msis = static_cast<MSIS*>(sens.get());
                        msis->InstallNewDataHandler(std::bind(&ROSSimulationManager::MSISScanReady, sim, std::placeholders::_1));
                        sonarMsgProto[sensorName] = ROSInterface::GenerateMSISMsgPrototypes(msis);
                        srvs[sensorName] = nh.advertiseService<stonefish_ros::SonarSettings2::Request, stonefish_ros::SonarSettings2::Response>(it->second + "/settings", MSISService(msis));
                        imgPubs[sensorName] = imgTrans.advertise(it->second + "/image", queueSize);
                        imgPubs[sensorName + "/display"] = imgTrans.advertise(it->second + "/display", queueSize);
                        pubs[sensorName + "/beam"] = nh.advertise<sensor_msgs::LaserScan>(it->second + "/last_beam", queueSize);
                    }
                }
                    break;
                default:
                    log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                    ROS_ERROR("Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                    break;
            }
            //Origin frame updates
            const char* originTopic = nullptr;
            if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
            {
                subs[sensorName] = nh.subscribe<geometry_msgs::Transform>(std::string(originTopic), queueSize, SensorOriginCallback(sens.get()));
            }
        }
            break;
        default:
            log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
            ROS_ERROR("Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
            break;
    }
    return sens;
}

std::unique_ptr<Comm, CommDeleter> ROSScenarioParser::ParseComm(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Comm, CommDeleter> comm = ScenarioParser::ParseComm(element, namePrefix);
    if(comm == nullptr)
        return comm;
    
    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
    ros::NodeHandle& nh = sim->GetNodeHandle();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
    std::string commName = comm->GetName();

    //Publishing info
    XMLElement* item;
    const char* pubTopic = nullptr;
    const char* subTopic = nullptr;
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
        item->QueryStringAttribute("topic", &pubTopic);
    if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
        item->QueryStringAttribute("topic", &subTopic);
    if(pubTopic == nullptr && subTopic == nullptr)
        return comm;

    //Generate subscribers for all comm types
    if(subTopic != nullptr)
    {
        std::string topicStr {subTopic};
        subs[commName + "/data_to_send"] = nh.subscribe<std_msgs::String>(topicStr + "/data_to_send", 1, CommCallback(comm.get()));
    }

    //Generate publishers for different comm types
    if (pubTopic != nullptr)
    {
        std::string topicStr {pubTopic};

        switch(comm->GetType())
        {
            case CommType::ACOUSTIC:
                pubs[commName] = nh.advertise<std_msgs::String>(topicStr + "/received_data", 10);
                break;
            case CommType::USBL:
            {
                pubs[commName] = nh.advertise<visualization_msgs::MarkerArray>(topicStr + "/beacons", 10);
                pubs[commName + "/beacon_info"] = nh.advertise<stonefish_ros::BeaconInfo>(topicStr + "/beacon_info", 10);
                pubs[commName + "/received_data"] = nh.advertise<std_msgs::String>(topicStr + "/received_data", 10);
            }
                break;
            case CommType::OPTICAL:
            {
                pubs[commName] = nh.advertise<std_msgs::Float64>(topicStr + "/reception_quality", 10);
                pubs[commName + "/received_data"] = nh.advertise<std_msgs::String>(topicStr + "/received_data", 10);
            }
                break;
            case CommType::PLUGIN:
            {
                ROSCommPluginInterface* iface = dynamic_cast<ROSCommPluginInterface*>(comm.get());
                if (iface != nullptr)
                    iface->CreatePublishers(nh, pubs, topicStr);
            }
                break;
            default:
                break;
        }
    }
    return comm;
}

std::unique_ptr<Light> ROSScenarioParser::ParseLight(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Light> l = ScenarioParser::ParseLight(element, namePrefix);
    if(l != nullptr)
    {
        ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
        ros::NodeHandle& nh = sim->GetNodeHandle();
        std::unordered_map<std::string, ros::Subscriber>& subs = sim->GetSubscribers();
        XMLElement* item;

        //Online update of light origin frame
        const char* originTopic = nullptr;
        if((item = element->FirstChildElement("ros_subscriber")) != nullptr
            && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
        {
            subs[l->GetName()] = nh.subscribe<geometry_msgs::Transform>(std::string(originTopic), 1, ActuatorOriginCallback(l.get()));
        }

        //Service for switching the light
        const char* switchTopic = nullptr;
        if((item = element->FirstChildElement("ros_service")) != nullptr
            && item->QueryStringAttribute("topic", &switchTopic) == XML_SUCCESS)
        {
            std::unordered_map<std::string, ros::ServiceServer>& srvs = sim->GetServiceServers();
            srvs[l->GetName()] = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(std::string(switchTopic), LightService(l.get()));
        }
    }
    return l;
}

bool ROSScenarioParser::ParseContact(XMLElement* element)
{
    if(!ScenarioParser::ParseContact(element))
        return false;

    ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
    ros::NodeHandle& nh = sim->GetNodeHandle();
    std::unordered_map<std::string, ros::Publisher>& pubs = sim->GetPublishers();

    //Contact info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string contactName = std::string(name);

    //Publishing info
    XMLElement* item;
    const char* topic = nullptr;
    if((item = element->FirstChildElement("ros_publisher")) == nullptr
        || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
    {
        return true;
    }
    std::string topicStr(topic);
    pubs[contactName] = nh.advertise<visualization_msgs::Marker>(topicStr, 10);
    return true;
}

FixedJoint* ROSScenarioParser::ParseGlue(XMLElement* element)
{
    FixedJoint* glue = ScenarioParser::ParseGlue(element);
    if(glue != nullptr)
    {
        XMLElement* item;
        const char* topic = nullptr;
        if ((item = element->FirstChildElement("ros_service")) == nullptr
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return glue;
        }

        ROSSimulationManager* sim = static_cast<ROSSimulationManager*>(GetSimulationManager());
        ros::NodeHandle& nh = sim->GetNodeHandle();
        std::unordered_map<std::string, ros::ServiceServer>& srvs = sim->GetServiceServers();
        srvs[glue->GetName()] = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(std::string(topic), GlueService(sim, glue));
    }
    return glue;
}

}