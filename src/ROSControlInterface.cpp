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
//  ROSControlInterface.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 27/07/22.
//  Copyright (c) 2022-2023 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSControlInterface.h"

namespace sf
{

ROSControlInterface::ROSControlInterface(Robot* robot, const std::vector<std::string>& jointNames, ServoControlMode ifType, const std::string& _namespace)
    : robot(robot), mode(ifType), nh(_namespace)
{    
    //Find actuators connected to joints
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        size_t id = 0;
        Actuator* act;
        while((act = robot->getActuator(id++)) != nullptr)
        {
            if(act->getType() == ActuatorType::SERVO && ((Servo*)act)->getJointName() == jointNames[i])
            {
                actuators.push_back(act->getName());   
                break;
            }
        }
    }
    if(actuators.size() != jointNames.size())
        ROS_ERROR_STREAM("Missing some actuators for the given joint names. Found " << actuators.size() << " of " << jointNames.size() << ".");

    // Resize memory
    cmd.resize(actuators.size(), 0.0);
    eff.resize(actuators.size(), 0.0);
    vel.resize(actuators.size(), 0.0);
    pos.resize(actuators.size(), 0.0);

    // Connect and register the joint state interface
    for(size_t i=0; i<actuators.size(); ++i)
    {
        hardware_interface::JointStateHandle stateHandle(jointNames[i], &pos[i], &vel[i], &eff[i]);
        jsif.registerHandle(stateHandle);
    }
    registerInterface(&jsif);

    // Connect and register the joint command interfaces
    switch (mode)
    {
        case ServoControlMode::POSITION:
        {
            jcif = new hardware_interface::PositionJointInterface();
            registerInterface((hardware_interface::PositionJointInterface*)jcif);
        }
            break;
        
        case ServoControlMode::VELOCITY:
        case ServoControlMode::TORQUE: // Effort interface not implemented at this time!
        {
            jcif = new hardware_interface::VelocityJointInterface();
            registerInterface((hardware_interface::VelocityJointInterface*)jcif);
        }
            break;
    }

    for(size_t i=0; i<actuators.size(); ++i)
    {
        hardware_interface::JointHandle cmdHandle(jsif.getHandle(jointNames[i]), &cmd[i]);
        jcif->registerHandle(cmdHandle);
    }

    // Create controller manager
    cm = new controller_manager::ControllerManager(this, nh);
}

ROSControlInterface::~ROSControlInterface()
{
    delete jcif;
    delete cm;
}

void ROSControlInterface::read()
{
    for(size_t i=0; i<actuators.size(); ++i)
    {
        Servo* srv = (Servo*)robot->getActuator(actuators[i]);    
        eff[i] = srv->getEffort();
        vel[i] = srv->getVelocity();
        pos[i] = srv->getPosition();
    }
}

void ROSControlInterface::update(const ros::Time& time, const ros::Duration& period)
{
    cm->update(time, period);
}

void ROSControlInterface::write()
{
    switch(mode)
    {
        case ServoControlMode::POSITION:
        {
            for(size_t i=0; i<actuators.size(); ++i)
            {
                Servo* srv = (Servo*)robot->getActuator(actuators[i]);
                srv->setControlMode(ServoControlMode::POSITION);
                srv->setDesiredPosition(cmd[i]);
            }
        }
        break;

        case ServoControlMode::VELOCITY:
        case ServoControlMode::TORQUE:
        {
            for(size_t i=0; i<actuators.size(); ++i)
            {
                Servo* srv = (Servo*)robot->getActuator(actuators[i]);
                srv->setControlMode(ServoControlMode::VELOCITY);
                srv->setDesiredVelocity(cmd[i]);
            }
        }
        break;
    }
}

}
