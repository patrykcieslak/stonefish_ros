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
//  Copyright (c) 2022-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSControlInterface.h"

namespace sf
{

ROSControlInterface::ROSControlInterface(Robot* robot, const std::vector<std::string>& jointNames, ServoControlMode ifType, std::string_view _namespace)
    : robot(robot), mode(ifType), nh(std::string(_namespace))
{    
    //Find actuators connected to joints
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        size_t id = 0;
        Actuator* act;
        while((act = robot->GetActuator(id++)) != nullptr)
        {
            if( act->GetType() == ActuatorType::JOINT
                && static_cast<JointActuator*>(act)->GetJointActuatorType() == JointActuatorType::SERVO
                && static_cast<Servo*>(act)->GetJointName() == jointNames[i])
            {
                actuators.push_back(act->GetName());   
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
            jcif = std::make_unique<hardware_interface::PositionJointInterface>();
            registerInterface(static_cast<hardware_interface::PositionJointInterface*>(jcif.get()));
        }
            break;
        
        case ServoControlMode::VELOCITY:
        case ServoControlMode::TORQUE: // Effort interface not implemented at this time!
        {
            jcif = std::make_unique<hardware_interface::VelocityJointInterface>();
            registerInterface(static_cast<hardware_interface::VelocityJointInterface*>(jcif.get()));
        }
            break;
    }

    for(size_t i=0; i<actuators.size(); ++i)
    {
        hardware_interface::JointHandle cmdHandle(jsif.getHandle(jointNames[i]), &cmd[i]);
        jcif->registerHandle(cmdHandle);
    }

    // Create controller manager
    cm = std::make_unique<controller_manager::ControllerManager>(this, nh);
}

void ROSControlInterface::read()
{
    for(size_t i=0; i<actuators.size(); ++i)
    {
        Servo* srv = static_cast<Servo*>(robot->GetActuator(actuators[i]));    
        eff[i] = srv->GetEffort();
        vel[i] = srv->GetVelocity();
        pos[i] = srv->GetPosition();
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
                Servo* srv = static_cast<Servo*>(robot->GetActuator(actuators[i]));
                srv->SetControlMode(ServoControlMode::POSITION);
                srv->SetDesiredPosition(cmd[i]);
            }
        }
        break;

        case ServoControlMode::VELOCITY:
        case ServoControlMode::TORQUE:
        {
            for(size_t i=0; i<actuators.size(); ++i)
            {
                Servo* srv = static_cast<Servo*>(robot->GetActuator(actuators[i]));
                srv->SetControlMode(ServoControlMode::VELOCITY);
                srv->SetDesiredVelocity(cmd[i]);
            }
        }
        break;
    }
}

}
