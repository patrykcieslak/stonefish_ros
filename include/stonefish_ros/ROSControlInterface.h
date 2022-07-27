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
//  ROSControlInterface.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 27/07/22.
//  Copyright (c) 2022 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROSControlInterface__
#define __Stonefish_ROSControlInterface__

//Stonefish
#include <Stonefish/core/Robot.h>
#include <Stonefish/actuators/Servo.h>
//ROS
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <array>

namespace sf
{
    class ROSControlInterface : public hardware_interface::RobotHW
    {
    public:
        ROSControlInterface(Robot* robot, const std::vector<std::string>& jointNames, ServoControlMode ifType, const std::string& _namespace);
        virtual ~ROSControlInterface();
        void read();
        void update(const ros::Time& time, const ros::Duration& period);
        void write();

    private:
        Robot* robot;
        std::vector<std::string> actuators;
        ServoControlMode mode;
        std::vector<double> cmd;
        std::vector<double> eff;
        std::vector<double> vel;
        std::vector<double> pos;

        hardware_interface::JointStateInterface jsif;
        hardware_interface::JointCommandInterface* jcif;
        controller_manager::ControllerManager* cm;
        ros::NodeHandle nh;
    };
}

#endif