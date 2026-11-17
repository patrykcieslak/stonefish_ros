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
//  ROSCommPluginInterface.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 17/11/26.
//  Copyright (c) 2026 Patryk Cieslak. All rights reserved.

#pragma once

#include <ros/ros.h>

namespace sf
{
    //! A class representing an interface for communication device plugins in ROS2.
    class ROSCommPluginInterface
    {
    public:
        //! A virtual destructor.
        virtual ~ROSCommPluginInterface() = default;

        //! A pure virtual method for creating ROS publishers for the communication device plugin.
        /*!
         \param nh a shared pointer to the ROS node handle
         \param pubs a map to store the created publishers, with communication device names as keys
         \param pubTopics a map storing all parsed publisher topics
         */
        virtual void CreatePublishers(ros::NodeHandle& nh, 
            std::unordered_map<std::string, ros::Publisher>& pubs, 
            std::string_view pubTopic) const = 0;

        //! A pure virtual method for publishing communication device data.
        /*!
         \param nh a shared pointer to the ROS node handle
         \param pubs a map of the available publishers, with communication device names as keys
         */
        virtual void Publish(ros::NodeHandle& nh, 
            std::unordered_map<std::string, ros::Publisher>& pubs) const = 0;
    };
}