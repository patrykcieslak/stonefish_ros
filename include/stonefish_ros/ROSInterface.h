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
//  ROSInterface.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 30/11/17.
//  Copyright (c) 2017-2020 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROSInterface__
#define __Stonefish_ROSInterface__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <Stonefish/StonefishCommon.h>

namespace sf
{
    class IMU;
    class Pressure;
    class DVL;
    class GPS;
    class Odometry;
    class ForceTorque;
    class RotaryEncoder;
    class Camera;
    class Multibeam;
    class Multibeam2;
    class FLS;
    class SSS;
    class Contact;
    class USBL;

    class ROSInterface
    {
    public:
        static void PublishTF(tf::TransformBroadcaster& broadcaster, const Transform& T, const ros::Time& t, const std::string &frame_id, const std::string &child_frame_id);
        static void PublishIMU(ros::Publisher& pub, IMU* imu);
        static void PublishPressure(ros::Publisher& pub, Pressure* press);
        static void PublishDVL(ros::Publisher& pub, ros::Publisher& altPub, DVL* dvl);
        static void PublishGPS(ros::Publisher& pub, GPS* gps);
        static void PublishOdometry(ros::Publisher& pub, Odometry* odom);
        static void PublishForceTorque(ros::Publisher& pub, ForceTorque* ft);
        static void PublishEncoder(ros::Publisher& pub, RotaryEncoder* enc);
        static void PublishLaserScan(ros::Publisher& laserScanPub, Multibeam* mbes);
        static void PublishPointCloud(ros::Publisher& pointCloudPub, Multibeam2* mb);
        static void PublishContact(ros::Publisher& contactPub, Contact* cnt);
        static void PublishUSBL(ros::Publisher& usblPub, USBL* usbl);

        static std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr> GenerateCameraMsgPrototypes(Camera* cam, bool depth);
        static std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr> GenerateFLSMsgPrototypes(FLS* fls);
        static std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr> GenerateSSSMsgPrototypes(SSS* sss);

    private:
        ROSInterface();
    };
}

#endif