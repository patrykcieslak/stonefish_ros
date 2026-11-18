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
//  ROSInterface.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 30/11/17.
//  Copyright (c) 2017-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros/ROSInterface.h"

#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/INS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/scalar/Profiler.h>
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
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/NED.h>

#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <stonefish_ros/DVL.h>
#include <stonefish_ros/INS.h>
#include <stonefish_ros/Int32Stamped.h>
#include <stonefish_ros/BeaconInfo.h>
#include <stonefish_ros/Event.h>
#include <stonefish_ros/EventArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace sf
{

void ROSInterface::PublishTF(tf::TransformBroadcaster& broadcaster, const Transform& T, const ros::Time& t, const std::string &frame_id, const std::string &child_frame_id)
{
    Vector3 o = T.getOrigin();
    Quaternion q = T.getRotation();
    tf::Transform TF;
    TF.setOrigin(tf::Vector3(o.x(), o.y(), o.z()));
    TF.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    broadcaster.sendTransform(tf::StampedTransform(TF, t, frame_id, child_frame_id));
}

void ROSInterface::PublishAccelerometer(ros::Publisher& pub, Accelerometer* acc)
{
    Sample s = acc->GetLastSample();
    Vector3 accelStdDev = Vector3(acc->GetSensorChannelDescription(0).stdDev, 
                                  acc->GetSensorChannelDescription(1).stdDev,
                                  acc->GetSensorChannelDescription(2).stdDev);
    
    geometry_msgs::AccelWithCovarianceStamped msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = acc->GetName();
    msg.accel.accel.linear.x = s.GetValue(0);
    msg.accel.accel.linear.y = s.GetValue(1);
    msg.accel.accel.linear.z = s.GetValue(2);
    msg.accel.covariance[0] = accelStdDev.getX() * accelStdDev.getX();
    msg.accel.covariance[7] = accelStdDev.getY() * accelStdDev.getY();
    msg.accel.covariance[14] = accelStdDev.getZ() * accelStdDev.getZ();
    pub.publish(msg);
}

void ROSInterface::PublishGyroscope(ros::Publisher& pub, Gyroscope* gyro)
{
    Sample s = gyro->GetLastSample();
    Vector3 avelocityStdDev = Vector3(gyro->GetSensorChannelDescription(0).stdDev, 
                                      gyro->GetSensorChannelDescription(1).stdDev,
                                      gyro->GetSensorChannelDescription(2).stdDev);

    geometry_msgs::TwistWithCovarianceStamped msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = gyro->GetName();
    msg.twist.twist.angular.x = s.GetValue(0);
    msg.twist.twist.angular.y = s.GetValue(1);
    msg.twist.twist.angular.z = s.GetValue(2);
    msg.twist.covariance[21] = avelocityStdDev.getX() * avelocityStdDev.getX();
    msg.twist.covariance[28] = avelocityStdDev.getY() * avelocityStdDev.getY();
    msg.twist.covariance[35] = avelocityStdDev.getZ() * avelocityStdDev.getZ();
    pub.publish(msg);
}

void ROSInterface::PublishIMU(ros::Publisher& pub, IMU* imu)
{
    Sample s = imu->GetLastSample();
    Vector3 rpy = Vector3(s.GetValue(0), s.GetValue(1), s.GetValue(2));
    Quaternion quat(rpy.z(), rpy.y(), rpy.x());
    Vector3 angleStdDev = Vector3(imu->GetSensorChannelDescription(0).stdDev,
                                  imu->GetSensorChannelDescription(1).stdDev,
                                  imu->GetSensorChannelDescription(2).stdDev);
    Vector3 avelocityStdDev = Vector3(imu->GetSensorChannelDescription(3).stdDev,
                                      imu->GetSensorChannelDescription(4).stdDev,
                                      imu->GetSensorChannelDescription(5).stdDev);
    Vector3 accStdDev = Vector3(imu->GetSensorChannelDescription(6).stdDev,
                                imu->GetSensorChannelDescription(7).stdDev,
                                imu->GetSensorChannelDescription(8).stdDev);
    //Variance is sigma^2!
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = imu->GetName();
    msg.orientation.x = quat.x();
    msg.orientation.y = quat.y();
    msg.orientation.z = quat.z();
    msg.orientation.w = quat.w();
    msg.orientation_covariance[0] = angleStdDev.getX() * angleStdDev.getX();
    msg.orientation_covariance[4] = angleStdDev.getY() * angleStdDev.getY();
    msg.orientation_covariance[8] = angleStdDev.getZ() * angleStdDev.getZ();
    msg.angular_velocity.x = s.GetValue(3);
    msg.angular_velocity.y = s.GetValue(4);
    msg.angular_velocity.z = s.GetValue(5);
    msg.angular_velocity_covariance[0] = avelocityStdDev.getX() * avelocityStdDev.getX();
    msg.angular_velocity_covariance[4] = avelocityStdDev.getY() * avelocityStdDev.getY();
    msg.angular_velocity_covariance[8] = avelocityStdDev.getZ() * avelocityStdDev.getZ();
    msg.linear_acceleration.x = s.GetValue(6);
    msg.linear_acceleration.y = s.GetValue(7);
    msg.linear_acceleration.z = s.GetValue(8);
    msg.linear_acceleration_covariance[0] = accStdDev.getX() * accStdDev.getX();
    msg.linear_acceleration_covariance[4] = accStdDev.getY() * accStdDev.getY();
    msg.linear_acceleration_covariance[8] = accStdDev.getZ() * accStdDev.getZ();
    pub.publish(msg);
}

void ROSInterface::PublishPressure(ros::Publisher& pub, Pressure* press)
{
    Sample s = press->GetLastSample();
    sensor_msgs::FluidPressure msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = press->GetName();
    msg.fluid_pressure = s.GetValue(0);
    msg.variance = press->GetSensorChannelDescription(0).stdDev;
    msg.variance *= msg.variance; //Variance is square of standard deviation
    pub.publish(msg);
}

void ROSInterface::PublishDVL(ros::Publisher& pub, DVL* dvl)
{
    //Get data
    Sample s = dvl->GetLastSample();
    unsigned short status = (unsigned short)trunc(s.GetValue(7));
    Scalar vVariance = dvl->GetSensorChannelDescription(0).stdDev;
    vVariance *= vVariance; //Variance is square of standard deviation
    //Publish DVL message
    stonefish_ros::DVL msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = dvl->GetName();
    msg.velocity.x = s.GetValue(0);
    msg.velocity.y = s.GetValue(1);
    msg.velocity.z = s.GetValue(2);
    msg.velocity_covariance[0] = vVariance;
    msg.velocity_covariance[4] = vVariance;
    msg.velocity_covariance[8] = vVariance;
    msg.altitude = (status == 0 || status == 2) ? s.GetValue(3) : -1.0;
    pub.publish(msg);
}

void ROSInterface::PublishDVLAltitude(ros::Publisher& pub, DVL* dvl)
{
    //Get data
    Sample s = dvl->GetLastSample();
    unsigned short status = (unsigned short)trunc(s.GetValue(7));
    Vector3 velMax;
    Scalar altMin, altMax;
    dvl->GetRange(velMax, altMin, altMax);
    Scalar beamAngle = dvl->GetBeamAngle();
    //Publish range message
    sensor_msgs::Range msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = dvl->GetName() + "_altitude";
    msg.radiation_type = msg.ULTRASOUND;
    msg.field_of_view = beamAngle*2;
    msg.min_range = altMin;
    msg.max_range = altMax;
    msg.range = (status == 0 || status == 2) ? s.GetValue(3) : -1.0;
    pub.publish(msg);
}

void ROSInterface::PublishGPS(ros::Publisher& pub, GPS* gps)
{
    Sample s = gps->GetLastSample();

    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = gps->GetName();
    msg.status.service = msg.status.SERVICE_GPS;

    if(s.GetValue(0) > Scalar(90) && s.GetValue(1) > Scalar(180)) //Underwater
    {
        msg.status.status = msg.status.STATUS_NO_FIX;
        msg.latitude = 0.0;
        msg.longitude = 0.0;
        msg.altitude = 0.0;
    }
    else
    {
        msg.status.status = msg.status.STATUS_FIX;
        msg.latitude = s.GetValue(0);
        msg.longitude = s.GetValue(1);
        msg.altitude = 0.0;
    }

    msg.position_covariance[0] = msg.position_covariance[4] = gps->GetNoise() * gps->GetNoise();
    msg.position_covariance[8] = 1.0;
    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub.publish(msg);
}

void ROSInterface::PublishOdometry(ros::Publisher& pub, Odometry* odom)
{
    Sample s = odom->GetLastSample();
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = "world_ned";
    msg.child_frame_id = odom->GetName();
    msg.pose.pose.position.x = s.GetValue(0);
    msg.pose.pose.position.y = s.GetValue(1);
    msg.pose.pose.position.z = s.GetValue(2);
    msg.twist.twist.linear.x = s.GetValue(3);
    msg.twist.twist.linear.y = s.GetValue(4);
    msg.twist.twist.linear.z = s.GetValue(5);
    msg.pose.pose.orientation.x = s.GetValue(6);
    msg.pose.pose.orientation.y = s.GetValue(7);
    msg.pose.pose.orientation.z = s.GetValue(8);
    msg.pose.pose.orientation.w = s.GetValue(9);
    msg.twist.twist.angular.x = s.GetValue(10);
    msg.twist.twist.angular.y = s.GetValue(11);
    msg.twist.twist.angular.z = s.GetValue(12);
    pub.publish(msg);
}


void ROSInterface::PublishINS(ros::Publisher& pub, INS* ins)
{
    Scalar lat, lon, h;
    SimulationApp::GetApp()->GetSimulationManager()->GetNed()->Ned2Geodetic(0.0, 0.0, 0.0, lat, lon, h);

    Sample s = ins->GetLastSample();
    stonefish_ros::INS msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = ins->GetName();
    
    msg.latitude = s.GetValue(4);
    msg.longitude = s.GetValue(5);
    msg.origin_latitude = lat;
    msg.origin_longitude = lon;

    msg.pose.north = s.GetValue(0);
    msg.pose.east = s.GetValue(1);
    msg.pose.down = s.GetValue(2);
    msg.pose.roll = s.GetValue(9);
    msg.pose.pitch = s.GetValue(10);
    msg.pose.yaw = s.GetValue(11);
    msg.altitude = s.GetValue(3);
    
    msg.body_velocity.x = s.GetValue(6);
    msg.body_velocity.y = s.GetValue(7);
    msg.body_velocity.z = s.GetValue(8);
    msg.rpy_rate.x = s.GetValue(12);
    msg.rpy_rate.y = s.GetValue(13);
    msg.rpy_rate.z = s.GetValue(14);

    pub.publish(msg);
}

void ROSInterface::PublishINSOdometry(ros::Publisher& pub, INS* ins)
{
    Sample s = ins->GetLastSample();
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = "world_ned";
    msg.child_frame_id = ins->GetName();
    msg.pose.pose.position.x = s.GetValue(0);
    msg.pose.pose.position.y = s.GetValue(1);
    msg.pose.pose.position.z = s.GetValue(2);
    msg.twist.twist.linear.x = s.GetValue(6);
    msg.twist.twist.linear.y = s.GetValue(7);
    msg.twist.twist.linear.z = s.GetValue(8);
    Quaternion q(s.GetValue(11), s.GetValue(10), s.GetValue(9));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.angular.x = s.GetValue(12);
    msg.twist.twist.angular.y = s.GetValue(13);
    msg.twist.twist.angular.z = s.GetValue(14);
    pub.publish(msg);
}

void ROSInterface::PublishForceTorque(ros::Publisher& pub, ForceTorque* ft)
{
    Sample s = ft->GetLastSample();    
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = ft->GetName();
    msg.wrench.force.x = s.GetValue(0);
    msg.wrench.force.y = s.GetValue(1);
    msg.wrench.force.z = s.GetValue(2);
    msg.wrench.torque.x = s.GetValue(3);
    msg.wrench.torque.y = s.GetValue(4);
    msg.wrench.torque.z = s.GetValue(5);
    pub.publish(msg);
}

void ROSInterface::PublishEncoder(ros::Publisher& pub, RotaryEncoder* enc)
{
    Sample s = enc->GetLastSample();
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = enc->GetName();
    msg.name.resize(1);
    msg.position.resize(1);
    msg.velocity.resize(1);
    msg.name[0] = enc->GetJointName();
    msg.position[0] = s.GetValue(0);
    msg.velocity[0] = s.GetValue(1);
    pub.publish(msg);
}

void ROSInterface::PublishMultibeam(ros::Publisher& pub, Multibeam* mb)
{
    Sample s = mb->GetLastSample();
    SensorChannel channel = mb->GetSensorChannelDescription(0);
    std::vector<Scalar> distances = s.GetData();

    Scalar angRange = mb->GetAngleRange();
    size_t angSteps = distances.size();

    sensor_msgs::LaserScan msg;
    msg.header.stamp = ros::Time(s.GetTimestamp());
    msg.header.frame_id = mb->GetName();
    
    msg.angle_min = -angRange/Scalar(2); // start angle of the scan [rad]
    msg.angle_max = angRange/Scalar(2); // end angle of the scan [rad]
    msg.angle_increment = angRange/Scalar(angSteps-1); // angular distance between measurements [rad]
    msg.range_min = channel.rangeMin; // minimum range value [m]
    msg.range_max = channel.rangeMax; // maximum range value [m]
    msg.time_increment = 0.; // time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
    msg.scan_time = 0.; // time between scans [seconds]
    
    msg.ranges.resize(angSteps); // range data [m]
    msg.intensities.resize(angSteps); // used to say if measurement is valid

    for(size_t i = 0; i<angSteps; ++i)
    {
        if(distances[i] < channel.rangeMax)
        {
            msg.ranges[i] = distances[i];
            msg.intensities[i] = 1.0;
        }
        else
        {
            msg.ranges[i] = std::numeric_limits<float>::infinity();
            msg.intensities[i] = 0.0;
        }
    }

    pub.publish(msg);
}

void ROSInterface::PublishMultibeamPCL(ros::Publisher& pub, Multibeam* mb)
{
    Sample s = mb->GetLastSample();
    SensorChannel channel = mb->GetSensorChannelDescription(0);
    std::vector<Scalar> distances = s.GetData();
    Scalar angRange = mb->GetAngleRange();
    size_t angSteps = distances.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = mb->GetName();
    msg->header.seq = s.GetId(); // In this message it does not increase automatically
    msg->height = msg->width = 1;
    Scalar angleMin = -angRange / Scalar(2);                  // start angle of the scan [rad]
    Scalar angleIncrement = angRange / Scalar(angSteps - 1);  // angular distance between measurements [rad]

    for(size_t i = 0; i < angSteps; ++i)
        if(distances[i] < channel.rangeMax && distances[i] > channel.rangeMin) // Only publish good points
        {  
            double angle = angleMin + i * angleIncrement;
            pcl::PointXYZ pt;
            pt.y = btSin(angle) * distances[i];
            pt.x = btCos(angle) * distances[i];
            pt.z = 0.0;
            msg->push_back(pt);
        }

    pcl_conversions::toPCL(ros::Time(s.GetTimestamp()), msg->header.stamp);
    try
    {
        pub.publish(msg);
    }
    catch (ros::serialization::StreamOverrunException& soe)
    {
        ROS_ERROR_STREAM("Stream overrun exception while publishing multibeam data: " << soe.what());
    }
    catch (std::runtime_error& e)
    {
        ROS_ERROR_STREAM("Runtime error whle publishing multibeam data: " << e.what());
    }
}

void ROSInterface::PublishProfiler(ros::Publisher& pub, Profiler* prof)
{
    std::unique_ptr<std::vector<Sample>> hist = prof->GetHistory();
    SensorChannel channel = prof->GetSensorChannelDescription(1); // range channel

    sensor_msgs::LaserScan msg;
    msg.header.stamp = ros::Time(hist->back().GetTimestamp());
    msg.header.frame_id = prof->GetName();
    
    msg.angle_min = hist->front().GetValue(0);
    msg.angle_max = hist->back().GetValue(0);
    msg.range_min = channel.rangeMin; // minimum range value [m]
    msg.range_max = channel.rangeMax; // maximum range value [m]
    msg.angle_increment = hist->size() == 1 ? 0.0 : hist->at(1).GetValue(0) - hist->at(0).GetValue(0);
    msg.time_increment = hist->size() == 1 ? 0.0 : hist->at(1).GetTimestamp() - hist->at(0).GetTimestamp();
    msg.scan_time = hist->back().GetTimestamp() - hist->front().GetTimestamp();
    
    if(hist->size() == 1) // RVIZ does not display LaserScan with one range
    {
        msg.ranges.resize(2);
        msg.intensities.resize(2);
        msg.ranges[0] = hist->front().GetValue(1);
        msg.intensities[0] = msg.ranges[0] == msg.range_max ? 0.1 : 1.0;
        msg.ranges[1] = msg.ranges[0];
        msg.intensities[1] = msg.intensities[0];
    }
    else
    {
        msg.ranges.resize(hist->size());
        msg.intensities.resize(hist->size());
        for(size_t i=0; i<hist->size(); ++i)
        {
            msg.ranges[i] = hist->at(i).GetValue(1);
            msg.intensities[i] = msg.ranges[i] == msg.range_max ? 0.1 : 1.0;
        }
    }

    pub.publish(msg);
}

void ROSInterface::PublishLidar(ros::Publisher& pub, Lidar* lidar)
{
    uint32_t hRes, vRes;
    lidar->GetResolution(hRes, vRes);
    glm::vec2 range = lidar->GetRangeLimits();
    float hFovRad = lidar->GetHorizontalFov()/180.f*M_PI;
    float vFovRad = lidar->GetVerticalFov()/180.f*M_PI; 
    float hStepAngleRad = hFovRad/(float)(hRes-1);
    float vStepAngleRad = vFovRad/(float)(vRes-1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = lidar->GetName();
    msg->height = msg->width = 1;

    float* data = (float*)lidar->GetRangeDataPointer();

    for(uint32_t v=0; v<vRes; ++v)
    {
        uint32_t offset = v*hRes;
        float hAngleRad = -hFovRad/2.f + (0.5f/hRes*hFovRad);
        float vAngleRad = vFovRad/2.f - ((0.5f+v)/vRes*vFovRad);
        
        for(uint32_t h=0; h<hRes; ++h)
        {
            float depth = data[offset + h];
            if(depth > range.x && depth < range.y) // Only publish good points
            {
                Eigen::Vector3f lidarPoint = Eigen::Vector3f(tanf(hAngleRad), tanf(vAngleRad), 1.f).normalized() * depth;
                pcl::PointXYZ pt;
                pt.x = lidarPoint.x();
                pt.y = lidarPoint.y();
                pt.z = lidarPoint.z();
                msg->push_back(pt);
            }
            hAngleRad += hStepAngleRad;
        }
    }
    
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    try
    {
        pub.publish(msg);
    }
    catch (ros::serialization::StreamOverrunException& soe)
    {
        ROS_ERROR_STREAM("Stream overrun exception while publishing multibeam data: " << soe.what());
    }
    catch (std::runtime_error& e)
    {
        ROS_ERROR_STREAM("Runtime error whle publishing multibeam data: " << e.what());
    }
}

void ROSInterface::PublishContact(ros::Publisher& pub, Contact* cnt)
{
    if(cnt->GetHistory().size() == 0)
        return;

    ContactPoint cp = cnt->GetHistory().back();
    
    //Publish marker message
    visualization_msgs::Marker msg;
    msg.header.frame_id = "world_ned";
    msg.header.stamp = ros::Time::now();
    msg.ns = cnt->GetName();
    msg.id = 0;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.points.resize(2);
    msg.points[0].x = cp.locationA.getX();
    msg.points[0].y = cp.locationA.getY();
    msg.points[0].z = cp.locationA.getZ();
    msg.points[1].x = cp.locationA.getX() + cp.normalForceA.getX();
    msg.points[1].y = cp.locationA.getY() + cp.normalForceA.getY();
    msg.points[1].z = cp.locationA.getZ() + cp.normalForceA.getZ();
    msg.color.r = 1.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    pub.publish(msg);
}

void ROSInterface::PublishUSBL(ros::Publisher& pub, ros::Publisher& pub_info, USBL* usbl)
{
    std::map<uint64_t, BeaconInfo>& beacons = usbl->GetBeaconInfo();
    if(beacons.size() == 0)
        return;   

    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    stonefish_ros::BeaconInfo info;
    
    marker.header.frame_id = usbl->GetName();
    marker.header.stamp = ros::Time::now();
    info.header.frame_id = marker.header.frame_id;
    info.header.stamp = marker.header.stamp;

    marker.ns = usbl->GetName();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    for(auto it = beacons.begin(); it!=beacons.end(); ++it)
    {
        marker.id = it->first;
        Vector3 pos = it->second.relPos;
        marker.pose.position.x = pos.getX();
        marker.pose.position.y = pos.getY();
        marker.pose.position.z = pos.getZ();
        msg.markers.push_back(marker);    

        info.beacon_id = it->first;
        info.range = it->second.range;
        info.azimuth = it->second.azimuth;
        info.elevation = it->second.elevation;
        info.relative_position.x = it->second.relPos.getX();
        info.relative_position.y = it->second.relPos.getY();
        info.relative_position.z = it->second.relPos.getZ();
        info.local_orientation.x = it->second.localOri.getX();
        info.local_orientation.y = it->second.localOri.getY();
        info.local_orientation.z = it->second.localOri.getZ();
        info.local_orientation.w = it->second.localOri.getW();
        info.local_depth = it->second.localDepth;
        pub_info.publish(info);
    }
    pub.publish(msg);
}

void ROSInterface::PublishTrajectoryState(ros::Publisher& odom, ros::Publisher& iter, AnimatedEntity* anim)
{
    Trajectory* tr = anim->GetTrajectory();
    Transform T = tr->GetInterpolatedTransform();
    Vector3 p = T.getOrigin();
    Quaternion q = T.getRotation();
    Vector3 v = tr->GetInterpolatedLinearVelocity();
    Vector3 omega = tr->GetInterpolatedAngularVelocity();

    //Odometry message
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world_ned";
    msg.header.stamp = ros::Time::now();
    msg.child_frame_id = anim->GetName();
    msg.pose.pose.position.x = p.x();
    msg.pose.pose.position.y = p.y();
    msg.pose.pose.position.z = p.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = v.x();
    msg.twist.twist.linear.y = v.y();
    msg.twist.twist.linear.z = v.z();
    msg.twist.twist.angular.x = omega.x();
    msg.twist.twist.angular.y = omega.y();
    msg.twist.twist.angular.z = omega.z();
    odom.publish(msg);

    //Iteration message
    stonefish_ros::Int32Stamped msg2;
    msg2.header = msg.header;
    msg2.data = (int32_t)tr->GetPlaybackIteration();
    iter.publish(msg2);
}

void ROSInterface::PublishEventBasedCamera(ros::Publisher& pub, EventBasedCamera* ebc)
{
    //Get access to event texture
    int32_t* data = (int32_t*)ebc->GetImageDataPointer();

    //Event array message
    stonefish_ros::EventArray msg;
    msg.header.frame_id = ebc->GetName();
    msg.header.stamp = ros::Time::now();
    ebc->GetResolution(msg.width, msg.height);
    msg.events.resize(ebc->GetLastEventCount());
    for(size_t i=0; i<msg.events.size(); ++i)
    {
        //First 4 bytes - pixel coords
        msg.events[i].x = (unsigned int)(data[i*2] >> 16);
        msg.events[i].y = (unsigned int)(data[i*2] & 0xFFFF); 
        //Next 4 bytes - polarity and time
        msg.events[i].ts = msg.header.stamp + ros::Duration(0, abs(data[i*2+1]));
        msg.events[i].polarity = data[i*2+1] > 0;
    }
    pub.publish(msg);
}

std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr> ROSInterface::GenerateCameraMsgPrototypes(Camera* cam, bool depth)
{
    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = cam->GetName();
	cam->GetResolution(img->width, img->height);
	img->encoding = depth ? "32FC1" : "rgb8";
	img->is_bigendian = 0;
    img->step = img->width * (depth ? sizeof(float) : 3);
    img->data.resize(img->step * img->height);

	//Camera info message
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
	info->header.frame_id = cam->GetName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->D.resize(5, 0.0);
    //Rectification (for stereo only)
    info->R[0] = 1.0;
    info->R[4] = 1.0;
    info->R[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->GetHorizontalFov()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->K[2] = (double)info->width/2.0; //cx
    info->K[5] = (double)info->height/2.0; //cy
    info->K[0] = info->K[2]/tanhfov2; //fx
    info->K[4] = info->K[5]/tanvfov2; //fy 
    info->K[8] = 1.0;
    //Projection
    info->P[2] = info->K[2]; //cx'
    info->P[6] = info->K[5]; //cy'
    info->P[0] = info->K[0]; //fx';
    info->P[5] = info->K[4]; //fy';
    info->P[3] = 0.0; //Tx - position of second camera from stereo pair
    info->P[7] = 0.0; //Ty;
    info->P[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;
	
    return std::make_pair(img, info);
}

std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr> ROSInterface::GenerateThermalCameraMsgPrototypes(ThermalCamera* cam)
{
    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = cam->GetName();
	cam->GetResolution(img->width, img->height);
	img->encoding = "32FC1";
	img->is_bigendian = 0;
    img->step = img->width * sizeof(float);
    img->data.resize(img->step * img->height);

	//Camera info message
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
	info->header.frame_id = cam->GetName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->D.resize(5, 0.0);
    //Rectification (for stereo only)
    info->R[0] = 1.0;
    info->R[4] = 1.0;
    info->R[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->GetHorizontalFov()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->K[2] = (double)info->width/2.0; //cx
    info->K[5] = (double)info->height/2.0; //cy
    info->K[0] = info->K[2]/tanhfov2; //fx
    info->K[4] = info->K[5]/tanvfov2; //fy 
    info->K[8] = 1.0;
    //Projection
    info->P[2] = info->K[2]; //cx'
    info->P[6] = info->K[5]; //cy'
    info->P[0] = info->K[0]; //fx';
    info->P[5] = info->K[4]; //fy';
    info->P[3] = 0.0; //Tx - position of second camera from stereo pair
    info->P[7] = 0.0; //Ty;
    info->P[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = cam->GetName();
	disp->width = img->width;
    disp->height = img->height;
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
	
    return std::make_tuple(img, info, disp);
}

std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr> ROSInterface::GenerateOpticalFlowCameraMsgPrototypes(OpticalFlowCamera* cam)
{
    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = cam->GetName();
	cam->GetResolution(img->width, img->height);
	img->encoding = "32FC2";
	img->is_bigendian = 0;
    img->step = img->width * 2 * sizeof(float);
    img->data.resize(img->step * img->height);

	//Camera info message
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
	info->header.frame_id = cam->GetName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->D.resize(5, 0.0);
    //Rectification (for stereo only)
    info->R[0] = 1.0;
    info->R[4] = 1.0;
    info->R[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->GetHorizontalFov()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->K[2] = (double)info->width/2.0; //cx
    info->K[5] = (double)info->height/2.0; //cy
    info->K[0] = info->K[2]/tanhfov2; //fx
    info->K[4] = info->K[5]/tanvfov2; //fy 
    info->K[8] = 1.0;
    //Projection
    info->P[2] = info->K[2]; //cx'
    info->P[6] = info->K[5]; //cy'
    info->P[0] = info->K[0]; //fx';
    info->P[5] = info->K[4]; //fy';
    info->P[3] = 0.0; //Tx - position of second camera from stereo pair
    info->P[7] = 0.0; //Ty;
    info->P[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = cam->GetName();
	disp->width = img->width;
    disp->height = img->height;
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
	
    return std::make_tuple(img, info, disp);
}

std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr> ROSInterface::GenerateSegmentationCameraMsgPrototypes(SegmentationCamera* cam)
{
    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = cam->GetName();
	cam->GetResolution(img->width, img->height);
	img->encoding = "16UC1";
	img->is_bigendian = 0;
    img->step = img->width * sizeof(uint16_t);
    img->data.resize(img->step * img->height);

	//Camera info message
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
	info->header.frame_id = cam->GetName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->D.resize(5, 0.0);
    //Rectification (for stereo only)
    info->R[0] = 1.0;
    info->R[4] = 1.0;
    info->R[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->GetHorizontalFov()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->K[2] = (double)info->width/2.0; //cx
    info->K[5] = (double)info->height/2.0; //cy
    info->K[0] = info->K[2]/tanhfov2; //fx
    info->K[4] = info->K[5]/tanvfov2; //fy 
    info->K[8] = 1.0;
    //Projection
    info->P[2] = info->K[2]; //cx'
    info->P[6] = info->K[5]; //cy'
    info->P[0] = info->K[0]; //fx';
    info->P[5] = info->K[4]; //fy';
    info->P[3] = 0.0; //Tx - position of second camera from stereo pair
    info->P[7] = 0.0; //Ty;
    info->P[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = cam->GetName();
	disp->width = img->width;
    disp->height = img->height;
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
	
    return std::make_tuple(img, info, disp);
}

std::pair<std::string, unsigned int> getSonarOutputFormatInfo(SonarOutputFormat fmt)
{
    switch (fmt)
    {
        case SonarOutputFormat::U8:
            return std::make_pair("8UC1", 1);
        case SonarOutputFormat::U16:
            return std::make_pair("16UC1", 2);
        case SonarOutputFormat::F32:
            return std::make_pair("32FC1", 4);
        default:
            throw std::runtime_error("Unsupported sonar output format for ROS message generation");
            break;
    }
}

std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr> ROSInterface::GenerateFLSMsgPrototypes(FLS* fls)
{
    auto formatInfo = getSonarOutputFormatInfo(fls->GetOutputFormat());
    
    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = fls->GetName();
    fls->GetResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = fls->GetName();
	fls->GetDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr> ROSInterface::GenerateSSSMsgPrototypes(SSS* sss)
{
    auto formatInfo = getSonarOutputFormatInfo(sss->GetOutputFormat());

    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = sss->GetName();
    sss->GetResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = sss->GetName();
	sss->GetDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr> ROSInterface::GenerateMSISMsgPrototypes(MSIS* msis)
{
    auto formatInfo = getSonarOutputFormatInfo(msis->GetOutputFormat());

    //Image message
    sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>();
    img->header.frame_id = msis->GetName();
    msis->GetResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::ImagePtr disp = boost::make_shared<sensor_msgs::Image>();
    disp->header.frame_id = msis->GetName();
	msis->GetDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

}
