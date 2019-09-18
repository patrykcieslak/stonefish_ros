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
//  Copyright (c) 2017-2019 Patryk Cieslak. All rights reserved.
//

#include "ROSInterface.h"

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

void ROSInterface::PublishIMU(ros::Publisher& pub, IMU* imu)
{
    Sample s = imu->getLastSample();
    Vector3 rpy = Vector3(s.getValue(0), s.getValue(1), s.getValue(2));
    Quaternion quat(rpy.z(), rpy.y(), rpy.x());
        
    Scalar oVariance = imu->getSensorChannelDescription(0).stdDev;
    oVariance *= oVariance; //Variance is square of standard deviation
    Scalar vVariance = imu->getSensorChannelDescription(3).stdDev;
    vVariance *= vVariance; //Variance is square of standard deviation

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();    
    msg.header.frame_id = imu->getName();
    msg.orientation.x = quat.x();
    msg.orientation.y = quat.y();
    msg.orientation.z = quat.z();
    msg.orientation.w = quat.w();
    msg.orientation_covariance[0] = oVariance;
    msg.orientation_covariance[4] = oVariance;
    msg.orientation_covariance[8] = oVariance;
    msg.angular_velocity.x = s.getValue(3);
    msg.angular_velocity.y = s.getValue(4);
    msg.angular_velocity.z = s.getValue(5);
    msg.angular_velocity_covariance[0] = vVariance;
    msg.angular_velocity_covariance[4] = vVariance;
    msg.angular_velocity_covariance[8] = vVariance;
    pub.publish(msg);
}

void ROSInterface::PublishPressure(ros::Publisher& pub, Pressure* press)
{
    Sample s = press->getLastSample();
    sensor_msgs::FluidPressure msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = press->getName();
    msg.fluid_pressure = s.getValue(0);
    msg.variance = press->getSensorChannelDescription(0).stdDev;
    msg.variance *= msg.variance; //Variance is square of standard deviation
    pub.publish(msg);
}

void ROSInterface::PublishDVL(ros::Publisher& pub, ros::Publisher& altPub, DVL* dvl)
{
    Sample s = dvl->getLastSample();
    Scalar vVariance = dvl->getSensorChannelDescription(0).stdDev;
    vVariance *= vVariance; //Variance is square of standard deviation

    cola2_msgs::DVL msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = dvl->getName();
    msg.velocity.x = s.getValue(0);
    msg.velocity.y = s.getValue(1);
    msg.velocity.z = s.getValue(2);
    msg.velocity_covariance[0] = vVariance;
    msg.velocity_covariance[4] = vVariance;
    msg.velocity_covariance[8] = vVariance;
    msg.altitude = s.getValue(3);
    pub.publish(msg);

    sensor_msgs::Range msg2;
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id = dvl->getName() + "_altitude";
    msg2.radiation_type = msg2.ULTRASOUND;
    msg2.field_of_view = 0.2;
    msg2.min_range = 0.5;
    msg2.max_range = 80.0;
    msg2.range = s.getValue(3);
    altPub.publish(msg2);
}

void ROSInterface::PublishGPS(ros::Publisher& pub, GPS* gps)
{
    Sample s = gps->getLastSample();

    sensor_msgs::NavSatFix msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = gps->getName();
    msg.status.service = msg.status.SERVICE_GPS;

    if(s.getValue(1) < Scalar(0)) //Underwater
    {
        msg.status.status = msg.status.STATUS_NO_FIX;
        msg.latitude = 0.0;
        msg.longitude = 0.0;
        msg.altitude = 0.0;
    }
    else
    {
        msg.status.status = msg.status.STATUS_FIX;
        msg.latitude = s.getValue(0);
        msg.longitude = s.getValue(1);
        msg.altitude = 0.0;
    }

    msg.position_covariance[0] = msg.position_covariance[4] = gps->getNoise() * gps->getNoise();
    msg.position_covariance[8] = 1.0;
    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    pub.publish(msg);
}

void ROSInterface::PublishOdometry(ros::Publisher& pub, Odometry* odom)
{
    Sample s = odom->getLastSample();
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world_ned";
    msg.child_frame_id = odom->getName();
    msg.pose.pose.position.x = s.getValue(0);
    msg.pose.pose.position.y = s.getValue(1);
    msg.pose.pose.position.z = s.getValue(2);
    msg.twist.twist.linear.x = s.getValue(3);
    msg.twist.twist.linear.y = s.getValue(4);
    msg.twist.twist.linear.z = s.getValue(5);
    msg.pose.pose.orientation.x = s.getValue(6);
    msg.pose.pose.orientation.y = s.getValue(7);
    msg.pose.pose.orientation.z = s.getValue(8);
    msg.pose.pose.orientation.w = s.getValue(9);
    msg.twist.twist.angular.x = s.getValue(10);
    msg.twist.twist.angular.y = s.getValue(11);
    msg.twist.twist.angular.z = s.getValue(12);
    pub.publish(msg);
}

void ROSInterface::PublishForceTorque(ros::Publisher& pub, ForceTorque* ft)
{
    Sample s = ft->getLastSample();    
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = ft->getName();
    msg.wrench.force.x = -s.getValue(0);
    msg.wrench.force.y = -s.getValue(1);
    msg.wrench.force.z = -s.getValue(2);
    msg.wrench.torque.x = -s.getValue(3);
    msg.wrench.torque.y = -s.getValue(4);
    msg.wrench.torque.z = -s.getValue(5);
    pub.publish(msg);
}

void ROSInterface::PublishCamera(ros::Publisher& imagePub, ros::Publisher& cameraInfoPub, ColorCamera* cam)
{
	//Publish image message
    sensor_msgs::Image img;
    img.header.stamp = ros::Time::now();
    img.header.frame_id = cam->getName();
	cam->getResolution(img.width, img.height);
	img.encoding = "rgb8";
	img.is_bigendian = 0;
    img.step = img.width*3;
    img.data.resize(img.width*img.height*3);
    //Copy image data
    uint8_t* data = (uint8_t*)cam->getImageDataPointer();
    for(uint32_t r = 0; r<img.height; ++r) //Every row of image
    {
		uint8_t* srcRow = data + r*img.step; 
		uint8_t* dstRow = img.data.data() + (img.height-1-r) * img.step; 
		memcpy(dstRow, srcRow, img.step);
    }
    imagePub.publish(img);
	
	//Publish camera info message
	sensor_msgs::CameraInfo info;
	info.header.stamp = img.header.stamp;
	info.header.frame_id = cam->getName();
    info.width = img.width;
    info.height = img.height;
    info.binning_x = 0;
    info.binning_y = 0;
    //Distortion
    info.distortion_model = "plumb_bob";
    info.D.resize(5, 0.0);
    //Rectification (for stereo only)
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->getHorizontalFOV()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info.height/(double)info.width * tanhfov2;
    info.K[2] = (double)info.width/2.0; //cx
    info.K[5] = (double)info.height/2.0; //cy
    info.K[0] = info.K[2]/tanhfov2; //fx
    info.K[4] = info.K[5]/tanvfov2; //fy 
    info.K[8] = 1.0;
    //Projection
    info.P[2] = info.K[2]; //cx'
    info.P[6] = info.K[5]; //cy'
    info.P[0] = info.K[0]; //fx';
    info.P[5] = info.K[4]; //fy';
    info.P[3] = 0.0; //Tx - position of second camera from stereo pair
    info.P[7] = 0.0; //Ty;
    info.P[10] = 1.0;
    //ROI
    info.roi.x_offset = 0;
    info.roi.y_offset = 0;
    info.roi.height = info.height;
    info.roi.width = info.width;
    info.roi.do_rectify = false;
	cameraInfoPub.publish(info);
}

void ROSInterface::PublishPointCloud(ros::Publisher& pointCloudPub, DepthCamera* cam)
{
	uint32_t width, height, nPoints;
	cam->getResolution(width, height);
	nPoints = width*height;
	float tanhfov2 = tanf(cam->getHorizontalFOV()/180.f * M_PI/2.f);
    float tanvfov2 = tanhfov2 * (float)height/(float)width;
    glm::vec2 range = cam->getDepthRange();

    Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
    K(0,2) = (float)width/2.f; //cx
    K(1,2) = (float)height/2.f; //cy
    K(0,0) = K(0,2)/tanhfov2; //fx
    K(1,1) = K(1,2)/tanvfov2; //fy
    K(2,2) = 1.f;
    Eigen::Matrix3f Kinv = K.inverse();

    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = cam->getName();
    msg.height = 1;
    
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, 
    								 "y", 1, sensor_msgs::PointField::FLOAT32,
    								 "z", 1, sensor_msgs::PointField::FLOAT32);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(nPoints);

    sensor_msgs::PointCloud2Iterator<float> iterX(msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iterY(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(msg, "z");

    float* data = (float*)cam->getImageDataPointer();
    uint32_t nGoodPoints = 0;

    for(uint32_t i = 0; i<width; ++i)
    	for(uint32_t h = 0; h<height; ++h)
    	{
    		float depth = data[width*h + i];
    		if(depth < range.y) //If not at the detection boundary
    		{
				Eigen::Vector3f imagePoint(((float)i+0.5f)*depth, ((float)height-(float)h-1.f+0.5f)*depth, depth);
    			Eigen::Vector3f camPoint = Kinv*imagePoint; 
    			*iterX = camPoint.x();
    			*iterY = camPoint.y();
    			*iterZ = camPoint.z();
    			++iterX;
    			++iterY;
    			++iterZ;
    			++nGoodPoints;
    		}
    	}

    msg.width = nGoodPoints;
    modifier.resize(nGoodPoints);

    pointCloudPub.publish(msg);
}

}