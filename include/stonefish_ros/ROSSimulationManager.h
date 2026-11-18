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
//  ROSSimulationManager.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 17/09/19.
//  Copyright (c) 2019-2026 Patryk Cieslak. All rights reserved.
//

#pragma once

//Stonefish
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/Servo.h>
//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <stonefish_ros/SonarSettings.h>
#include <stonefish_ros/SonarSettings2.h>
#include <stonefish_ros/Respawn.h>
#include <image_transport/image_transport.h>
#include "stonefish_ros/ROSControlInterface.h"
#include "stonefish_ros/DebugPhysics.h"

namespace sf
{
	class ColorCamera;
	class DepthCamera;
	class ThermalCamera;
	class OpticalFlowCamera;
	class SegmentationCamera;
	class EventBasedCamera;
	class Lidar;
	class FLS;
	class SSS;
	class MSIS;
	class SuctionCup;
	class Light;
	class FixedJoint;
	class ManualTrajectory;
	class Uniform;
	class Jet;
	class Comm;

	struct ROSRobot
	{
		Robot* robot;
		bool publishBaseLinkTransform;
		std::vector<Scalar> thrusterSetpoints;
		std::vector<Scalar> propellerSetpoints;
		std::vector<Scalar> rudderSetpoints;
		std::map<std::string, std::pair<ServoControlMode, Scalar>> servoSetpoints;
		bool thrusterSetpointsChanged;
		bool propellerSetpointsChanged;
		bool rudderSetpointsChanged;
		bool respawnRequested;
		Transform respawnOrigin;

		ROSRobot(Robot* robot, unsigned int nThrusters, unsigned int nPropellers, unsigned int nRudders=0)
			: robot(robot), publishBaseLinkTransform(false), thrusterSetpointsChanged(false),
			  propellerSetpointsChanged(false), rudderSetpointsChanged(false), respawnRequested(false)
		{
			thrusterSetpoints = std::vector<Scalar>(nThrusters, Scalar(0));
			propellerSetpoints = std::vector<Scalar>(nPropellers, Scalar(0));
			rudderSetpoints = std::vector<Scalar>(nRudders, Scalar(0));
		}
	};

	// A class representing a standard simulation manager with ROS support.
	class ROSSimulationManager : public SimulationManager
	{
	public:
		ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath);
	    virtual ~ROSSimulationManager() = default;

		virtual void BuildScenario();
		virtual void DestroyScenario();

	    void AddROSRobot(std::unique_ptr<ROSRobot> robot);
	    bool RespawnROSRobot(std::string_view robotName, const Transform& origin);

		virtual void SimulationStepCompleted(Scalar timeStep);
	    virtual void ColorCameraImageReady(ColorCamera* cam);
	    virtual void DepthCameraImageReady(DepthCamera* cam);
		virtual void ThermalCameraImageReady(ThermalCamera* cam);
		virtual void OpticalFlowCameraImageReady(OpticalFlowCamera* cam);
		virtual void SegmentationCameraImageReady(SegmentationCamera* cam);
		virtual void EventBasedCameraOutputReady(EventBasedCamera* cam);
		virtual void LidarScanReady(Lidar* lidar);
		virtual void FLSScanReady(FLS* fls);
		virtual void SSSScanReady(SSS* sss);
		virtual void MSISScanReady(MSIS* sss);

	    bool EnableCurrents(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool DisableCurrents(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
		bool RespawnRobot(stonefish_ros::Respawn::Request& req, stonefish_ros::Respawn::Response& res);
        
		virtual uint64_t GetSimulationClock() const;
		virtual void SimulationClockSleep(uint64_t us);
	    ros::NodeHandle& GetNodeHandle();
	    image_transport::ImageTransport& GetImageTransportHandle();
	    std::unordered_map<std::string, ros::ServiceServer>& GetServiceServers();
		std::unordered_map<std::string, ros::Publisher>& GetPublishers();
		std::unordered_map<std::string, image_transport::Publisher>& GetImagePublishers();
	    std::unordered_map<std::string, ros::Subscriber>& GetSubscribers();
		std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& GetCameraMsgPrototypes();
		std::unordered_map<std::string, std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr>>& GetDualImageCameraMsgPrototypes();
		std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& GetSonarMsgPrototypes();
		std::vector<std::unique_ptr<ROSControlInterface>>& GetControlInterfaces();

	protected:
		std::string scnFilePath_;
		std::vector<std::unique_ptr<ROSRobot>> rosRobots_;
		std::vector<std::unique_ptr<ROSControlInterface>> controlIfs_;
		ros::NodeHandle nh_;
		ros::AsyncSpinner spinner_;
		image_transport::ImageTransport it_;
		tf::TransformBroadcaster br_;
		ros::ServiceServer srvECurrents_, srvDCurrents_, srvRespawn_;
		std::unordered_map<std::string, ros::ServiceServer> srvs_;
		std::unordered_map<std::string, ros::Publisher> pubs_;
		std::unordered_map<std::string, image_transport::Publisher> imgPubs_;
		std::unordered_map<std::string, ros::Subscriber> subs_;
		std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>> cameraMsgPrototypes_;
		std::unordered_map<std::string, std::tuple<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr, sensor_msgs::ImagePtr>> dualImageCameraMsgPrototypes_;
		std::unordered_map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>> sonarMsgPrototypes_;
	};

	//Callback functors
	class UniformVFCallback
	{
	public:
		UniformVFCallback(Uniform* vf);
		void operator()(const geometry_msgs::Vector3ConstPtr& msg);

	private:
		Uniform* vf_;
	};

	class JetVFCallback
	{
	public:
		JetVFCallback(Jet* vf);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		Jet* vf_;
	};

	class ThrustersCallback
	{
	public:
		ThrustersCallback(ROSRobot* robot);
		void operator()(const std_msgs::Float64MultiArrayConstPtr& msg);

	private:
		ROSRobot* robot_;
	};

	class PropellersCallback
	{
	public:
		PropellersCallback(ROSRobot* robot);
		void operator()(const std_msgs::Float64MultiArrayConstPtr& msg);

	private:
		ROSRobot* robot_;
	};

	class RuddersCallback
	{
	public:
		RuddersCallback(ROSRobot* robot);
		void operator()(const std_msgs::Float64MultiArrayConstPtr& msg);

	private:
		ROSRobot* robot_;
	};

	class ServosCallback
	{
	public:
		ServosCallback(ROSRobot* robot);
		void operator()(const sensor_msgs::JointStateConstPtr& msg);

	private:
		ROSRobot* robot_;
	};

	class JointGroupCallback
	{
	public:
		JointGroupCallback(ROSRobot* robot,
						   ServoControlMode mode, const std::vector<std::string>& jointNames);
		void operator()(const std_msgs::Float64MultiArrayConstPtr& msg);

	private:
		ROSRobot* robot_;
		ServoControlMode mode_;
		std::vector<std::string> jointNames_;
	};

	class JointCallback
	{
	public:
		JointCallback(ROSRobot* robot, ServoControlMode mode, const std::string& jointName);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		ROSRobot* robot_;
		ServoControlMode mode_;
		std::string jointName_;
	};

	class PushCallback
	{
	public:
		PushCallback(Push* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		Push* act_;
	};

	class SimpleThrusterCallback
	{
	public:
		SimpleThrusterCallback(SimpleThruster* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		SimpleThruster* act_;
	};

	class ThrusterCallback
	{
	public:
		ThrusterCallback(Thruster* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		Thruster* act_;
	};

	class PropellerCallback
	{
	public:
		PropellerCallback(Propeller* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		Propeller* act_;
	};

	class VBSCallback
	{
	public:
		VBSCallback(VariableBuoyancy* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		VariableBuoyancy* act_;
	};

	class TrajectoryCallback
	{
	public:
		TrajectoryCallback(ManualTrajectory* tr);
		void operator()(const nav_msgs::OdometryConstPtr& msg);

	private:
		ManualTrajectory* tr_;
	};

	class ActuatorOriginCallback
	{
	public:
		ActuatorOriginCallback(Actuator* act);
		void operator()(const geometry_msgs::TransformConstPtr& msg);

	private:
		Actuator* act_;
	};

	class SensorOriginCallback
	{
	public:
		SensorOriginCallback(Sensor* sens);
		void operator()(const geometry_msgs::TransformConstPtr& msg);

	private:
		Sensor* sens_;
	};

	class CommCallback
	{
	public:
		CommCallback(Comm* comm);
		void operator()(const std_msgs::StringConstPtr& msg);

	private:
		Comm* comm_;
	};

	class SensorService
	{
	public:
		SensorService(Sensor* sens);
		bool operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

	private:
		Sensor* sens_;
	};

	class FLSService
	{
	public:
		FLSService(FLS* fls);
		bool operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res);

	private:
		FLS* fls_;
	};

	class SSSService
	{
	public:
		SSSService(SSS* sss);
		bool operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res);

	private:
		SSS* sss_;
	};

	class MSISService
	{
	public:
		MSISService(MSIS* msis);
		bool operator()(stonefish_ros::SonarSettings2::Request& req, stonefish_ros::SonarSettings2::Response& res);

	private:
		MSIS* msis_;
	};

	class SuctionCupService
	{
	public:
		SuctionCupService(SuctionCup* suction);
		bool operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

	private:
		SuctionCup* suction_;
	};

	class LightService
	{
	public:
		LightService(Light* l);
		bool operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

	private:
		Light* l_;
	};

	class GlueService
	{
	public:
		GlueService(ROSSimulationManager* sm, FixedJoint* fix);
		bool operator()(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
	
	private:
		ROSSimulationManager* sm_;
		FixedJoint* fix_;
	};
}

