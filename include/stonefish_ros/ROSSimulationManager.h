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
//  Copyright (c) 2019-2021 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROSSimulationManager__
#define __Stonefish_ROSSimulationManager__

//Stonefish
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <cola2_msgs/Setpoints.h>
#include <std_srvs/Trigger.h>
#include <stonefish_ros/SonarSettings.h>
#include <stonefish_ros/SonarSettings2.h>

namespace sf
{
	class ColorCamera;
	class DepthCamera;
	class Multibeam2;
	class FLS;
	class SSS;
	class MSIS;
	class ManualTrajectory;
	class Uniform;
	class Jet;

	struct ROSRobot
	{
		Robot* robot;
		bool servoVelocityMode;
		bool publishBaseLinkTransform;
		std::vector<Scalar> thrusterSetpoints;
		std::vector<Scalar> propellerSetpoints;
		std::vector<Scalar> rudderSetpoints;
		std::map<std::string, Scalar> servoSetpoints;

		ROSRobot(Robot* robot, unsigned int nThrusters, unsigned int nPropellers, unsigned int nRudders=0) 
			: robot(robot), servoVelocityMode(true), publishBaseLinkTransform(false)
		{
			thrusterSetpoints = std::vector<Scalar>(nThrusters, Scalar(0));
			propellerSetpoints = std::vector<Scalar>(nPropellers, Scalar(0));
			rudderSetpoints = std::vector<Scalar>(nRudders, Scalar(0));
		} 
	};

	// A class....
	class ROSSimulationManager : public SimulationManager
	{
	public:
		ROSSimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath);
	    virtual ~ROSSimulationManager();

		virtual void BuildScenario();

	    void AddROSRobot(ROSRobot* robot);

		virtual void SimulationStepCompleted(Scalar timeStep);		
	    virtual void ColorCameraImageReady(ColorCamera* cam);
	    virtual void DepthCameraImageReady(DepthCamera* cam);
		virtual void Multibeam2ScanReady(Multibeam2* mb);
		virtual void FLSScanReady(FLS* fls);
		virtual void SSSScanReady(SSS* sss);
		virtual void MSISScanReady(MSIS* sss);

	    bool EnableCurrents(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
		bool DisableCurrents(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

		virtual uint64_t getSimulationClock();
		virtual void SimulationClockSleep(uint64_t us);
	    ros::NodeHandle& getNodeHandle();
	    std::map<std::string, ros::ServiceServer>& getServiceServers();
		std::map<std::string, ros::Publisher>& getPublishers();
	    std::map<std::string, ros::Subscriber>& getSubscribers();
		std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>>& getCameraMsgPrototypes();
		std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>>& getSonarMsgPrototypes();

	protected:
		std::string scnFilePath;
		ros::NodeHandle nh;
		tf::TransformBroadcaster br;
		ros::ServiceServer srvECurrents, srvDCurrents;
		std::map<std::string, ros::ServiceServer> srvs;
		std::map<std::string, ros::Publisher> pubs;
		std::map<std::string, ros::Subscriber> subs;
		std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>> cameraMsgPrototypes;
		std::map<std::string, std::pair<sensor_msgs::ImagePtr, sensor_msgs::ImagePtr>> sonarMsgPrototypes;
		std::vector<ROSRobot*> rosRobots;
	};

	//Callback functors
	class UniformVFCallback
	{
	public:
		UniformVFCallback(Uniform* vf);
		void operator()(const geometry_msgs::Vector3ConstPtr& msg);

	private:
		Uniform* vf;
	};

	class JetVFCallback
	{
	public:
		JetVFCallback(Jet* vf);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		Jet* vf;
	};

	class ThrustersCallback
	{
	public: 
		ThrustersCallback(ROSSimulationManager* sm, ROSRobot* robot);
		void operator()(const cola2_msgs::SetpointsConstPtr& msg);

	private:
		ROSSimulationManager* sm;
		ROSRobot* robot;
	};

	class PropellersCallback
	{
	public: 
		PropellersCallback(ROSSimulationManager* sm, ROSRobot* robot);
		void operator()(const cola2_msgs::SetpointsConstPtr& msg);

	private:
		ROSSimulationManager* sm;
		ROSRobot* robot;
	};

	class RuddersCallback
	{
	public: 
		RuddersCallback(ROSSimulationManager* sm, ROSRobot* robot);
		void operator()(const cola2_msgs::SetpointsConstPtr& msg);

	private:
		ROSSimulationManager* sm;
		ROSRobot* robot;
	};

	class ServosCallback
	{
	public: 
		ServosCallback(ROSSimulationManager* sm, ROSRobot* robot);
		void operator()(const sensor_msgs::JointStateConstPtr& msg);

	private:
		ROSSimulationManager* sm;
		ROSRobot* robot;
	};

	class VBSCallback
	{
	public: 
		VBSCallback(VariableBuoyancy* act);
		void operator()(const std_msgs::Float64ConstPtr& msg);

	private:
		VariableBuoyancy* act;
	};

	class TrajectoryCallback
	{
	public:
		TrajectoryCallback(ManualTrajectory* tr);
		void operator()(const nav_msgs::OdometryConstPtr& msg);

	private:
		ManualTrajectory* tr;
	};

	class ActuatorOriginCallback
	{
	public:
		ActuatorOriginCallback(Actuator* act);
		void operator()(const geometry_msgs::TransformConstPtr& msg);

	private:
		Actuator* act;
	};

	class SensorOriginCallback
	{
	public:
		SensorOriginCallback(Sensor* sens);
		void operator()(const geometry_msgs::TransformConstPtr& msg);

	private:
		Sensor* sens;
	};

	class FLSService
	{
	public:
		FLSService(FLS* fls);
		bool operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res);
	
	private:
		FLS* fls;
	};

	class SSSService
	{
	public:
		SSSService(SSS* sss);
		bool operator()(stonefish_ros::SonarSettings::Request& req, stonefish_ros::SonarSettings::Response& res);
	
	private:
		SSS* sss;
	};

	class MSISService
	{
	public:
		MSISService(MSIS* msis);
		bool operator()(stonefish_ros::SonarSettings2::Request& req, stonefish_ros::SonarSettings2::Response& res);
	
	private:
		MSIS* msis;
	};
}

#endif
