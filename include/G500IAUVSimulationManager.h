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
//  G500IAUVSimulationManager.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#ifndef __G500IAUVSimulationManager__
#define __G500IAUVSimulationManager__

//Stonefish
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
//ROS sim nav sensors
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <cola2_msgs/DVL.h>
#include <cola2_msgs/Float32Stamped.h>
#include <tf/transform_broadcaster.h>
//ROS sim actuators
#include <cola2_msgs/Setpoints.h>
#include <cola2_lib/rosutils/diagnostic_helper.h>

class G500IAUVSimulationManager : public sf::SimulationManager
{
public:
    G500IAUVSimulationManager(sf::Scalar stepsPerSecond);
    
    //Overriden methods
    void BuildScenario();
    void SimulationStepCompleted(sf::Scalar timeStep);
    void DestroyScenario();
    
    //ROS callbacks & services
    void ThrustCallback(const cola2_msgs::Setpoints& msg);
    void ArmCallback(const sensor_msgs::JointState& msg);
    
    //Others
    void CameraImageReady(sf::ColorCamera* c);
    sf::Scalar getFilteringCoeff();
    void setFilteringCoeff(sf::Scalar c);
    
private:
	//Stonefish
	sf::Robot* iauv;
	sf::Odometry* odom;
	sf::Pressure* pressure;
	sf::DVL* dvl;
	sf::IMU* imu;
	sf::GPS* gps;
	sf::ForceTorque* ft;
	sf::ColorCamera* cam;
	sf::DepthCamera* dcam;
	bool armCtrlVelocity;
	sf::Vector3 rpy;
	std::vector<double> thrustSetpoints;
	std::vector<double> armSetpoints;
	std::vector<sf::Scalar> filteredFT;
	sf::Scalar filteringCoeff;
	
	//ROS
	ros::NodeHandle nh;
	
	//Sensors
	ros::Publisher odomPub;
	ros::Publisher imuPub;
	ros::Publisher pressurePub;
	ros::Publisher soundVelocityPub;
	ros::Publisher temperaturePub;
	ros::Publisher dvlPub;
	ros::Publisher altitudePub;
	ros::Publisher gpsPub;
	ros::Publisher ftPub;
	ros::Publisher jointStatePub;
	ros::Publisher cameraPub;
	ros::Publisher cameraInfoPub;
	cola2::rosutils::DiagnosticHelper* imuDiag;
	cola2::rosutils::DiagnosticHelper* dvlDiag;
	cola2::rosutils::DiagnosticHelper* gpsDiag;
	cola2::rosutils::DiagnosticHelper* svsDiag;
	tf::TransformBroadcaster br;

	//Actuators
	ros::Subscriber thrustSub;
	ros::Subscriber armSub;
};

#endif
