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
//  G500IAUVSimulationManager.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include "G500IAUVSimulationManager.h"
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Cylinder.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/entities/statics/Obstacle.h>
#include <Stonefish/entities/solids/Wing.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <Stonefish/utils/UnitSystem.h>
#include <Stonefish/core/NED.h>
#include "ROSInterface.h"

G500IAUVSimulationManager::G500IAUVSimulationManager(sf::Scalar stepsPerSecond) 
	: SimulationManager(stepsPerSecond, sf::SolverType::SOLVER_SI, sf::CollisionFilteringType::COLLISION_EXCLUSIVE, sf::FluidDynamicsType::GEOMETRY_BASED)
{
    std::string ns = cola2::rosutils::getNamespace();

	//Output from simulation
	gpsPub = nh.advertise<sensor_msgs::NavSatFix>(ns + "/navigator/gps", 2);
    pressurePub = nh.advertise<sensor_msgs::FluidPressure>(ns + "/navigator/pressure", 2);
    soundVelocityPub = nh.advertise<cola2_msgs::Float32Stamped>(ns + "/navigator/sound_velocity", 2);
    temperaturePub = nh.advertise<sensor_msgs::Temperature>(ns + "/valeport_minisvs_svs/temperature", 2);
    dvlPub = nh.advertise<cola2_msgs::DVL>(ns + "/navigator/dvl", 2);
    altitudePub = nh.advertise<sensor_msgs::Range>(ns + "/navigator/altitude", 2);
    imuPub = nh.advertise<sensor_msgs::Imu>(ns + "/navigator/imu", 2);
    odomPub = nh.advertise<nav_msgs::Odometry>(ns + "/dynamics/odometry", 2);
    ftPub = nh.advertise<geometry_msgs::WrenchStamped>(ns + "/ft_sensor/force_torque", 2);
    cameraPub = nh.advertise<sensor_msgs::Image>(ns + "/proscilica/image_color", 2);
    cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>(ns + "/proscilica/camera_info", 2);
    
    //Input to simulation
    thrustSub = nh.subscribe(ns + "/controller/thruster_setpoints", 1, &G500IAUVSimulationManager::ThrustCallback, this);
   	armSub = nh.subscribe(ns + "/eca_5emicro_manipulator/desired_joint_states", 1, &G500IAUVSimulationManager::ArmCallback, this);
    jointStatePub = nh.advertise<sensor_msgs::JointState>(ns + "/eca_5emicro_manipulator/joint_states", 2);
   
    //Diagnostics
    dvlDiag = new cola2::rosutils::DiagnosticHelper(nh, "dvl", "Simulated");
    gpsDiag = new cola2::rosutils::DiagnosticHelper(nh, "gps", "Simulated");
    imuDiag = new cola2::rosutils::DiagnosticHelper(nh, "imu", "Simulated");
    svsDiag = new cola2::rosutils::DiagnosticHelper(nh, "pressure", "Simulated");
    
    ft = NULL;
    filteredFT = std::vector<btScalar>(6, 0.0);
    filteringCoeff = 0.5;
    
    thrustSetpoints = std::vector<double>(5, 0.0);
	armSetpoints = std::vector<double>(4, 0.0);
	armCtrlVelocity = false;		
}

sf::Scalar G500IAUVSimulationManager::getFilteringCoeff()
{
	return filteringCoeff;	
}

void G500IAUVSimulationManager::setFilteringCoeff(sf::Scalar c)
{
	filteringCoeff = c;
}

void G500IAUVSimulationManager::DestroyScenario()
{
	SimulationManager::DestroyScenario();
	
	delete dvlDiag;
	delete gpsDiag;
	delete imuDiag;
	delete svsDiag;
}

void G500IAUVSimulationManager::BuildScenario()
{
	std::string ns = cola2::rosutils::getNamespace();

    ///////MATERIALS////////
    CreateMaterial("Neutral", 1000.0, 0.5);
    CreateMaterial("Rock", 3000.0, 0.1);
    CreateMaterial("Fiberglass", 1500.0, 0.3);
    CreateMaterial("Aluminium", 2710.0, 0.7);
    
    SetMaterialsInteraction("Neutral", "Neutral", 0.5, 0.2);
    SetMaterialsInteraction("Neutral", "Rock", 0.02, 0.01);
    SetMaterialsInteraction("Neutral", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Neutral", "Aluminium", 0.5, 0.2);
    
    SetMaterialsInteraction("Rock", "Rock", 0.9, 0.7);
    SetMaterialsInteraction("Rock", "Fiberglass", 0.6, 0.4);
    SetMaterialsInteraction("Rock", "Aluminium", 0.6, 0.3);

    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Fiberglass", "Aluminium", 0.5, 0.2);
    
    SetMaterialsInteraction("Aluminium", "Aluminium", 0.5, 0.2);
    
    ///////LOOKS///////////
    CreateLook("yellow", sf::Color::RGB(1.f, 0.9f, 0.f), 0.3f, 0.f);
    CreateLook("gray", sf::Color::Gray(0.3f), 0.4f, 0.5f);
    CreateLook("seabed", sf::Color::RGB(0.7f, 0.7f, 0.5f), 0.9f, 0.f);
    CreateLook("propeller", sf::Color::Gray(1.f), 0.3f, 0.f, 0.f, sf::GetDataPath() + "girona500/propeller_tex.png");
    CreateLook("duct", sf::Color::Gray(0.1f), 0.4f, 0.5f);
    CreateLook("manipulator", sf::Color::RGB(0.2f, 0.15f, 0.1f), 0.6f, 0.8f);
    CreateLook("link4", sf::Color::Gray(1.f), 0.6f, 0.8f, 0.f, sf::GetDataPath() + "eca5emicro/link4_tex.png");
    CreateLook("ee", sf::Color::RGB(0.59f, 0.56f, 0.51f), 0.6f, 0.8f);
    CreateLook("pipe", sf::Color::RGB(1.0f, 0.2f, 0.0f), 0.2f, 0.3f);
   
    ////////OBJECTS    
    //Create environment
    EnableOcean(0.0);

    sf::Plane* plane = new sf::Plane("Bottom", 10000.0, "Rock", "seabed");
    AddStaticEntity(plane, sf::Transform(sf::IQ(), sf::Vector3(0,0,5.0)));    
       
	//Create underwater vehicle body
    //Externals
    sf::Polyhedron* hullB = new sf::Polyhedron(ns + "/HullBottom", sf::GetDataPath() + "girona500/hull_phy.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* hullP = new sf::Polyhedron(ns + "/HullPort", sf::GetDataPath() + "girona500/hull_phy.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* hullS = new sf::Polyhedron(ns + "/HullStarboard", sf::GetDataPath() + "girona500/hull_phy.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* vBarStern = new sf::Polyhedron(ns + "/TBarStern", sf::GetDataPath() + "girona500/tbar_phy.obj", 1.0, sf::I4(), "Aluminium", sf::BodyPhysicsType::SUBMERGED_BODY, "grey", 0.003, false);
    sf::Polyhedron* vBarBow = new sf::Polyhedron(ns + "/TBarBow", sf::GetDataPath() + "girona500/tbar_phy.obj", 1.0, sf::I4(), "Aluminium", sf::BodyPhysicsType::SUBMERGED_BODY, "grey", 0.003, false);
    sf::Polyhedron* ductSway = new sf::Polyhedron(ns + "/DuctSway", sf::GetDataPath() + "girona500/duct_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductSurgeP = new sf::Polyhedron(ns + "/DuctSurgePort", sf::GetDataPath() + "girona500/duct_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductSurgeS = new sf::Polyhedron(ns + "/DuctSurgeStarboard", sf::GetDataPath() + "girona500/duct_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductHeaveS = new sf::Polyhedron(ns + "/DuctHeaveStern", sf::GetDataPath() + "girona500/duct_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductHeaveB = new sf::Polyhedron(ns + "/DuctHeaveBow", sf::GetDataPath() + "girona500/duct_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    //Internals
    sf::Cylinder* batteryCyl = new sf::Cylinder(ns + "/BatteryCylinder", 0.13, 0.6, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    batteryCyl->ScalePhysicalPropertiesToArbitraryMass(70);
    sf::Cylinder* portCyl = new sf::Cylinder(ns + "/PortCylinder", 0.13, 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    portCyl->ScalePhysicalPropertiesToArbitraryMass(20);
    sf::Cylinder* starboardCyl = new sf::Cylinder(ns + "/StarboardCylinder", 0.13, 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    starboardCyl->ScalePhysicalPropertiesToArbitraryMass(20);
    
    //Build whole body
    sf::Compound* vehicle = new sf::Compound(ns + "/Vehicle", hullB, sf::I4(), sf::BodyPhysicsType::SUBMERGED_BODY);
    vehicle->AddExternalPart(hullP, sf::Transform(sf::IQ(), sf::Vector3(0,-0.35,-0.7)));
    vehicle->AddExternalPart(hullS, sf::Transform(sf::IQ(), sf::Vector3(0,0.35,-0.7)));
    vehicle->AddExternalPart(vBarStern, sf::Transform(sf::IQ(), sf::Vector3(-0.25,0.0,-0.15)));
    vehicle->AddExternalPart(vBarBow, sf::Transform(sf::IQ(), sf::Vector3(0.30,0.0,-0.15)));
    vehicle->AddExternalPart(ductSway, sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    vehicle->AddExternalPart(ductSurgeP, sf::Transform(sf::Quaternion(0,0,M_PI), sf::Vector3(-0.2807,-0.2587,-0.38)));
    vehicle->AddExternalPart(ductSurgeS, sf::Transform(sf::IQ(), sf::Vector3(-0.2807,0.2587,-0.38)));
    vehicle->AddExternalPart(ductHeaveS, sf::Transform(sf::Quaternion(M_PI_2,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    vehicle->AddExternalPart(ductHeaveB, sf::Transform(sf::Quaternion(-M_PI_2,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
    vehicle->AddInternalPart(batteryCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(-0.05,0,0)));
    vehicle->AddInternalPart(portCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,-0.35,-0.7)));
    vehicle->AddInternalPart(starboardCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,0.35,-0.7)));
    
    //Create thrusters
    sf::Polyhedron* prop1 = new sf::Polyhedron(ns + "/Propeller1", sf::GetDataPath() + "girona500/propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop2 = new sf::Polyhedron(ns + "/Propeller2", sf::GetDataPath() + "girona500/propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop3 = new sf::Polyhedron(ns + "/Propeller3", sf::GetDataPath() + "girona500/propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop4 = new sf::Polyhedron(ns + "/Propeller4", sf::GetDataPath() + "girona500/propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop5 = new sf::Polyhedron(ns + "/Propeller5", sf::GetDataPath() + "girona500/propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Thruster* thSway = new sf::Thruster(ns + "/ThrusterSway", prop1, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thSurgeP = new sf::Thruster(ns + "/ThrusterSurgePort", prop2, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thSurgeS = new sf::Thruster(ns + "/ThrusterSurgeStarboard", prop3, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thHeaveS = new sf::Thruster(ns + "/ThrusterHeaveStern", prop4, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thHeaveB = new sf::Thruster(ns + "/ThrusterHeaveBow", prop5, 0.18, 0.48, 0.05, 1000.0, true);

    //Create sensors
    odom = new sf::Odometry(ns + "/dynamics", 30);

    pressure = new sf::Pressure(ns + "/pressure", 5);
    pressure->setNoise(5.0);
    
    dvl = new sf::DVL(ns + "/dvl", 30, 5);
    dvl->setRange(sf::Vector3(9.0, 9.0, 9.0), 0.3, 81.0);
    dvl->setNoise(0.0015, 0.001);
    
    imu = new sf::IMU(ns + "/imu_filter", 20);
    imu->setNoise(sf::UnitSystem::Angle(true, 0.0001), sf::UnitSystem::Angle(true, 0.001));
      
    sf::Scalar latitude, longitude;
    nh.getParam(ns + "/navigator/ned_latitude", latitude);
    nh.getParam(ns + "/navigator/ned_longitude", longitude);
    getNED()->Init(latitude, longitude, 0.0);
    gps = new sf::GPS(ns + "/gps", 1);
    gps->setNoise(0.5);

    cam = new sf::ColorCamera(ns + "/proscilica", 1360, 1024, 55.0, 1, 10);
    cam->InstallNewDataHandler(std::bind(&G500IAUVSimulationManager::CameraImageReady, this, std::placeholders::_1));
    cam->setDisplayOnScreen(false);
    
    //ROBOT
    iauv = new sf::Robot(ns);
    sf::Vector3 vehicleCenter(0.049, 0.0, -0.361);

    //Manipulator links
    sf::Polyhedron* baseLink = new sf::Polyhedron(ns + "/ECABaseLink", sf::GetDataPath() + "eca5emicro/base_link_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    sf::Polyhedron* link1 = new sf::Polyhedron(ns + "/ECALink1", sf::GetDataPath() + "eca5emicro/link1_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    sf::Polyhedron* link2 = new sf::Polyhedron(ns + "/ECALink2", sf::GetDataPath() + "eca5emicro/link2_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    sf::Polyhedron* link3 = new sf::Polyhedron(ns + "/ECALink3", sf::GetDataPath() + "eca5emicro/link3_phy.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    sf::Polyhedron* link4 = new sf::Polyhedron(ns + "/ECALink4", sf::GetDataPath() + "eca5emicro/link4_ft.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    sf::Polyhedron* ee = new sf::Polyhedron(ns + "/ECAEndEffector", sf::GetDataPath() + "eca5emicro/eeprobe.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");

    std::vector<sf::SolidEntity*> arm;
    arm.push_back(baseLink);
    arm.push_back(link1);
    arm.push_back(link2);
    arm.push_back(link3);
    arm.push_back(link4);
    arm.push_back(ee);

    //Mechanical structure
    iauv->DefineLinks(vehicle, arm);
    iauv->DefineFixedJoint("VehicleToArm", ns + "/Vehicle", ns + "/ECABaseLink", sf::Transform(sf::IQ(), vehicleCenter + sf::Vector3(0.74, -0.011, 0.372 + 0.0136))); //0.0136 --> offset between base link 0 and joint 1 axis
    iauv->DefineRevoluteJoint("joint1", ns + "/ECABaseLink", ns + "/ECALink1", sf::I4(), sf::Vector3(0.0, 0.0, 1.0), std::make_pair(-0.43, 1.54));
    iauv->DefineRevoluteJoint("joint2", ns + "/ECALink1", ns + "/ECALink2", sf::Transform(sf::IQ(), sf::Vector3(0.1065, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-0.1, 1.45));
    iauv->DefineRevoluteJoint("joint3", ns + "/ECALink2", ns + "/ECALink3", sf::Transform(sf::IQ(), sf::Vector3(0.23332, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-1.22, 0.645));
    iauv->DefineRevoluteJoint("joint4", ns + "/ECALink3", ns + "/ECALink4", sf::Transform(sf::IQ(), sf::Vector3(0.103, 0.0, 0.201)), sf::Vector3(0.0, 0.0, 1.0));
    iauv->DefineFixedJoint("ForceTorqueJoint", ns + "/ECALink4", ns + "/ECAEndEffector", sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.05)));
  
    //Drives
    sf::Servo* srv1 = new sf::Servo(ns + "/Servo1", 1.0, 1.0, 1000.0);
    sf::Servo* srv2 = new sf::Servo(ns + "/Servo2", 1.0, 1.0, 1000.0);
    sf::Servo* srv3 = new sf::Servo(ns + "/Servo3", 1.0, 1.0, 1000.0);
    sf::Servo* srv4 = new sf::Servo(ns + "/Servo4", 1.0, 1.0, 1000.0);
    iauv->AddJointActuator(srv1, "joint1");
    iauv->AddJointActuator(srv2, "joint2");
    iauv->AddJointActuator(srv3, "joint3");
    iauv->AddJointActuator(srv4, "joint4");

    //Sensors
    ft = new sf::ForceTorque(ns + "/force_torque", sf::I4(), 100);
    ft->setRange(sf::Vector3(2000.0, 2000.0, 2000.0), sf::Vector3(40.0, 40.0, 30.0));
    ft->setNoise(0.25, 0.005);
    iauv->AddJointSensor(ft, "ForceTorqueJoint");
    
    //Thrusters
    iauv->AddLinkActuator(thSway, ns + "/Vehicle", sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    iauv->AddLinkActuator(thSurgeP, ns + "/Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,-0.2587,-0.38)));
    iauv->AddLinkActuator(thSurgeS, ns + "/Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,0.2587,-0.38)));
    iauv->AddLinkActuator(thHeaveS, ns + "/Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    iauv->AddLinkActuator(thHeaveB, ns + "/Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
  
    //Sensors
    iauv->AddLinkSensor(odom, ns + "/Vehicle", sf::Transform(sf::IQ(), vehicleCenter));
    iauv->AddLinkSensor(pressure, ns + "/Vehicle", sf::Transform(sf::IQ(), vehicleCenter + sf::Vector3(-0.48,0.35,-0.4)));
    iauv->AddLinkSensor(dvl, ns + "/Vehicle", sf::Transform(sf::Quaternion(-M_PI/4.0,0,M_PI), vehicleCenter + sf::Vector3(-0.75,0.0,0.4)));
    iauv->AddLinkSensor(imu, ns + "/Vehicle", sf::Transform(sf::IQ(), vehicleCenter));
    iauv->AddLinkSensor(gps, ns + "/Vehicle", sf::Transform(sf::IQ(), vehicleCenter + sf::Vector3(-0.54, -0.35, -0.74)));
    iauv->AddVisionSensor(cam, ns + "/Vehicle", sf::Transform(sf::Quaternion(M_PI_2,0,0), vehicleCenter + sf::Vector3(0.5, -0.12, 0.5)));

    AddRobot(iauv, sf::Transform(sf::IQ(), sf::Vector3(0,0,1.0)));
}

void G500IAUVSimulationManager::SimulationStepCompleted(sf::Scalar timeStep)
{
	/////////////////////////////////////////SENSORS//////////////////////////////////////////////
    std::string ns = cola2::rosutils::getNamespace();
	sf::Transform robotFrame = odom->getSensorFrame();
	
	if(odom->isNewDataAvailable())
	{
		//Odometry from physics engine (ground truth)
		sf::ROSInterface::PublishOdometry(odomPub, odom);
		odom->MarkDataOld();
		
        //Manipulator joints
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";

        msg.name.resize(4);
        msg.position.resize(4);
        msg.velocity.resize(4);
        msg.effort.resize(4);

        sf::Servo* srv;

        for(unsigned int i=0; i<4; ++i)
        {
            srv = (sf::Servo*)iauv->getActuator(ns + "/Servo" + std::to_string(i+1));
            msg.name[i] = ns.substr(1) + "/eca_5emicro_manipulator/" + srv->getJointName();
            msg.position[i] = srv->getPosition();
            msg.velocity[i] = srv->getVelocity();
            msg.effort[i] = srv->getEffort();  
        }
       
        jointStatePub.publish(msg);
	}
	
	//IMU readings
	if(imu->isNewDataAvailable())
	{
		sf::ROSInterface::PublishIMU(imuPub, imu);
		imu->MarkDataOld();
		imuDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
	}
	
	//DVL readings
	if(dvl->isNewDataAvailable())
	{
        sf::ROSInterface::PublishDVL(dvlPub, altitudePub, dvl);
        dvl->MarkDataOld();
        dvlDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
	}
	
	//GPS readings
	if(gps->isNewDataAvailable())
	{
		sf::ROSInterface::PublishGPS(gpsPub, gps);
		gps->MarkDataOld();
        gpsDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
	}

	//SVS readings
	if(pressure->isNewDataAvailable())
	{
		sf::ROSInterface::PublishPressure(pressurePub, pressure);
        pressure->MarkDataOld();
		
        cola2_msgs::Float32Stamped svMsg;
        svMsg.header.stamp = ros::Time::now();
        svMsg.header.frame_id = pressure->getName();
        svMsg.data = 1500.0;
        soundVelocityPub.publish(svMsg);

        sensor_msgs::Temperature tempMsg;
        tempMsg.header = svMsg.header;
        tempMsg.temperature = 15.42;
        temperaturePub.publish(tempMsg);
		
		svsDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
	}

	//Force/Torque sensor readings with exponential filtering
	if(ft != NULL && ft->isNewDataAvailable())
	{
		sf::Sample s = ft->getLastSample();
		ft->MarkDataOld();
		
		std::vector<sf::Scalar> ftdata = s.getData();
		for(unsigned int i=0; i<ftdata.size(); ++i)
			filteredFT[i] = filteringCoeff * ftdata[i] + (btScalar(1) - filteringCoeff) * filteredFT[i];
				
		geometry_msgs::WrenchStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = ns + "/ft_sensor";
        msg.wrench.force.x = -filteredFT[0];
		msg.wrench.force.y = -filteredFT[1];
		msg.wrench.force.z = -filteredFT[2];
		msg.wrench.torque.x = -filteredFT[3];
		msg.wrench.torque.y = -filteredFT[4];
		msg.wrench.torque.z = -filteredFT[5];
		ftPub.publish(msg);
		
        sf::ROSInterface::PublishTF(br, robotFrame.inverse() * ft->getSensorFrame(), msg.header.stamp, ns + "/base_link", ns + "/" + ft->getName());
	}
	
	//Camera stream
	if(cam->isNewDataAvailable())
	{
		cam->MarkDataOld();
        sf::ROSInterface::PublishTF(br, robotFrame.inverse() * cam->getSensorFrame(), ros::Time::now(), ns + "/base_link", ns + "/" + cam->getName());
	}
    
	//////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
	//Thrusters
    sf::Thruster* th = (sf::Thruster*)iauv->getActuator(ns + "/ThrusterSurgePort");
    th->setSetpoint(thrustSetpoints[0]);

    th = (sf::Thruster*)iauv->getActuator(ns + "/ThrusterSurgeStarboard");
    th->setSetpoint(thrustSetpoints[1]);

    th = (sf::Thruster*)iauv->getActuator(ns + "/ThrusterHeaveBow");
    th->setSetpoint(thrustSetpoints[2]);

    th = (sf::Thruster*)iauv->getActuator(ns + "/ThrusterHeaveStern");
    th->setSetpoint(thrustSetpoints[3]);

    th = (sf::Thruster*)iauv->getActuator(ns + "/ThrusterSway");
    th->setSetpoint(thrustSetpoints[4]);

    //Manipulator
    sf::Servo* srv;
	if(armCtrlVelocity)
	{
		for(unsigned int i=0; i<armSetpoints.size(); ++i)
        {
	   	   srv = (sf::Servo*)iauv->getActuator(ns + "/Servo" + std::to_string(i+1));
           srv->setDesiredVelocity(armSetpoints[i]);
        }
	}
	else
	{
		for(unsigned int i=0; i<armSetpoints.size(); ++i)
        {
           srv = (sf::Servo*)iauv->getActuator(ns + "/Servo" + std::to_string(i+1));
           srv->setDesiredPosition(armSetpoints[i]);
        }
	}

	ros::spinOnce();
}

//Callbacks
void G500IAUVSimulationManager::ThrustCallback(const cola2_msgs::Setpoints& msg)
{
	if(msg.setpoints.size() != 5)
	{
		ROS_ERROR("Wrong number of thruster setpoints!");
		return;
	}
	
	for(unsigned int i=0; i<5; ++i)
		thrustSetpoints[i] = -msg.setpoints[i];
}

void G500IAUVSimulationManager::ArmCallback(const sensor_msgs::JointState& msg)
{
	unsigned int nJoints = 8;
	
	if(msg.position.size() >= nJoints)
	{
		armCtrlVelocity = false;
		for(unsigned int i=0; i<nJoints; ++i)
			armSetpoints[i] = msg.position[i];	
	}
	else if(msg.velocity.size() >= nJoints)
	{
		armCtrlVelocity = true;
		for(unsigned int i=0; i<nJoints; ++i)
			armSetpoints[i] = msg.velocity[i];
	}
	else if(msg.effort.size() >= nJoints)
	{
		ROS_ERROR("No voltage mode implemented in simulation!");
	}
	else
	{
		ROS_ERROR("Wrong number of manipulator setpoints!");
	}
}

void G500IAUVSimulationManager::CameraImageReady(sf::ColorCamera* c)
{
	sf::ROSInterface::PublishCamera(cameraPub, cameraInfoPub, c);
}