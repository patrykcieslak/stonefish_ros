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
//  G500AUVSimulationManager.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include "G500AUVSimulationManager.h"
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/entities/statics/Obstacle.h>
#include <Stonefish/entities/solids/Polyhedron.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/entities/solids/Cylinder.h>
#include <Stonefish/entities/solids/Compound.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <Stonefish/utils/UnitSystem.h>
#include <Stonefish/core/NED.h>
#include "StonefishROSInterface.hpp"

G500AUVSimulationManager::G500AUVSimulationManager(sf::Scalar stepsPerSecond) 
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
    cameraPub = nh.advertise<sensor_msgs::Image>(ns + "/proscilica/image_color", 2);
    cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>(ns + "/proscilica/camera_info", 2);
    
    //Input to simulation
    thrustSub = nh.subscribe(ns + "/controller/thruster_setpoints", 1, &G500AUVSimulationManager::ThrustCallback, this);
    
    //Diagnostics
    dvlDiag = new cola2::rosutils::DiagnosticHelper(nh, "dvl", "Simulated");
    gpsDiag = new cola2::rosutils::DiagnosticHelper(nh, "gps", "Simulated");
    imuDiag = new cola2::rosutils::DiagnosticHelper(nh, "imu", "Simulated");
    svsDiag = new cola2::rosutils::DiagnosticHelper(nh, "pressure", "Simulated");
    
    thrustSetpoints = std::vector<double>(5, 0.0);		
}

void G500AUVSimulationManager::DestroyScenario()
{
	SimulationManager::DestroyScenario();
	
	delete dvlDiag;
	delete gpsDiag;
	delete imuDiag;
	delete svsDiag;
}

void G500AUVSimulationManager::BuildScenario()
{
    std::string ns = cola2::rosutils::getNamespace();
    
    ///////MATERIALS////////
    CreateMaterial("Dummy", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.0), 0.5);
    CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.1);
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.3);
    CreateMaterial("Aluminium", sf::UnitSystem::Density(sf::CGS, sf::MKS, 2.71), 0.7);
    
    SetMaterialsInteraction("Dummy", "Dummy", 0.5, 0.2);
    SetMaterialsInteraction("Dummy", "Rock", 0.02, 0.01); //0.25, 0.1
    SetMaterialsInteraction("Dummy", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Dummy", "Aluminium", 0.5, 0.2);
    
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
    CreateLook("propeller", sf::Color::Gray(1.f), 0.3f, 0.f, 0.f, sf::GetDataPath() + "propeller_tex.png");
    CreateLook("duct", sf::Color::Gray(0.1f), 0.4f, 0.5f);
    CreateLook("manipulator", sf::Color::RGB(0.2f, 0.15f, 0.1f), 0.6f, 0.8f);
    CreateLook("link4", sf::Color::Gray(1.f), 0.6f, 0.8f, 0.f, sf::GetDataPath() + "link4_tex.png");
    CreateLook("ee", sf::Color::RGB(0.59f, 0.56f, 0.51f), 0.6f, 0.8f);
    CreateLook("pipe", sf::Color::RGB(1.0f, 0.2f, 0.0f), 0.2f, 0.3f);
    
    ////////OBJECTS    
    EnableOcean(0.0);

    //Create environment
    sf::Plane* plane = new sf::Plane("Bottom", 10000.0, "Rock", "seabed");
    AddStaticEntity(plane, sf::Transform(sf::Quaternion(0, 0, 0), sf::Vector3(0,0,5.0)));

    sf::Obstacle* pipe = new sf::Obstacle("HorizontalPipe", 0.25, 4.0, "Fiberglass", "pipe");
    AddStaticEntity(pipe, sf::Transform(sf::Quaternion(0, 0, M_PI_2), sf::Vector3(0.0, 0.0, 3.5)));  
     
	//Create underwater vehicle body
    //Externals
    sf::Polyhedron* hullB = new sf::Polyhedron("HullBottom", sf::GetDataPath() + "hull_hydro.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* hullP = new sf::Polyhedron("HullPort", sf::GetDataPath() + "hull_hydro.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* hullS = new sf::Polyhedron("HullStarboard", sf::GetDataPath() + "hull_hydro.obj", 1.0, sf::I4(), "Fiberglass", sf::BodyPhysicsType::SUBMERGED_BODY, "yellow", 0.003, false);
    sf::Polyhedron* vBarStern = new sf::Polyhedron("VBarStern", sf::GetDataPath() + "vbar_hydro.obj", 1.0, sf::I4(), "Aluminium", sf::BodyPhysicsType::SUBMERGED_BODY, "grey", 0.003, false);
    sf::Polyhedron* vBarBow = new sf::Polyhedron("VBarBow", sf::GetDataPath() + "vbar_hydro.obj", 1.0, sf::I4(), "Aluminium", sf::BodyPhysicsType::SUBMERGED_BODY, "grey", 0.003, false);
    sf::Polyhedron* ductSway = new sf::Polyhedron("DuctSway", sf::GetDataPath() + "duct_hydro.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductSurgeP = new sf::Polyhedron("DuctSurgePort", sf::GetDataPath() + "duct_hydro.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductSurgeS = new sf::Polyhedron("DuctSurgeStarboard", sf::GetDataPath() + "duct_hydro.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductHeaveS = new sf::Polyhedron("DuctHeaveStern", sf::GetDataPath() + "duct_hydro.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    sf::Polyhedron* ductHeaveB = new sf::Polyhedron("DuctHeaveBow", sf::GetDataPath() + "duct_hydro.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "duct");
    //Internals
    sf::Cylinder* batteryCyl = new sf::Cylinder("BatteryCylinder", 0.13, 0.6, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    batteryCyl->ScalePhysicalPropertiesToArbitraryMass(70);
    sf::Cylinder* portCyl = new sf::Cylinder("PortCylinder", 0.13, 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    portCyl->ScalePhysicalPropertiesToArbitraryMass(20);
    sf::Cylinder* starboardCyl = new sf::Cylinder("StarboardCylinder", 0.13, 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "manipulator");
    starboardCyl->ScalePhysicalPropertiesToArbitraryMass(20);
    
    //Build whole body
    sf::Compound* vehicle = new sf::Compound("Vehicle", hullB, sf::I4(), sf::BodyPhysicsType::SUBMERGED_BODY);
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
    sf::Polyhedron* prop1 = new sf::Polyhedron("Propeller1", sf::GetDataPath() + "propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop2 = new sf::Polyhedron("Propeller2", sf::GetDataPath() + "propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop3 = new sf::Polyhedron("Propeller3", sf::GetDataPath() + "propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop4 = new sf::Polyhedron("Propeller4", sf::GetDataPath() + "propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Polyhedron* prop5 = new sf::Polyhedron("Propeller5", sf::GetDataPath() + "propeller.obj", 1.0, sf::I4(), "Neutral", sf::BodyPhysicsType::SUBMERGED_BODY, "propeller");
    sf::Thruster* thSway = new sf::Thruster("ThrusterSway", prop1, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thSurgeP = new sf::Thruster("ThrusterSurgePort", prop2, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thSurgeS = new sf::Thruster("ThrusterSurgeStarboard", prop3, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thHeaveS = new sf::Thruster("ThrusterHeaveStern", prop4, 0.18, 0.48, 0.05, 1000.0, true);
    sf::Thruster* thHeaveB = new sf::Thruster("ThrusterHeaveBow", prop5, 0.18, 0.48, 0.05, 1000.0, true);

    //Create sensors
    odom = new sf::Odometry("dynamics", 30);

    pressure = new sf::Pressure("pressure", 5);
    pressure->setNoise(5.0);
    
    dvl = new sf::DVL("dvl", 30, 5);
    dvl->setRange(sf::Vector3(9.0, 9.0, 9.0), 0.5, 81.0);
    dvl->setNoise(0.0015, 0.001);
    
    imu = new sf::IMU("imu_filter", 20);
    imu->setNoise(sf::UnitSystem::Angle(true, 0.0001), sf::UnitSystem::Angle(true, 0.001));
     
    sf::Scalar latitude, longitude;
    nh.getParam(ns + "/navigator/ned_latitude", latitude);
    nh.getParam(ns + "/navigator/ned_longitude", longitude);
    getNED()->Init(latitude, longitude, 0.0);
    gps = new sf::GPS("gps", 1);
    gps->setNoise(0.5);

    cam = new sf::ColorCamera("proscilica", 1360, 1024, 55.0, 1, 10);
    cam->InstallNewDataHandler(std::bind(&G500AUVSimulationManager::CameraImageReady, this, std::placeholders::_1));
    cam->setDisplayOnScreen(false);

    /////////////////////// ROBOT ////////////////////////
    auv = new sf::Robot("GIRONA500");
    
    //Mechanical structure
    auv->DefineLinks(vehicle);
    
    //Thrusters
    auv->AddLinkActuator(thSway, "Vehicle", sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    auv->AddLinkActuator(thSurgeP, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,-0.2587,-0.38)));
    auv->AddLinkActuator(thSurgeS, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,0.2587,-0.38)));
    auv->AddLinkActuator(thHeaveS, "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    auv->AddLinkActuator(thHeaveB, "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
  
    //Sensors
    sf::Vector3 vehicleCenter(0.049, 0.0, -0.361);
    sf::Vector3 ladybugCenter(0,0,2);

    auv->AddLinkSensor(odom, "Vehicle", sf::Transform(sf::IQ(), vehicleCenter));
    auv->AddLinkSensor(pressure, "Vehicle", sf::Transform(sf::IQ(), vehicleCenter + sf::Vector3(-0.48,0.35,-0.4)));
    auv->AddLinkSensor(dvl, "Vehicle", sf::Transform(sf::Quaternion(-M_PI/4.0,0,M_PI), vehicleCenter + sf::Vector3(-0.75,0.0,0.4)));
    auv->AddLinkSensor(imu, "Vehicle", sf::Transform(sf::IQ(), vehicleCenter));
    auv->AddLinkSensor(gps, "Vehicle", sf::Transform(sf::IQ(), vehicleCenter + sf::Vector3(0,0,-0.7)));
    auv->AddVisionSensor(cam, "Vehicle", sf::Transform(sf::Quaternion(M_PI_2,0,0), vehicleCenter + sf::Vector3(0.5, -0.12, 0.5)));
    
    AddRobot(auv, sf::Transform(sf::IQ(), sf::Vector3(0,0,1)));
}

void G500AUVSimulationManager::SimulationStepCompleted()
{
	/////////////////////////////////////////SENSORS//////////////////////////////////////////////
	std::string ns = cola2::rosutils::getNamespace();
    sf::Transform robotFrame = odom->getSensorFrame();

    //Ground truth
    if(odom->isNewDataAvailable())
    {
        publishOdometry(odomPub, odom);
        odom->MarkDataOld();
    }
    
	//IMU readings
    if(imu->isNewDataAvailable())
    {
        publishIMU(imuPub, imu);
        imu->MarkDataOld();
        imuDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
    }
    
    //DVL readings
    if(dvl->isNewDataAvailable())
    {
        sf::Sample s = dvl->getLastSample();
        publishDVL(dvlPub, dvl);
        dvl->MarkDataOld();
    
        sensor_msgs::Range msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = ns + "/" + dvl->getName() + "_altitude";
        msg.radiation_type = msg.ULTRASOUND;
        msg.field_of_view = 0.2;
        msg.min_range = 0.5;
        msg.max_range = 80.0;
        msg.range = s.getValue(3);
        altitudePub.publish(msg);
        
        dvlDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
    }
    
    //GPS readings
    if(gps->isNewDataAvailable())
    {
        publishGPS(gpsPub, gps);
        gps->MarkDataOld();
        gpsDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
    }

	//SVS readings
    if(pressure->isNewDataAvailable())
    {
        publishPressure(pressurePub, pressure);
        pressure->MarkDataOld();
        
        cola2_msgs::Float32Stamped svMsg;
        svMsg.header.stamp = ros::Time::now();
        svMsg.header.frame_id = ns + "/" + pressure->getName();
        svMsg.data = 1500.0;
        soundVelocityPub.publish(svMsg);

        sensor_msgs::Temperature tempMsg;
        tempMsg.header = svMsg.header;
        tempMsg.temperature = 15.42;
        temperaturePub.publish(tempMsg);
        
        svsDiag->setLevel(diagnostic_msgs::DiagnosticStatus::OK);
    }
	
	//Camera stream
    if(cam->isNewDataAvailable())
        cam->MarkDataOld();
    
	//////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    sf::Thruster* th = (sf::Thruster*)auv->getActuator("ThrusterSurgePort");
    th->setSetpoint(thrustSetpoints[0]);

    th = (sf::Thruster*)auv->getActuator("ThrusterSurgeStarboard");
    th->setSetpoint(thrustSetpoints[1]);

    th = (sf::Thruster*)auv->getActuator("ThrusterHeaveBow");
    th->setSetpoint(thrustSetpoints[2]);

    th = (sf::Thruster*)auv->getActuator("ThrusterHeaveStern");
    th->setSetpoint(thrustSetpoints[3]);

    th = (sf::Thruster*)auv->getActuator("ThrusterSway");
    th->setSetpoint(thrustSetpoints[4]);

    ros::spinOnce();
}

//Callbacks
void G500AUVSimulationManager::ThrustCallback(const cola2_msgs::Setpoints& msg)
{
	if(msg.setpoints.size() != 5)
	{
		ROS_ERROR("Wrong number of thruster setpoints!");
		return;
	}
	
	for(unsigned int i=0; i<5; ++i)
		thrustSetpoints[i] = -msg.setpoints[i];
}

void G500AUVSimulationManager::CameraImageReady(sf::ColorCamera* c)
{
    publishCamera(cameraPub, cameraInfoPub, c, "proscilica"); 
}
