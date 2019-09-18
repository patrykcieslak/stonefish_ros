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
//  parsed_simulator.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include <ros/ros.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include "ROSSimulationManager.h"

/*
    Command line arguments:
    1. Path to the data directory
    2. Path to the scenario description file (in the data directory)
    3. Horizontal resolution of the simulation window
    4. Vertical resolution of the simulation window
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "parsed_simulator", ros::init_options::NoSigintHandler);

	if(argc < 5)
	{
		ROS_FATAL("Not enough command line parameters provided!");
		return 1;
	}

	sf::RenderSettings s;
    s.windowW = atoi(argv[3]);
    s.windowH = atoi(argv[4]);
    s.shadows = sf::RenderQuality::QUALITY_HIGH;
    s.ao = sf::RenderQuality::QUALITY_HIGH;
    s.atmosphere = sf::RenderQuality::QUALITY_HIGH;
    s.ocean = sf::RenderQuality::QUALITY_HIGH;
    s.msaa = true;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = true;
    h.showBulletDebugInfo = false;
    h.showSensors = true;
    h.showActuators = true;
    h.showForces = true;
	
	sf::ROSSimulationManager manager(500.0, std::string(argv[2]));
	sf::GraphicalSimulationApp app("Stonefish Simulator", std::string(argv[1]), s, h, &manager); 
	app.Run();

	return 0;
}
