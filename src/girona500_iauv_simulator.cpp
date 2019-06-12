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
//  girona500_iauv_simulator.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#include <ros/ros.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include "G500IAUVSimulationManager.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "girona500_iauv_simulator", ros::init_options::NoSigintHandler);

	if(argc < 4)
	{
		ROS_FATAL("Not enough command line parameters provided!");
		return 1;
	}

	sf::RenderSettings s;
    s.windowW = atoi(argv[2]);
    s.windowH = atoi(argv[3]);
    s.shadows = sf::RenderQuality::QUALITY_HIGH;
    s.ao = sf::RenderQuality::QUALITY_HIGH;
    s.atmosphere = sf::RenderQuality::QUALITY_HIGH;
    s.ocean = sf::RenderQuality::QUALITY_HIGH;
    s.msaa = true;

	sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = true;
    h.showActuators = true;
    h.showForces = false;
	
	G500IAUVSimulationManager manager(500.0);
	sf::GraphicalSimulationApp app("GIRONA500 I-AUV Simulator", std::string(argv[1]), s, h, &manager); 
	app.Run();

	return 0;
}