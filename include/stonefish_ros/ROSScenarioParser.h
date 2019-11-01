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
//  ROSScenarioParser.h
//  stonefish_ros
//
//  Created by Patryk Cieslak on 17/09/19.
//  Copyright (c) 2019 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROSScenarioParser__
#define __Stonefish_ROSScenarioParser__

#include <core/ScenarioParser.h>
#include <ros/ros.h>

namespace sf
{
    class ROSSimulationManager;

    class ROSScenarioParser : public ScenarioParser
    {
    public:
        ROSScenarioParser(ROSSimulationManager* sm);

    protected:
        virtual bool PreProcess(XMLNode* root);
        virtual bool ParseRobot(XMLElement* element);
        virtual bool ParseSensor(XMLElement* element, Robot* robot);

    private:
        std::string SubstituteROSVars(const std::string& value);
        bool ReplaceROSVars(XMLNode* node);
    };
}

#endif
