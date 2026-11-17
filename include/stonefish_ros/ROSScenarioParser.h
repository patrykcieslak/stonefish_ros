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
//  Copyright (c) 2019-2026 Patryk Cieslak. All rights reserved.
//

#pragma once

#include <Stonefish/core/ScenarioParser.h>
#include <ros/ros.h>

namespace sf
{
    class ROSSimulationManager;

    class ROSScenarioParser : public ScenarioParser
    {
    public:
        //! A constructor.
        /*!
         \param sm a pointer to the simulation manager
         */
        ROSScenarioParser(ROSSimulationManager* sm);

    protected:
        //! A method used to pre-process the xml description file after loading.
        /*!
         \param root a pointer to a root node
         \param args a map of arguments and their values
         \return success
         */
        virtual bool PreProcess(XMLNode* root, 
            const std::map<std::string, std::string>& args = std::map<std::string, std::string>()) override;
        
        //! A method used to parse a definition of a velocity field (current, wind).
        /*!
         \param element a pointer to the XML node
         \return a pointer to the new velocity field object
         */
        virtual std::unique_ptr<VelocityField> ParseVelocityField(XMLElement* element) override;

        //! A method used to parse a robot description.
        /*!
         \param element a pointer to the XML node
         \return success
         */
        virtual bool ParseRobot(XMLElement* element) override;

        //! A method used to parse an animated object description.
        /*!
         \param element a pointer to the XML node
         \return success
         */
        virtual bool ParseAnimated(XMLElement* element) override;

        //! A method used to parse a single actuator description.
        /*!
         \param element a pointer to the XML node
         \param namePrefix an optional prefix string
         \return success
         */
        virtual std::unique_ptr<Actuator, ActuatorDeleter> 
            ParseActuator(XMLElement* element, std::string_view namePrefix) override;

        //! A method used to parse a single sensor description.
        /*!
         \param element a pointer to the XML node
         \param namePrefix an optional prefix string
         \return success
         */
        virtual std::unique_ptr<Sensor, SensorDeleter> 
            ParseSensor(XMLElement* element, std::string_view namePrefix) override;

        //! A method used to parse a communication device description.
        /*!
         \param element a pointer to the XML node
         \param namePrefix an optional prefix string
         \return pointer to the communication device
         */
        virtual std::unique_ptr<Comm, CommDeleter> 
            ParseComm(XMLElement* element, std::string_view namePrefix) override;

        //! A method used to parse a description of a light source.
        /*!
         \param element a pointer to the XML node
         \param namePrefix a string added at the beginning of the light name
         \return pointer to light
         */        
        virtual std::unique_ptr<Light> ParseLight(XMLElement* element, std::string_view namePrefix) override;

        //! A method used to parse a single contact description.
        /*!
         \param element a pointer to the XML node
         \return success
         */
        virtual bool ParseContact(XMLElement* element) override;

        //! A method used to parse a single glue joint description.
        /*!
         \param element a pointer to the XML node
         \return pointer to the fixed joint
        */
        virtual FixedJoint* ParseGlue(XMLElement* element) override;

    private:
        std::string SubstituteROSVars(const std::string& value);
        bool ReplaceROSVars(XMLNode* node);
    };
}

