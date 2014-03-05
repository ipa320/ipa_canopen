/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_ros
 *
 * \author
 *   Author: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: July 2013
 *
 * \brief
 *   Implementation of canopen.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ros/ros.h>
#include "ipa_canopen_ros/JointLimits.h"


bool JointLimits::checkVelocityLimits(std::vector<double> velocities)
{

    for (int i = 0; i < JointLimits::getDOF(); i++)
    {
        if(abs(velocities[i]) > JointLimits::getMaxVelocities()[i])
        {
            if(velocities[i] >= 0)
                // set velocities command to max value
                velocities[i] = JointLimits::getMaxVelocities()[i];
            else
                velocities[i] = -JointLimits::getMaxVelocities()[i];

            ROS_INFO("Velocity %f exceeds limit %f for axis %i. moving with max velocity %f instead", velocities[i], JointLimits::getMaxVelocities()[i], i, JointLimits::getMaxVelocities()[i]);
            return true;
        }

    }
    return false;
}


bool JointLimits::checkPositionLimits(std::vector<double> target_pos,std::vector<double> velocities)
{

    for (int i = 0; i < JointLimits::getDOF(); i++)
    {
        if ((target_pos[i] < JointLimits::getLowerLimits()[i]) && (velocities[i] < 0))
        {
            ROS_INFO("Skipping command: %f Target position exceeds lower limit (%f).", target_pos[i], JointLimits::getLowerLimits()[i]);
            // target position is set to actual position and velocity to Null. So only movement in the non limit direction is possible.

            return true;
        }

        // if target position is outer limits and the command velocity is in in direction away from working range, skip command
        if ((target_pos[i] > JointLimits::getUpperLimits()[i]) && (velocities[i] > 0))
        {
            ROS_INFO("Skipping command: %f Target position exceeds upper limit (%f).", target_pos[i], JointLimits::getUpperLimits()[i]);
            // target position is set to actual position. So only movement in the non limit direction is possible.

            return true;
        }
    }

    return false;
}
