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

#ifndef JOINTLIMITS_H
#define JOINTLIMITS_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <cstdlib>


class JointLimits
    {
        private:
            std::vector<double> MaxVelocities_;
            std::vector<double> LowerLimits_;
            std::vector<double> UpperLimits_;
            std::vector<double> Offsets_;
            std::vector<double> HWPositive_;
            std::vector<double> HWNegative_;
            int DOF_;

        public:

            bool checkPositionLimits(std::vector<double> positions,std::vector<double> velocities);
            bool checkVelocityLimits(std::vector<double> velocities);

            bool checkHardwareLimits(std::vector<double> velocities);

            int setMaxVelocities(std::vector<double> max_vel)
            {
                if ((int)max_vel.size() == getDOF())
                {
                    MaxVelocities_ = max_vel;
                    return 0;
                }
                return -1;

            }

            int setLowerLimits(std::vector<double> low_lim)

            {
                if ((int)low_lim.size() == getDOF())
                {
                    LowerLimits_ = low_lim;
                    return 0;
                }
                return -1;

            }

            int setUpperLimits(std::vector<double> up_lim)
            {
                if ((int)up_lim.size() == getDOF())
                {
                    UpperLimits_ = up_lim;
                    return 0;
                }
                return -1;
            }

            int setOffsets(std::vector<double> offs)
            {
                if ((int)offs.size() == getDOF())
                {
                    Offsets_ = offs;
                    return 0;
                }
                return -1;
            }

            void setDOF(int dof)
            {
                DOF_ = dof;
            }

            std::vector<double> getMaxVelocities()
            {
                return MaxVelocities_;
            }

            std::vector<double> getLowerLimits()
            {
                return LowerLimits_;
            }

            std::vector<double> getUpperLimits()
            {
                return UpperLimits_;
            }

            std::vector<double> getOffsets()
            {
                return Offsets_;
            }

            int getDOF()
            {
                return DOF_;
            }


    };

#endif // JOINTLIMITS
