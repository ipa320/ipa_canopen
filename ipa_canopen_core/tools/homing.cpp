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
 *   ROS package name: ipa_canopen_core
 *
 * \author
 *   Author: Eduard Herkel, Thiago de Freitas, Tobias Sing
 * \author
 *   Supervised by: Eduard Herkel, Thiago de Freitas, Tobias Sing, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Homing procedure for the Schunk devices.
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

#include <utility>
#include "ipa_canopen_core/canopen.h"

int main(int argc, char *argv[]) {

    if (argc != 4) {
        std::cout << "Arguments:" << std::endl
                  << "(1) device file" << std::endl
                  << "(2) CAN deviceID" << std::endl
                  << "(3) Baud Rate" << std::endl
                  << "Example: ./homing /dev/pcan32 12 500K" << std::endl;
        return -1;
    }
    std::string deviceFile = std::string(argv[1]);
    uint16_t CANid = std::stoi(std::string(argv[2]));
    canopen::baudRate = std::string(argv[3]);
    canopen::syncInterval = std::chrono::milliseconds(static_cast<int>(10.0));
    // configure CANopen device objects and custom incoming and outgoing PDOs:
    if (!canopen::openConnection(deviceFile,canopen::baudRate)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    canopen::devices[ CANid ] = canopen::Device(CANid);
    
    canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status( CANid, m ); };
    canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos( CANid, m ); };
    canopen::sendPos = canopen::defaultPDOOutgoing_interpolated;

    std::string chainName = "test_chain";
    std::vector <uint8_t> ids;
    ids.push_back(CANid);
    std::vector <std::string> j_names;
    j_names.push_back("joint_1");
    canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(ids, j_names);

    canopen::init(deviceFile, canopen::syncInterval);

    canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_HOMING_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    canopen::sendSDO(CANid, canopen::CONTROLWORD, (uint16_t) (canopen::CONTROLWORD_ENABLE_OPERATION | canopen::CONTROLWORD_START_HOMING));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Homing complete" << std::endl;
}
