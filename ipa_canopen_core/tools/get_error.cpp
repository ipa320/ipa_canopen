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
 *   Author: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: July 2013
 *
 * \brief
 *   Get errors from the canopen device
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
#include <iostream>
#include <iomanip>
#include "ipa_canopen_core/canopen.h"
#include <sstream>

int main(int argc, char *argv[])
{

    if (argc != 4) {
        std::cout << "Arguments:" << std::endl
        << "(1) device file" << std::endl
        << "(2) CAN deviceID" << std::endl
           << "(3) Baud Rate" << std::endl
        << "Example: ./get_error /dev/pcan32 12 500K" << std::endl;
        return -1;
    }

    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    std::string deviceFile = std::string(argv[1]);
    canopen::baudRate = std::string(argv[3]);

    if (!canopen::openConnection(deviceFile, canopen::baudRate)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;

        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
        std::cout << "Baud Rate:" << canopen::baudRate << std::endl;
    }

    uint16_t CANid = std::stoi(std::string(argv[2]));

    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    std::shared_ptr<TPCANRdMsg> m;


    canopen::readErrorsRegister(CANid, m);

    /***************************************************************/
    //		Manufacturer specific errors register
    /***************************************************************/
    canopen::readManErrReg(CANid, m);

    /**************************
     * Hardware and Software Information
    *************************/

    std::vector<uint16_t> vendor_id = canopen::obtainVendorID(CANid, m);
    uint16_t rev_number = canopen::obtainRevNr(CANid, m);
    std::vector<uint16_t> product_code = canopen::obtainProdCode(CANid, m);
    std::vector<char> manufacturer_device_name = canopen::obtainManDevName(CANid,m);
    std::vector<char> manufacturer_hw_version =  canopen::obtainManHWVersion(CANid, m);
    std::vector<char> manufacturer_sw_version =  canopen::obtainManSWVersion(CANid, m);

        /****
         *Printing the data
         */

        std::cout << "vendor_id=0x";

        for (auto it : vendor_id)
        {
           std::cout <<  std::hex << it;
        }

        std::cout << std::endl;

        std::cout << "revision_number: "<< std::hex << int(rev_number) << std::dec << std::endl;
        std::cout << "device_name:";

        for (auto it : manufacturer_device_name)
        {
           std::cout << it;
        }

        std::cout << std::endl;

        std::cout << "hardware_version:";

        for (auto it : manufacturer_hw_version)
        {
           std::cout << it;
        }

        std::cout << std::endl;

        std::cout << "software_version:";

        for (auto it : manufacturer_sw_version)
        {
           std::cout << it;
        }

        std::cout << std::endl;


}
