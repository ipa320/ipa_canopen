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
 * \date Date of creation: September 2013
 *
 * \brief
 *   Moves the elmo device according to an specific velocity and acceleration
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
#include "canopen.h"

int main(int argc, char *argv[]) {

	if (argc != 6) {
		std::cout << "Arguments:" << std::endl
		<< "(1) device file" << std::endl
		<< "(2) CAN deviceID" << std::endl
		<< "(3) sync rate [msec]" << std::endl
		<< "(4) target velocity [rad/sec]" << std::endl
		<< "(5) acceleration [rad/sec^2]" << std::endl
		<< "(enter acceleration '0' to omit acceleration phase)" << std::endl
		<< "Example 1: ./move_device /dev/pcan32 12 10 0.2 0.05" << std::endl
		<< "Example 2 (reverse direction): "
		<< "./move_device /dev/pcan32 12 10 -0.2 -0.05" << std::endl;
		return -1;
	}
	std::cout << "Interrupt motion with Ctrl-C" << std::endl;
	std::string deviceFile = std::string(argv[1]);
	uint16_t CANid = std::stoi(std::string(argv[2]));
	canopen::syncInterval = std::chrono::milliseconds(std::stoi(std::string(argv[3])));
	double targetVel = std::stod(std::string(argv[4]));
    double accel = std::stod(std::string(argv[5]));

    canopen::operation_mode = canopen::MODES_OF_OPERATION_PROFILE_VELOCITY_MODE;
	//std::cout << deviceFile << std::endl;
	//std::cout << CANid << std::endl;
	//std::cout << canopen::syncInterval.count() << std::endl;
	//std::cout << targetVel << std::endl;
	//std::cout << accel << std::endl;

	canopen::devices[ CANid ] = canopen::Device(CANid);
    canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos_elmo( CANid, m ); };
    canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status_elmo( CANid, m ); };
    canopen::sendVel = canopen::defaultPDOOutgoing_elmo;



    canopen::init_elmo(deviceFile, canopen::syncInterval);

	std::this_thread::sleep_for(std::chrono::milliseconds(100));


//CHANGED FROM INTERPOLATED
//    canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, (uint8_t)canopen::MODES_OF_OPERATION_PROFILE_VELOCITY_MODE);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));



       std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::initDeviceManagerThread(canopen::deviceManager_elmo);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	canopen::devices[CANid].setInitialized(true);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));



    //std::cout << "sending Statusword request" << std::endl;
    //canopen::sendSDO(CANid, canopen::STATUSWORD);
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));

    /*std::cout << "\t\t\t\tNMTState: " << canopen::devices[CANid].getNMTState() << std::endl;
    std::cout << "\t\t\t\tMotorState: " << canopen::devices[CANid].getMotorState() << std::endl;
    std::cout << "\t\t\t\tCANid: " << (uint16_t)canopen::devices[CANid].getCANid() << std::endl;
    std::cout << "\t\t\t\tActualPos: " << canopen::devices[CANid].getActualPos() << std::endl;
    std::cout << "\t\t\t\tDesiredPos: " << canopen::devices[CANid].getDesiredPos() << std::endl;
    std::cout << "\t\t\t\tActualVel: " << canopen::devices[CANid].getActualVel() << std::endl;
    std::cout << "\t\t\t\tDesiredVel: " << canopen::devices[CANid].getDesiredVel() << std::endl;*/

	// rest of the code is for moving the device:
	if (accel != 0) {  // accel of 0 means "move at target vel immediately"
	 	std::chrono::milliseconds accelerationTime( static_cast<int>(round( 1000.0 * targetVel / accel)) );
		double vel = 0;
		auto startTime = std::chrono::high_resolution_clock::now();
		auto tic = std::chrono::high_resolution_clock::now();

		// increasing velocity ramp up to target velocity:
		std::cout << "Accelerating to target velocity" << std::endl;
		while (tic < startTime + accelerationTime) {
			tic = std::chrono::high_resolution_clock::now();
			vel = accel * 0.000001 * std::chrono::duration_cast<std::chrono::microseconds>(tic-startTime).count();
			canopen::devices[ CANid ].setDesiredVel(vel);
			std::this_thread::sleep_for(canopen::syncInterval - (std::chrono::high_resolution_clock::now() - tic));
            canopen::sendSync();

		}
	}

	// constant velocity when target vel has been reached:
	std::cout << "Target velocity reached!" << std::endl;

    /*canopen::devices[ CANid ].setDesiredVel(targetVel);
    std::cout << "\t\t\t\t\t\tNMTState: " << canopen::devices[CANid].getNMTState() << std::endl;
    std::cout << "\t\t\t\t\t\tMotorState: " << canopen::devices[CANid].getMotorState() << std::endl;
    std::cout << "\t\t\t\t\t\tCANid: " << (uint16_t)canopen::devices[CANid].getCANid() << std::endl;
    std::cout << "\t\t\t\t\t\tActualPos: " << canopen::devices[CANid].getActualPos() << std::endl;
    std::cout << "\t\t\t\t\t\tDesiredPos: " << canopen::devices[CANid].getDesiredPos() << std::endl;
    std::cout << "\t\t\t\t\t\tActualVel: " << canopen::devices[CANid].getActualVel() << std::endl;
    std::cout << "\t\t\t\t\t\tDesiredVel: " << canopen::devices[CANid].getDesiredVel() << std::endl;*/

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	//std::cout << "sending Statusword request" << std::endl;
	//canopen::sendSDO(CANid, canopen::STATUSWORD);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
