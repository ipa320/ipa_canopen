// Copyright (c) 2012 Fraunhofer Institute
// for Manufacturing Engineering and Automation (IPA)
// See the file license.txt for copying permission.

// This program performs homing (referencing) of a device.
// See the user manual for details:
// https://github.com/ipa-tys/canopen/blob/master/doc/usermanual.pdf?raw=true

#include <utility>
#include "canopen.h"

int main(int argc, char *argv[]) {

	if (argc != 3) {
		std::cout << "Arguments:" << std::endl
		<< "(1) device file" << std::endl
		<< "(2) CAN deviceID" << std::endl
		<< "Example: ./homing /dev/pcan32 12" << std::endl;
		return -1;
	}
	std::string deviceFile = std::string(argv[1]);
	uint16_t CANid = std::stoi(std::string(argv[2]));

  	// configure CANopen device objects and custom incoming and outgoing PDOs:

	canopen::devices[ CANid ] = canopen::Device(CANid);
	canopen::init(deviceFile, std::chrono::milliseconds(10));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
	canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_HOMING_MODE);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	canopen::sendSDO(CANid, canopen::CONTROLWORD, (uint16_t) (canopen::CONTROLWORD_ENABLE_OPERATION | canopen::CONTROLWORD_START_HOMING));
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	std::cout << "Homing complete" << std::endl;
}
