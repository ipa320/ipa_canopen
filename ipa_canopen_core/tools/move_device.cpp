// Copyright (c) 2012 Fraunhofer Institute
// for Manufacturing Engineering and Automation (IPA)
// See the file license.txt for copying permission.

// This program performs homing (referencing) of a device.
// See the user manual for details:
// https://github.com/ipa-tys/canopen/blob/master/doc/usermanual.pdf?raw=true

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

	//std::cout << deviceFile << std::endl;
	//std::cout << CANid << std::endl;
	//std::cout << canopen::syncInterval.count() << std::endl;
	//std::cout << targetVel << std::endl;
	//std::cout << accel << std::endl;

	canopen::devices[ CANid ] = canopen::Device(CANid);
	canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::schunkDefaultPDO_incoming( CANid, m ); };
	canopen::sendPos = canopen::schunkDefaultPDOOutgoing;

	canopen::init(deviceFile, canopen::syncInterval);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

  	canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, (uint8_t)canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	canopen::initDeviceManagerThread(canopen::deviceManager);
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
