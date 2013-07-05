#include "canopen.h"

namespace canopen{

	/***************************************************************/
	//			define global variables and functions
	/***************************************************************/
	
	std::chrono::milliseconds syncInterval;
	std::map<uint8_t, Device> devices;
	std::map<std::string, DeviceGroup> deviceGroups;
	HANDLE h;
    std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers{ { STATUSWORD, statusword_incoming } };
	std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;
	bool recover_active;

	/***************************************************************/
	//		define init and recover sequence
	/***************************************************************/

	bool atFirstInit = true;

	bool openConnection(std::string devName){
		h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
		if (!h)
			return false;
		errno = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
		return true;
	}

	void init(std::string deviceFile, std::chrono::milliseconds syncInterval){
        CAN_Close(h);

		NMTmsg.ID = 0;
		NMTmsg.MSGTYPE = 0x00;
		NMTmsg.LEN = 2;

		syncMsg.ID = 0x80;
		syncMsg.MSGTYPE = 0x00;

		syncMsg.LEN = 0x00;

		recover_active = false;

		if (!canopen::openConnection(deviceFile)){
			std::cout << "Cannot open CAN device; aborting." << std::endl;
			exit(EXIT_FAILURE);
		}
		else{
			std::cout << "Connection to CAN bus established" << std::endl;
		}

		if (atFirstInit){
			canopen::initListenerThread(canopen::defaultListener);
		}

        /*for (auto device : devices){
			std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected" << std::endl; 
            getErrors(device.second.getCANid());
        }*/

		for (auto device : devices){
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::cout << "Resetting CAN-device with CAN-ID " << (uint16_t)device.second.getCANid() << std::endl;
			canopen::sendNMT(device.second.getCANid(), canopen::NMT_RESET_NODE);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			canopen::sendNMT(device.second.getCANid(), canopen::NMT_START_REMOTE_NODE);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			//canopen::sendSDO(device.second.getCANid(), canopen::HEARTBEAT, canopen::HEARTBEAT_TIME);
			//std::cout << "Heartbeat protocol for device with CAN-ID " << (uint16_t)device.second.getCANid() << " started" << std::endl;
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			canopen::setMotorState(device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			sendSDO(device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			sendSDO(device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			sendSDO(device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (atFirstInit)
			atFirstInit = false;
	}

	void recover(std::string deviceFile, std::chrono::milliseconds syncInterval){
        CAN_Close(h);
	
		recover_active = true;

		NMTmsg.ID = 0;
		NMTmsg.MSGTYPE = 0x00;
		NMTmsg.LEN = 2;

		syncMsg.ID = 0x80;
		syncMsg.MSGTYPE = 0x00;

		syncMsg.LEN = 0x00;

		if (!canopen::openConnection(deviceFile)){
			std::cout << "Cannot open CAN device; aborting." << std::endl;
			exit(EXIT_FAILURE);
		}
		else{
			std::cout << "Connection to CAN bus established (recover)" << std::endl;
		}


		for (auto device : devices){
			std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected (recover)" << std::endl; 
		}

		for (auto device : devices){
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::cout << "Resetting CAN-device with CAN-ID " << (uint16_t)device.second.getCANid() << std::endl;
			canopen::sendNMT(device.second.getCANid(), canopen::NMT_RESET_NODE);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			canopen::sendNMT(device.second.getCANid(), canopen::NMT_START_REMOTE_NODE);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			canopen::sendSDO((uint16_t)device.second.getCANid(), canopen::STATUSWORD);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			//canopen::sendSDO(device.second.getCANid(), canopen::HEARTBEAT, canopen::HEARTBEAT_TIME);
			//std::cout << "Heartbeat protocol for device with CAN-ID " << (uint16_t)device.second.getCANid() << " started" << std::endl;
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			canopen::setMotorState(device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			sendSDO(device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			sendSDO(device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			sendSDO(device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	/***************************************************************/
	//		define state machine functions
	/***************************************************************/

	void setNMTState(uint16_t CANid, std::string targetState){
	
	}

	void setMotorState(uint16_t CANid, std::string targetState){
		while (devices[CANid].getMotorState() != targetState){
            canopen::sendSDO(CANid, canopen::STATUSWORD);
			if (devices[CANid].getMotorState() == MS_FAULT){
				canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen:: CONTROLWORD_FAULT_RESET_0);
				//std::this_thread::sleep_for(std::chrono::milliseconds(50));
				canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen:: CONTROLWORD_FAULT_RESET_1);
				//std::this_thread::sleep_for(std::chrono::milliseconds(200));
			}
			if (devices[CANid].getMotorState() == MS_SWITCHED_ON_DISABLED){
				canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SHUTDOWN);
			}
			if (devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON){
				canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SWITCH_ON);
			}
			if (devices[CANid].getMotorState() == MS_SWITCHED_ON){
				canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_ENABLE_OPERATION);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	/***************************************************************/
	//			define NMT variables
	/***************************************************************/

	TPCANMsg NMTmsg;

	/***************************************************************/
	//			define SYNC variables
	/***************************************************************/

	TPCANMsg syncMsg;

	/***************************************************************/
	//		define SDO protocol functions
	/***************************************************************/

	void sendSDO(uint8_t CANid, SDOkey sdo){
		TPCANMsg msg;
		msg.ID = CANid + 0x600;
		msg.MSGTYPE = 0x00;
		msg.LEN = 8;
		msg.DATA[0] = 0x40;
		msg.DATA[1] = sdo.index & 0xFF;
		msg.DATA[2] = (sdo.index >> 8) & 0xFF;
		msg.DATA[3] = sdo.subindex;
		msg.DATA[4] = 0x00;
		msg.DATA[5] = 0x00;
		msg.DATA[6] = 0x00;
		msg.DATA[7] = 0x00;
        std::cout << "" << std::endl;
		CAN_Write(h, &msg);
	}

	void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value){
		TPCANMsg msg;
		msg.ID = CANid + 0x600;
		msg.LEN = 8;
		msg.DATA[0] = 0x23;
		msg.DATA[1] = sdo.index & 0xFF;
		msg.DATA[2] = (sdo.index >> 8) & 0xFF;
		msg.DATA[3] = sdo.subindex;
		msg.DATA[4] = value & 0xFF;
		msg.DATA[5] = (value >> 8) & 0xFF;
		msg.DATA[6] = (value >> 16) & 0xFF;
		msg.DATA[7] = (value >> 24) & 0xFF;
		CAN_Write(h, &msg);
	}

	void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value){
		TPCANMsg msg;
		msg.ID = CANid + 0x600;
		msg.LEN = 8;
		msg.DATA[0] = 0x23;
		msg.DATA[1] = sdo.index & 0xFF;
		msg.DATA[2] = (sdo.index >> 8) & 0xFF;
		msg.DATA[3] = sdo.subindex;
		msg.DATA[4] = value & 0xFF;
		msg.DATA[5] = (value >> 8) & 0xFF;
		msg.DATA[6] = (value >> 16) & 0xFF;
		msg.DATA[7] = (value >> 24) & 0xFF;
		CAN_Write(h, &msg);
	}

	void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value){
		TPCANMsg msg;
		msg.ID = CANid + 0x600;
		msg.LEN = 8;
		msg.DATA[0] = 0x2F;
		msg.DATA[1] = sdo.index & 0xFF;
		msg.DATA[2] = (sdo.index >> 8) & 0xFF;
		msg.DATA[3] = sdo.subindex;
		msg.DATA[4] = value & 0xFF;
		msg.DATA[5] = 0x00;
		msg.DATA[6] = 0x00;
		msg.DATA[7] = 0x00;
		// TODO: why is it only working when inserting the following command
		std::cout << "" << std::endl;
		CAN_Write(h, &msg);


	}

	void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value){
		TPCANMsg msg;
		msg.ID = CANid + 0x600;
		msg.LEN = 8;
		msg.DATA[0] = 0x2B;
		msg.DATA[1] = sdo.index & 0xFF;
		msg.DATA[2] = (sdo.index >> 8) & 0xFF;
		msg.DATA[3] = sdo.subindex;
		msg.DATA[4] = value & 0xFF;
		msg.DATA[5] = (value >> 8) & 0xFF;
		msg.DATA[6] = 0x00;
		msg.DATA[7] = 0x00;
		CAN_Write(h, &msg);
	}

	/***************************************************************/
	//		define PDO protocol functions
	/***************************************************************/

	void initDeviceManagerThread(std::function<void ()> const& deviceManager) {
		std::thread device_manager_thread(deviceManager);
		device_manager_thread.detach();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	void deviceManager() {
		// todo: init, recover... (e.g. when to start/stop sending SYNCs)
		while (true) {
			auto tic = std::chrono::high_resolution_clock::now();
			if (!recover_active){
				for (auto device : canopen::devices) {
					if (device.second.getInitialized()) {
						devices[device.first].updateDesiredPos();
						sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
					}
				}
				canopen::sendSync();
				std::this_thread::sleep_for(syncInterval - (std::chrono::high_resolution_clock::now() - tic ));
			}

		}
	}

	std::function< void (uint16_t CANid, double positionValue) > sendPos;

	void schunkDefaultPDOOutgoing(uint16_t CANid, double positionValue) {
		static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
		TPCANMsg msg;
		msg.ID = 0x200 + CANid;
		msg.MSGTYPE = 0x00;
		msg.LEN = 8;
		msg.DATA[0] = myControlword & 0xFF;
		msg.DATA[1] = (myControlword >> 8) & 0xFF;
		msg.DATA[2] = 0;
		msg.DATA[3] = 0;
		int32_t mdegPos = rad2mdeg(positionValue);
		msg.DATA[4] = mdegPos & 0xFF;
		msg.DATA[5] = (mdegPos >> 8) & 0xFF;
		msg.DATA[6] = (mdegPos >> 16) & 0xFF;
		msg.DATA[7] = (mdegPos >> 24) & 0xFF;
		CAN_Write(h, &msg);
	}  

	void schunkDefaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m) {
		double newPos = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

		if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0)) {
			auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
			auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
			double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
			double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
			devices[CANid].setActualVel(result);
      			if (! devices[CANid].getInitialized()) {
				devices[CANid].setDesiredPos(newPos);
				devices[CANid].setInitialized(true);
			}
			//std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
		}

     
		devices[CANid].setActualPos(newPos);
		devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
		devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));



        bool fault = m.Msg.DATA[0] & 0x08;
        bool homing_error = m.Msg.DATA[1] & 0x80;
        devices[CANid].setFault(fault);
        devices[CANid].setHoming(homing_error);


	}

	/***************************************************************/
	//		define functions for receiving data
	/***************************************************************/

	void initListenerThread(std::function<void ()> const& listener){
		std::thread listener_thread(listener);
		listener_thread.detach();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//std::cout << "Listener thread initialized" << std::endl;
	}

	void defaultListener(){
		while(true){
			//std::cout << "Reading incoming data" << std::endl;
			TPCANRdMsg m;
			if (errno = LINUX_CAN_Read(h, &m))
				perror("LINUX_CAN_Read() error");

			// incoming SYNC
			else if (m.Msg.ID == 0x080){
				//std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
			}
		
			// incoming EMCY
			else if (m.Msg.ID >= 0x081 && m.Msg.ID <= 0x0FF){
                //std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
                if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
                    incomingEMCYHandlers[m.Msg.ID](m);
			}

			// incoming TIME
			else if (m.Msg.ID == 0x100){
				//std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
			}

            // incoming PD0
			else if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4FF){
                //std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl; ;
				if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end()) 
					incomingPDOHandlers[m.Msg.ID](m); 
			}

			// incoming SD0
			else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5FF){
                //std::cout << std::hex << "SDO received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
				SDOkey sdoKey(m);
				if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
					incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
			}

			// incoming NMT error control
			else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF){
				uint16_t CANid = m.Msg.ID - 0x700;
				if (m.Msg.DATA[0] == 0x00){
					std::cout << "Bootup received. Node-ID =  " << (uint16_t)(m.Msg.ID - 0x700) << std::endl;	
				}
				else{
					std::cout << "NMT error control received:  " << (uint16_t)(m.Msg.ID - 0x700) << "  " << (uint16_t)m.Msg.DATA[0] << std::endl;
				}
			}
			else{
				 std::cout << "Received unknown message" << std::endl;
			}
		}
	}

/******************************************************************************
 * Define get errors function
 *****************************************************************************/
    void getErrors(uint16_t CANid){
    TPCANMsg msg;
    msg.ID = CANid + 0x1001;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x40;
    msg.DATA[1] = 0x1001 & 0xFF;
    msg.DATA[2] = (0x1001 & 0xFF00)>>8;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    std::cout << "SDO sent Error" << (uint16_t)msg.ID << "  " << (uint16_t)msg.DATA[0] << " " << (uint16_t)msg.DATA[1] << " " << (uint16_t)msg.DATA[2] << " " << (uint16_t)msg.DATA[3] << " " << (uint16_t)msg.DATA[4] << " " << (uint16_t)msg.DATA[5] << " " << (uint16_t)msg.DATA[6] << " " << (uint16_t)msg.DATA[7] << std::endl;
    CAN_Write(h, &msg);
    }

void statusword_incoming(uint8_t CANid, BYTE data[8]) {

		uint16_t mydata = data[4] + (data[5] << 8);
		uint16_t received_state = mydata & 0x006F;

		if (received_state == 0x0000 | received_state == 0x0020){
			devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
		}
		else if (received_state == 0x0040 | received_state == 0x0060){
			devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
		}
		else if (received_state == 0x0021){
			devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);
		}
		else if (received_state == 0x0023){
			devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
		}
		else if (received_state == 0x0027){
			devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
		}
		else if (received_state == 0x0007){
			devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);
		}
		else if (received_state == 0x000F | received_state == 0x002F | received_state == 0x0008 | received_state == 0x0028){
			devices[CANid].setMotorState(canopen::MS_FAULT);
		}

        //std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
	}
}
