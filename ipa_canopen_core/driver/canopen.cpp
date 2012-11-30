#include "canopen.h"

namespace canopen {

  // ----------------- Variable definitions: --------------

  std::chrono::milliseconds syncInterval;
  HANDLE h;
  std::map<uint8_t, Device> devices;
  std::map<std::string, DeviceGroup> deviceGroups;

  std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers
  { { statusword, statusword_incoming },
      { modes_of_operation_display, modes_of_operation_display_incoming }  };

  std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;

  TPCANMsg NMTmsg;
  TPCANMsg syncMsg;
  TPCANMsg nodeguardMsg;
  bool atFirstInit = true;

  // ----------------- Function definitions: --------------

  bool openConnection(std::string devName) {
    h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
    if (!h) return false;
    errno = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
    return true;
  }

  void setMotorState(uint16_t CANid, std::string targetState) { // todo: not finished
    // if (devices[CANid].motorState_ == "fault")
    while (devices[CANid].motorState_ != targetState) {
      canopen::sendSDO(CANid, canopen::statusword);
      if (devices[CANid].motorState_ == "fault") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_fault_reset_0);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_fault_reset_1);
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      if (devices[CANid].motorState_ == "switch_on_disabled") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_shutdown);
      }
      if (devices[CANid].motorState_ == "ready_to_switch_on") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_switch_on);
      }
      if (devices[CANid].motorState_ == "switched_on") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_enable_operation);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void init(std::string deviceFile, std::chrono::milliseconds syncInterval) {
    // canopen::devices must be set up before this function is called

    CAN_Close(h);

    syncMsg.ID = 0x80;
    syncMsg.MSGTYPE = 0x00;
    syncMsg.LEN = 0x00;
    NMTmsg.ID = 0;
    NMTmsg.MSGTYPE = 0x00;
    NMTmsg.LEN = 2;
    nodeguardMsg.MSGTYPE = 0x01; // remote frame
    nodeguardMsg.LEN = 0;

    // if (atFirstInit) {
    
    if (!canopen::openConnection(deviceFile)) {
      std::cout << "Cannot open CAN device; aborting." << std::endl;
      exit(EXIT_FAILURE);
    }

    if (atFirstInit)
      canopen::initListenerThread(canopen::defaultListener);
      // atFirstInit = false;
    // }

    for (auto device : devices) {
      sendSDO(device.second.CANid_, ip_time_units, (uint8_t) syncInterval.count() );
      sendSDO(device.second.CANid_, ip_time_index, ip_time_index_milliseconds);
      sendSDO(device.second.CANid_, sync_timeout_factor, sync_timeout_factor_disable_timeout);

      // NMT & motor state machine:
      if (atFirstInit) {
	canopen::sendNMT(device.second.CANid_, canopen::NMT_stop);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	canopen::sendNMT(device.second.CANid_, canopen::NMT_start);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      setMotorState(device.second.CANid_, "operation_enable");
      /* canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_shutdown);
      canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_switch_on);
      canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_enable_operation); */
    }
    
    if (atFirstInit)
      atFirstInit = false;
  }

  void defaultListener() {
    while (true) {
      TPCANRdMsg m;
      if ((errno = LINUX_CAN_Read(h, &m))) {
	perror("LINUX_CAN_Read() error");
      }

      if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4ff) { // incoming PDO 
	if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
	  incomingPDOHandlers[m.Msg.ID](m);
      }
      else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5ff) { // incoming SDO 
	SDOkey sdoKey(m);
	if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
	  incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
      } else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF) 
	incomingNodeguardHandler(m.Msg.ID - 0x700, m.Msg.DATA);
    }
  }

  void initListenerThread(std::function<void ()> const& listener) {
    std::thread listener_thread(listener);
    listener_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void initDeviceManagerThread(std::function<void ()> const& deviceManager) {
    std::thread device_manager_thread(deviceManager);
    device_manager_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void deviceManager() {
    // todo: init, recover... (e.g. when to start/stop sending SYNCs)
    while (true) {
      auto tic = std::chrono::high_resolution_clock::now();
      for (auto device : devices) {
	if (device.second.initialized) {
	  devices[device.first].updateDesiredPos();
	  sendPos(device.second.CANid_, device.second.desiredPos_);
	}
      }
      sendSync();
      std::this_thread::sleep_for(syncInterval - (std::chrono::high_resolution_clock::now() - tic ));
    }
  }

  // ----------------- Functions to handle incoming data: --------------

  void incomingNodeguardHandler(uint8_t CANid, BYTE data[8]) {
    // update variables of the corresponding device object with nodeguard info
  }

  void statusword_incoming(uint8_t CANid, BYTE data[8]) {

    /* for (int i=0; i<8; i++)
      printf("%02x ", data[i]);
      std::cout << std::endl; */

    uint16_t mydata = data[4] + (data[5] << 8);
    // update variables of the corresponding device object, e.g.
    // printf("mydata: %04x\n", mydata);
    if ((mydata & 8) == 8) 
      devices[CANid].motorState_ = "fault";
    else if ((mydata & 0x4f) == 0x40) 
      devices[CANid].motorState_ = "switch_on_disabled";
    else if ((mydata & 0x6f) == 0x21)
      devices[CANid].motorState_ = "ready_to_switch_on";
    else if ((mydata & 0x6f) == 0x23)
      devices[CANid].motorState_ = "switched_on";
    else if ((mydata & 0x6f) == 0x27)
      devices[CANid].motorState_ = "operation_enable";

    devices[CANid].voltage_enabled_ = (mydata >> 4) & 0x1;
    devices[CANid].driveReferenced_ = (mydata >> 15) & 0x1;
    
    std::cout << "STATUSWORD incoming; drive referenced? " << 
      devices[CANid].driveReferenced_ << std::endl;
  }

  void modes_of_operation_display_incoming(uint8_t CANid, BYTE data[8]) {
    // update variables of the corresponding device object
  }

  void schunkDefaultPDO_incoming(uint8_t CANid, const TPCANRdMsg m) {
    double newPos = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + 
			     (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

    if (devices[CANid].timeStamp_msec_ != std::chrono::milliseconds(0) ||  // already 1 msg received
	devices[CANid].timeStamp_usec_ != std::chrono::microseconds(0)) {
      auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].timeStamp_msec_;
      auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].timeStamp_usec_;
      double deltaTime_double = static_cast<double>
	(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
      devices[ CANid ].actualVel_ = (newPos - devices[CANid].actualPos_) / deltaTime_double;

      if (! devices[CANid].initialized) {
	devices[ CANid ].desiredPos_ = devices[ CANid ].actualPos_;
	devices[ CANid ].initialized = true;
      }
    }
     
    devices[ CANid ].actualPos_ = newPos;
    devices[ CANid ].timeStamp_msec_ = std::chrono::milliseconds(m.dwTime);
    devices[ CANid ].timeStamp_usec_ = std::chrono::microseconds(m.wUsec);
  }
  
  // ----------------- Functions for sending out data: --------------

  void sendSDO(uint8_t CANid, SDOkey sdo) {
    // for SDO read commands, e.g. statusword
    TPCANMsg msg;
    msg.ID = CANid + 0x600; // 0x600 = SDO identifier
    msg.MSGTYPE = 0x00; // standard message
    msg.LEN = 4;
    msg.DATA[0] = 0x40;
    msg.DATA[1] = (sdo.index & 0xFF);
    msg.DATA[2] = (sdo.index >> 8) & 0xFF; 
    msg.DATA[3] = sdo.subindex;
    CAN_Write(h, &msg);
  }

  void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value) {
    TPCANMsg msg;
    msg.ID = CANid + 0x600; // 0x600 = SDO identifier
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = (sdo.index & 0xFF);
    msg.DATA[2] = (sdo.index >> 8) & 0xFF; 
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value) {
    TPCANMsg msg;
    msg.ID = CANid + 0x600; // 0x600 = SDO identifier
    msg.LEN = 8;
    msg.DATA[0] = 0x23;
    msg.DATA[1] = (sdo.index & 0xFF);
    msg.DATA[2] = (sdo.index >> 8) & 0xFF; // todo: & 0xFF not needed, I think
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value) {
    TPCANMsg msg;
    msg.ID = CANid + 0x600; // 0x600 = SDO identifier
    msg.LEN = 5;
    msg.DATA[0] = 0x2F;
    msg.DATA[1] = (sdo.index & 0xFF);
    msg.DATA[2] = (sdo.index >> 8) & 0xFF; // todo: & 0xFF not needed, I think
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    CAN_Write(h, &msg);
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value) {
    TPCANMsg msg;
    msg.ID = CANid + 0x600; // 0x600 = SDO identifier
    msg.LEN = 6;
    msg.DATA[0] = 0x2B;
    msg.DATA[1] = (sdo.index & 0xFF);
    msg.DATA[2] = (sdo.index >> 8) & 0xFF; // todo: & 0xFF not needed, I think
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    CAN_Write(h, &msg);
  } 

  std::function< void (uint16_t CANid, double positionValue) > sendPos;

  void schunkDefaultPDOOutgoing(uint16_t CANid, double positionValue) {
    static const uint16_t myControlword = 
      controlword_enable_operation | controlword_enable_ip_mode;
    TPCANMsg msg;
    msg.ID = 0x200 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = myControlword & 0xFF;
    msg.DATA[1] = (myControlword >> 8) & 0xFF; // todo: & not needed (?)
    msg.DATA[2] = 0;
    msg.DATA[3] = 0;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[4] = mdegPos & 0xFF;
    msg.DATA[5] = (mdegPos >> 8) & 0xFF;
    msg.DATA[6] = (mdegPos >> 16) & 0xFF;
    msg.DATA[7] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
  }

}
