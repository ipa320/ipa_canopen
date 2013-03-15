#include "canopen.h"

namespace canopen {

  /***************************************************************/
  //		Definitions
  /***************************************************************/

  std::chrono::milliseconds syncInterval;
  HANDLE h;
  std::map<uint8_t, Device> devices;
  std::map<std::string, DeviceGroup> deviceGroups;

  /***************************************************************/
  //		State machines
  /***************************************************************/

  // NMT State Machine
  void setNMTState(uint16_t CANid, std::string targetState){
    if (devices[CANid].NMTState_ == "INITIALISATION"){
      if (targetState == "PRE_OPERATIONAL"){
	std::cout << "Switching NMTState to PRE_OPERATIONAL at device with CAN_ID = " << CANid << std::endl;
      }
      else if (targetState == "STOPPED"){
	std::cout << "Invalid NMTState transition. Resetting the device with CAN_ID = " << CANid << std::endl;
	canopen::sendNMT(CANid, canopen::NMT_reset_node);
      }
      else if (targetState == "OPERATIONAL"){
	std::cout << "Invalid NMTState transition. Resetting the device with CAN_ID = " << CANid << std::endl;
	canopen::sendNMT(CANid, canopen::NMT_reset_node);
      }
    }
    else if (targetState == "RESET_APPLICATION"){
      std::cout << "Switching NMTState to RESET_APPLICATION at device with CAN_ID = " << CANid << std::endl;
      canopen::sendNMT(CANid, canopen::NMT_reset_node);
    }

    else if (targetState == "RESET_COMMUNICATION"){
      std::cout << "Switching NMTState to RESET_COMMUNICATION at device with CAN_ID = " << CANid << std::endl;
      canopen::sendNMT(CANid, canopen::NMT_reset_communication);
    }
    else if (targetState == "PRE_OPERATIONAL"){
      std::cout << "Switching NMTState to PRE_OPERATIONAL at device with CAN_ID = " << CANid << std::endl;
      canopen::sendNMT(CANid, canopen::NMT_enter_pre_operational);
    }

    else if (targetState == "OPERATIONAL"){
      std::cout << "Switching NMTState to OPERATIONAL at device with CAN_ID = " << CANid << std::endl;
      canopen::sendNMT(CANid, canopen::NMT_start_remote_node);
    }
    else if (targetState == "STOPPED"){
      std::cout << "Switching NMTState to STOPPED at device with CAN_ID = " << CANid << std::endl;
      canopen::sendNMT(CANid, canopen::NMT_stop_remote_node);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  // Motor State Machine
  void setMotorState(uint16_t CANid, std::string targetState) { // todo: not finished
    // if (devices[CANid].motorState_ == "fault")
    while (devices[CANid].motorState_ != targetState) {
      canopen::sendSDO(CANid, canopen::statusword);
      if (devices[CANid].motorState_ == "FAULT") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_fault_reset_0);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_fault_reset_1);
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      if (devices[CANid].motorState_ == "SWITCH_ON_DISABLED") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_shutdown);
      }
      if (devices[CANid].motorState_ == "READY_TO_SWITCH_ON") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_switch_on);
      }
      if (devices[CANid].motorState_ == "SWITCHED_ON") {
	canopen::sendSDO(CANid, canopen::controlword, canopen::controlword_enable_operation);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  /***************************************************************/
  //		Init sequence
  /***************************************************************/

  bool atFirstInit = true;

  bool openConnection(std::string devName) {
    h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
    if (!h) return false;
    errno = CAN_Init(h, CAN_BAUD_500K, CAN_INIT_TYPE_ST);
    //errno = CAN_Init(h,CAN_BAUD_500K, CAN_INIT_TYPE_EX);
    return true;
  }

  void init(std::string deviceFile, std::chrono::milliseconds syncInterval) {
    // canopen::devices must be set up before this function is called
    //CAN_Close(h);

    syncMsg.ID = 0x80;
    syncMsg.MSGTYPE = 0x00;
    syncMsg.LEN = 0x00;
    NMTmsg.ID = 0;
    NMTmsg.MSGTYPE = 0x00;
    NMTmsg.LEN = 2;
    nodeguardMsg.MSGTYPE = 0x01;
    nodeguardMsg.LEN = 0;

    if (!canopen::openConnection(deviceFile)) {
      std::cout << "Cannot open CAN device; aborting." << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << "Connection to CAN device established" << std::endl;

    // initialize alle special threads. currently: listenerthread
    canopen::initListenerThread(canopen::defaultListener);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    for (auto device : devices) {
 
      uint16_t CANid = (uint16_t)device.second.CANid_;

      // Running NMT State machine
      setNMTState(CANid, "RESET_APPLICATION");
      std::cout << "Initial NMT-state of device with CAN ID = " << CANid << " : " << devices[CANid].NMTState_ << std::endl;
      while (devices[CANid].NMTState_ != "PRE_OPERATIONAL"){
	setNMTState(CANid, "PRE_OPERATIONAL");
      }	
      setNMTState(CANid, "OPERATIONAL");

      // Setting sync_interval, ip_mode etc.
      //sendSDO(CANid, ip_time_units, (uint8_t) syncInterval.count() );
      //sendSDO(CANid, ip_time_index, ip_time_index_milliseconds);
      //sendSDO(CANid, sync_timeout_factor, sync_timeout_factor_disable_timeout);
      //sendSDO(CANid, life_time_factor, life_time_factor_value);							// preferences for nodeguarding: life_time = guard_time * 2 * number of devices
      //sendSDO(CANid, guard_time, guard_time_value);									//				 guard_time = 250ms

      // Running motor State machine
      std::cout << "Initial Motor-state of device with CAN ID: " << CANid << " : " << devices[CANid].motorState_ << std::endl;
      while (devices[CANid].motorState_ != "SWITCH_ON_DISABLED" & devices[CANid].motorState_ != "FAULT"){
        canopen::sendSDO(CANid, canopen::statusword);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }
      /*setMotorState(device.second.CANid_, "operation_enable");
      canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_shutdown);
      canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_switch_on);
      canopen::sendSDO(device.second.CANid_, canopen::controlword, canopen::controlword_enable_operation); */
    }
    std::cout << "initialisation finished" << std::endl;
  }

  /***************************************************************/
  //		Thread initialization
  /***************************************************************/

  // initialize listener thread for receiving all messages
  void initListenerThread(std::function<void ()> const& listener) {
    std::thread listener_thread(listener);
    listener_thread.detach();
    std::cout << "Listener thread initialized" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // initialize nodeguard thread for sending nodeguard messages to each device 
  void initNodeguardThread(std::function<void ()> const& nodeguard){
    std::thread nodeguard_thread(nodeguard);
    std::cout << "Nodeguard thread initialized" << std::endl;
    nodeguard_thread.detach();
  }

  // initialize devicemanager thread for sending PDOs automatically
  void initDeviceManagerThread(std::function<void ()> const& deviceManager) {
    std::thread device_manager_thread(deviceManager);
    device_manager_thread.detach();
    std::cout << "Device manager thread initalized" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  /***************************************************************/
  //		NMT Protocol
  /***************************************************************/

  TPCANMsg NMTmsg;

  /***************************************************************/
  //		SDO Protocol
  /***************************************************************/


  std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers
  { { statusword, statusword_incoming },
      { modes_of_operation_display, modes_of_operation_display_incoming }  };

  void sendSDO(uint8_t CANid, SDOkey sdo) {
    // for SDO read commands, e.g. statusword
    if (devices[CANid].NMTState_ == "PRE_OPERATIONAL" | devices[CANid].NMTState_ == "OPERATIONAL"){
      std::cout << "Sending SDO to device with CAN ID: " << (uint16_t)CANid << " and SDOkey: " << sdo.index << std::endl;
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
    else{
      std::cout << "ERROR: Cannot send SDOs while device is in mode STOPPED" << std::endl;
    }
  }

  void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value) {
    if (devices[CANid].NMTState_ == "PRE_OPERATIONAL" | devices[CANid].NMTState_ == "OPERATIONAL"){
      std::cout << "Sending SDO to device with CAN ID: " << (uint16_t)CANid << " and SDOkey: " << sdo.index << " and payload: " << value << std::endl;
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
    else{
      std::cout << "ERROR: Cannot send SDOs while device is in mode STOPPED" << std::endl;
    }
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value) {
    if (devices[CANid].NMTState_ == "PRE_OPERATIONAL" | devices[CANid].NMTState_ == "OPERATIONAL"){
      std::cout << "Sending SDO to device with CAN ID: " << (uint16_t)CANid << " and SDOkey: " << sdo.index << " and payload: " << value << std::endl;
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
    else{
      std::cout << "ERROR: Cannot send SDOs while device is in mode STOPPED" << std::endl;
    }
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value) {
    if (devices[CANid].NMTState_ == "PRE_OPERATIONAL" | devices[CANid].NMTState_ == "OPERATIONAL"){
      std::cout << "Sending SDO to device with CAN ID: " << (uint16_t)CANid << " and SDOkey: " << sdo.index << " and payload: " << (uint16_t)value << std::endl;
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
    else{
      std::cout << "ERROR: Cannot send SDOs while device is in mode STOPPED" << std::endl;
    }
  } 

  void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value) {
    if (devices[CANid].NMTState_ == "PRE_OPERATIONAL" | devices[CANid].NMTState_ == "OPERATIONAL"){
      std::cout << "Sending SDO to device with CAN ID: " << (uint16_t)CANid << " and SDOkey: " << sdo.index << " and payload: " << value << std::endl;
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
    else{
      std::cout << "ERROR: Cannot send SDOs while device is in mode STOPPED" << std::endl;
    }
  }

  void statusword_incoming(uint8_t CANid, BYTE data[8]) {

    uint16_t mydata = data[4] + (data[5] << 8);
    uint16_t received_state = mydata & 0x006F;
    uint16_t voltage_enabled = (mydata & 0x0010)>>4;
    uint16_t warning = (mydata & 0x0080)>>7;
    uint16_t drive_is_moving = (mydata & 0x0100)>>8;
    uint16_t remote = (mydata & 0x0200)>>9;
    uint16_t target_reached = (mydata & 0x0400)>>10;
    uint16_t internal_limit_active = (mydata & 0x0800)>>11;
    uint16_t ip_mode_active = (mydata & 0x1000)>>12;
    uint16_t homing_error = (mydata & 0x2000)>>13;
    uint16_t manufacturer_statusbit = (mydata & 0x4000)>>14;
    uint16_t drive_referenced = (mydata & 0x8000)>>15;


    if (received_state == 0x0000 | received_state == 0x0020){
      devices[CANid].motorState_ = "NOT_READY_TO_SWITCH_ON";
    }
    else if (received_state == 0x0040 | received_state == 0x0060){
      devices[CANid].motorState_ = "SWITCH_ON_DISABLED";
    }
    else if (received_state == 0x0021){
      devices[CANid].motorState_ = "READY_TO_SWITCH_ON";
    }
    else if (received_state == 0x0023){
      devices[CANid].motorState_ = "SWITCHED_ON";
    }
    else if (received_state == 0x0027){
      devices[CANid].motorState_ = "OPERATION_ENABLE";
    }
    else if (received_state == 0x0007){
      devices[CANid].motorState_ = "QUICK_STOP_ACTIVE";
    }
    else if (received_state == 0x000F | received_state == 0x002F | received_state == 0x0008 | received_state == 0x0028){
      devices[CANid].motorState_ = "FAULT";
    }

    devices[CANid].voltage_enabled_ = voltage_enabled;
    devices[CANid].driveReferenced_ = drive_referenced;

    std::cout << "received_state = " << received_state << std::endl;
    std::cout << "voltage_enabled = " << voltage_enabled << std::endl;
    std::cout << "warning = " << warning << std::endl;
    std::cout << "drive_is_moving = " << drive_is_moving << std::endl;
    std::cout << "remote = " << remote << std::endl;
    std::cout << "target_reched = " << target_reached << std::endl;
    std::cout << "internal_limit_active = " << internal_limit_active << std::endl;
    std::cout << "ip_mode_active = " << ip_mode_active << std::endl;
    std::cout << "homing_error = " << homing_error << std::endl;
    std::cout << "manufacturer_statusbit = " << manufacturer_statusbit << std::endl;
    std::cout << "drive_referenced = " << drive_referenced << std::endl;
    std::cout << "current motor state = " << devices[CANid].motorState_ << std::endl;
    
    /* for (int i=0; i<8; i++)
      printf("%02x ", data[i]);
      std::cout << std::endl; 

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
      devices[CANid].driveReferenced_ << std::endl;*/
  }

  void modes_of_operation_display_incoming(uint8_t CANid, BYTE data[8]) {
    // update variables of the corresponding device object
  }

  /***************************************************************/
  //		PDO Protocol
  /***************************************************************/

  TPCANMsg syncMsg;

  std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;

  std::function< void (uint16_t CANid, double positionValue) > sendPos;

  void schunkDefaultPDOOutgoing(uint16_t CANid, double positionValue) {
    if (devices[CANid].NMTState_ == "OPERATIONAL"){
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
    else{
      std::cout << "ERROR: PDOs can only be send while device is in mode OPERATIONAL" << std::endl;
    }
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

  // funtcion for devicemanager thread to automatically send PDOs
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

  /***************************************************************/
  //		Nodeguard Protocol
  /***************************************************************/

  TPCANMsg nodeguardMsg;

  void incomingNodeguardHandler(uint8_t CANid_, BYTE data[8]) {
    // get current NMT state from device with specific CANid
    uint16_t CANid = (uint16_t)CANid_;
    if (data[0] == 0x00){					// Bootup message
      devices[CANid].NMTState_ = "PRE_OPERATIONAL";
      std::cout << "Received bootup message from device with CAN-ID: " << CANid << std::endl;
      std::cout << "Device with CAN-ID " << CANid << " is now in state PRE_OPERATIONAL";
    }
    else if (data[0] == 0x04 | data[0] == 0x84){		// Device in state stopped
      devices[CANid].NMTState_ = "STOPPED";
      std::cout << "Device with CAN-ID " << CANid << " is now in state STOPPED";
    }
    else if (data[0] == 0x05 | data[0] == 0x85){		// Device in state operational
      devices[CANid].NMTState_ = "OPERATIONAL";
      std::cout << "Device with CAN-ID " << CANid << " is now in state OPERATIONAL";
    }
    else if (data[0] == 0x7F | data[0] == 0xFF){		// Device in state pre-operational
      devices[CANid].NMTState_ = "PRE_OPERATIONAL";
      std::cout << "Device with CAN-ID " << CANid << " is now in state PRE_OPERATIONAL";
    }
  }

  // funtion for nodeguard thread to update NMT State
  void nodeGuard(){
    while (true){
      for (auto CANid : canopen::DeviceGroup().CANids_){
        auto tic = std::chrono::high_resolution_clock::now();
        canopen::sendNodeguard(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(basic_guard_time));
      }
    }
  }

  /***************************************************************/
  //		Receive Data
  /***************************************************************/

  // function for listener thread to analyze incoming data
  void defaultListener(){
    while (true) {
      TPCANRdMsg m;
      if ((errno = LINUX_CAN_Read(h, &m))) {
	perror("LINUX_CAN_Read() error");
      }

      if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x400) { // incoming PDO 
	if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end()){
	  incomingPDOHandlers[m.Msg.ID](m);
        }
      }
      else if (m.Msg.ID == 0x48C){
        std::cout << "Dummy message with ID = 0x48C received" << std::endl;
      }
      else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5ff) { // incoming SDO 
	SDOkey sdoKey(m);
	if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end()){
	  incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
        }
      } 
      else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF){ // incoming Nodeguard
	incomingNodeguardHandler(m.Msg.ID - 0x700, m.Msg.DATA);
      }
    }
  }

}
