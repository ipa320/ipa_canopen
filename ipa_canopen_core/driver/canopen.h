#ifndef CANOPEN_H
#define CANOPEN_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <thread>
#include <math.h>
#include <libpcan.h>
#include <utility>
#include <fcntl.h>    // for O_RDWR
#include <stdint.h>
#include <inttypes.h>

namespace canopen{

	extern std::chrono::milliseconds syncInterval;

	/***************************************************************/
	//		    define classes and structs
	/***************************************************************/

	class Device{

		private:

			uint8_t CANid_;
			std::string NMTState_;
			std::string motorState_;
			std::string deviceFile_;
			std::string name_;
			std::string group_;
			bool initialized_;
			bool driveReferenced_;
            bool ip_mode_active_;
            bool homingError_;
			double actualPos_;		// unit = rad
			double desiredPos_;		// unit = rad
			double actualVel_;		// unit = rad/sec
			double desiredVel_;		// unit = rad/sec
			std::chrono::milliseconds timeStamp_msec_;
			std::chrono::microseconds timeStamp_usec_;
			
			

            bool ready_switch_on_;
            bool switched_on_;
            bool op_enable_;
            bool fault_;
            bool volt_enable_;
            bool quick_stop_;
            bool switch_on_disabled_;
            bool warning_;

            bool mode_specific_;
            bool remote_;
            bool target_reached_;
            bool internal_limit_;
            bool op_specific_;
            bool op_specific1_;
            bool man_specific1_;
            bool man_specific2_;


		public:		

			Device() {};

			Device(uint8_t CANid):
				CANid_(CANid),
				desiredVel_(0),
				actualVel_(0),
				desiredPos_(0),
				actualPos_(0),
				initialized_(false),
				NMTState_("START_UP"),
				motorState_("START_UP") {};

			Device(uint8_t CANid, std::string name, std::string group, std::string bus):
				CANid_(CANid),
				name_(name),
				group_(group),
				deviceFile_(bus),
				desiredVel_(0),
				actualVel_(0),
				desiredPos_(0),
				actualPos_(0),
				initialized_(false) {};

			std::string getNMTState(){
				return NMTState_;
			}
			std::string getMotorState(){
				return motorState_;
			}
			uint8_t getCANid(){
				return CANid_;
			}
			std::string getDeviceFile(){
				return deviceFile_;
			}
			std::string getGroup(){
				return group_;
			}
			std::string getName(){
				return name_;
			}
			bool getInitialized(){
				return initialized_;
			}


			bool getVoltageEnabled(){
                return volt_enable_;
			}

            bool getReadySwitchOn(){
                return ready_switch_on_;
            }

            bool getSwitchOn(){
                return switched_on_;
            }

            bool getOpEnabled(){
                return op_enable_;
            }

            bool getQuickStop(){
                return quick_stop_;
            }

            bool getSwitchOnDisabled(){
                return switch_on_disabled_;
            }

            bool getWarning(){
                return warning_;
            }

            bool getModeSpecific(){
                return mode_specific_;
            }

            bool getRemote(){
                return remote_;
            }
            bool getTargetReached(){
                return target_reached_;
            }

            bool getInternalLimits(){
                return internal_limit_;
            }


            bool getOpSpec0(){
                return op_specific_;
            }

            bool getOpSpec1(){
                return op_specific1_;
            }

            bool getManSpec1(){
                return man_specific1_;
            }

            bool getmanSpec2(){
                return man_specific2_;
            }

            bool getHomingError(){
                return homingError_;
            }

            bool getFault(){
                return fault_;
            }

            bool getIPMode(){
                return ip_mode_active_;
            }

			bool getDriveReferenced(){
				return driveReferenced_;
			}
			double getActualPos(){
				return actualPos_;
			}
			double getDesiredPos(){
				return desiredPos_;
			}

			double getActualVel(){
				return actualVel_;
			}

			double getDesiredVel(){
				return desiredVel_;
			}

			inline std::chrono::milliseconds getTimeStamp_msec(){
				return timeStamp_msec_;
			}

			inline std::chrono::microseconds getTimeStamp_usec(){
				return timeStamp_usec_;
			}

			void setActualPos(double pos){
				actualPos_ = pos;
			}

			void setDesiredPos(double pos){
				desiredPos_ = pos;
			}

			void setActualVel(double vel){
				actualVel_ = vel;
			}

			void setDesiredVel(double vel){
				desiredVel_ = vel;
			}

			void setMotorState(std::string nextState){
				motorState_ = nextState;
			}

			void setNMTState(std::string nextState){
				NMTState_ = nextState;
			}
			
			
            void setVoltageEnabled(bool voltage_enabled){
                volt_enable_ = voltage_enabled;
			}

            void setReadySwitchON(bool r_switch_on){
                ready_switch_on_ = r_switch_on;
            }

            void setSwitchON(bool switch_on){
                switched_on_ = switch_on;
            }

            void setOpEnable(bool op_enable){
                op_enable_ = op_enable;
            }

            void setQuickStop(bool quick_stop){
                quick_stop_ = quick_stop;
            }

            void setSwitchOnDisable(bool switch_disabled){
                switch_on_disabled_ = switch_disabled;
            }

            void setWarning(bool warning){
                warning_ = warning;
            }


            void setModeSpec(bool modespec){
                mode_specific_ = modespec;
            }


            void setRemote(bool remote){
                remote_ = remote;
            }

            void setManSpec1(bool manspec1){
                man_specific1_ = manspec1;
            }

            void setTargetReached(bool target_reached){
                target_reached_ = target_reached;
            }

            void setInternalLimits(bool internal_limits){
                internal_limit_ = internal_limits;
            }


            void setManSpec2(bool manspec2){
                man_specific2_ = manspec2;
            }

            void setOpSpec1(bool opspec1){
                op_specific1_ = opspec1;
            }

            void setOpSpec0(bool opspec0){
                op_specific_ = opspec0;
            }

            void setFault(bool fault){
                fault_ = fault;
            }

            void setIPMode(bool ip_mode){
                ip_mode_active_ = ip_mode;
            }

            void setHoming(bool homing_error){
                homingError_ = homing_error;
            }

			void setInitialized(bool initialized){
				initialized_ = initialized;
			}

			void updateDesiredPos(){
                desiredPos_ += desiredVel_ * (syncInterval.count() / 1000.0);
			}

			void setTimeStamp_msec(std::chrono::milliseconds timeStamp){
				timeStamp_msec_ = timeStamp; 
			}

			void setTimeStamp_usec(std::chrono::microseconds timeStamp){
				timeStamp_usec_ = timeStamp;
			}
	};

	extern std::map<uint8_t, Device> devices;

	class DeviceGroup{

		private:

			std::vector<uint8_t> CANids_;
			std::vector<std::string> names_;

		public:

			DeviceGroup() {};
			
			DeviceGroup(std::vector<uint8_t> CANids):
				CANids_(CANids) {};

			DeviceGroup(std::vector<uint8_t> CANids, std::vector<std::string> names):
				CANids_(CANids),
				names_(names) {};

			std::vector<uint8_t> getCANids(){
				return CANids_;
			}

			std::vector<std::string> getNames(){
				return names_;
			}

			std::vector<double> getActualPos() {
      				std::vector<double> actualPos;
      				for (uint8_t CANid : CANids_)
					actualPos.push_back(devices[CANid].getActualPos());
      				return actualPos;
    			}

	    		std::vector<double> getDesiredPos() {
	      			std::vector<double> desiredPos;
	      			for (auto CANid : CANids_)
						desiredPos.push_back(devices[CANid].getDesiredPos());
	      			return desiredPos;
	    		}

			std::vector<double> getActualVel() {
				std::vector<double> actualVel;
				for (auto CANid : CANids_)
					actualVel.push_back(devices[CANid].getActualVel());
				return actualVel;
			}

			std::vector<double> getDesiredVel() {
				std::vector<double> desiredVel;
				for (auto CANid : CANids_)
                    desiredVel.push_back(devices[CANid].getDesiredVel());
				return desiredVel;
			}

			void setVel(std::vector<double> velocities) {
                for (unsigned int i=0; i<velocities.size(); i++) {
					devices[CANids_[i]].setDesiredVel(velocities[i]);
				}
    			}
	};

	struct SDOkey{
		uint16_t index;
		uint8_t subindex;

		inline SDOkey(TPCANRdMsg m):
			index((m.Msg.DATA[2] << 8) + m.Msg.DATA[1]),
			subindex(m.Msg.DATA[3]) {};

		inline SDOkey(uint16_t i, uint8_t s):
			index(i),
			subindex(s) {};
	};

	/***************************************************************/
	//		define global variables and functions
	/***************************************************************/

	inline bool operator<(const SDOkey &a, const SDOkey&b) {
    		return a.index < b.index || (a.index == b.index && a.subindex < b.subindex);
	}

	inline int32_t rad2mdeg(double phi){
		return static_cast<int32_t>(round(phi/(2*M_PI)*360000.0));
	}

	inline double mdeg2rad(int32_t alpha){
		return static_cast<double>(static_cast<double>(alpha)/360000.0*2*M_PI);
	}

	void statusword_incoming(uint8_t CANid, BYTE data[8]);
    void errorword_incoming(uint8_t CANid, BYTE data[8]);

	extern std::map<std::string, DeviceGroup> deviceGroups;	// DeviceGroup name -> DeviceGroup object
	extern HANDLE h;
	extern std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers;	
    extern std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingErrorHandlers;
	extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;

	/***************************************************************/
	//			define state machine functions
	/***************************************************************/

	void setNMTState(uint16_t CANid, std::string targetState);
	void setMotorState(uint16_t CANid, std::string targetState);

    /***************************************************************/
    //	define get errors functions
    /***************************************************************/

    void getErrors(uint16_t CANid);


	/***************************************************************/
	//	define init and recover variables and functions
	/***************************************************************/

	extern bool atFirstInit;
    extern bool recover_active;
    extern bool homing_active;

	bool openConnection(std::string devName);
	void init(std::string deviceFile, std::chrono::milliseconds syncInterval);
    void recover(std::string deviceFile, std::chrono::milliseconds syncInterval);
    void homing(std::string deviceFile, std::chrono::milliseconds syncInterval);
	
	extern std::function< void (uint16_t CANid, double positionValue) > sendPos;
    extern std::function< void (uint16_t CANid) > geterrors;

	/***************************************************************/
	//	define NMT constants, variables and functions
	/***************************************************************/

	const uint8_t NMT_START_REMOTE_NODE = 0x01;
	const uint8_t NMT_STOP_REMOTE_NODE = 0x02;
	const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
	const uint8_t NMT_RESET_NODE = 0x81;
	const uint8_t NMT_RESET_COMMUNICATION = 0x82;

	extern TPCANMsg NMTmsg;

	inline void sendNMT(uint8_t CANid, uint8_t command){
		//std::cout << "Sending NMT. CANid: " << (uint16_t)CANid << "\tcommand: " << (uint16_t)command << std::endl;
		NMTmsg.DATA[0] = command;
		NMTmsg.DATA[1] = CANid;
		CAN_Write(h, &NMTmsg);
	}

	/***************************************************************/
	//	define SYNC variables and functions
	/***************************************************************/

	extern TPCANMsg syncMsg;

	inline void sendSync() {
		CAN_Write(h, &syncMsg);
	}

	/***************************************************************/
	//		define NMT error control constants
	/***************************************************************/
	
	const SDOkey HEARTBEAT(0x1017,0x0);

	const uint16_t HEARTBEAT_TIME = 1500;

	/***************************************************************/
	//		define motor state constants
	/***************************************************************/

	const std::string MS_NOT_READY_TO_SWITCH_ON = "NOT_READY_TO_SWITCH_ON";
	const std::string MS_FAULT = "FAULT";
	const std::string MS_SWITCHED_ON_DISABLED = "SWITCHED_ON_DISABLED";
	const std::string MS_READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON";
	const std::string MS_SWITCHED_ON = "SWITCHED_ON";
	const std::string MS_OPERATION_ENABLED = "OPERATION_ENABLED";
	const std::string MS_QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE";
    const std::string MS_FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE";

	/***************************************************************/
	//		define SDO protocol constants and functions
	/***************************************************************/

	const SDOkey STATUSWORD(0x6041, 0x0);
    const SDOkey ERRORWORD(0x1001, 0x0);
	const SDOkey CONTROLWORD(0x6040, 0x0);
	const SDOkey MODES_OF_OPERATION(0x6060, 0x0);
 	const SDOkey MODES_OF_OPERATION_DISPLAY(0x6061, 0x0);
	const SDOkey SYNC_TIMEOUT_FACTOR(0x200e, 0x0);
	const SDOkey IP_TIME_UNITS(0x60C2, 0x1);
	const SDOkey IP_TIME_INDEX(0x60C2, 0x2);
    const SDOkey ERROR_CODE(0x603F, 0x0);
    const SDOkey ABORT_CONNECTION(0x6007, 0x0);
    const SDOkey QUICK_STOP(0x605A, 0x0);
    const SDOkey SHUTDOWN(0x605B, 0x0);
    const SDOkey DISABLE_CODE(0x605C, 0x0);
    const SDOkey HALT(0x605D, 0x0);
    const SDOkey FAULT(0x605E, 0x0);
    const SDOkey MODES(0x6060, 0x0);

    const uint16_t CONTROLWORD_SHUTDOWN = 6;
    const uint16_t CONTROLWORD_SWITCH_ON = 7;
    const uint16_t CONTROLWORD_ENABLE_OPERATION = 15;
    const uint16_t CONTROLWORD_START_HOMING = 16;
    const uint16_t CONTROLWORD_ENABLE_IP_MODE = 16;
    const uint16_t CONTROLWORD_DISABLE_INTERPOLATED = 7;
    const uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0x7D;
    const uint16_t CONTROLWORD_FAULT_RESET_0 = 0x00; //0x00;
    const uint16_t CONTROLWORD_FAULT_RESET_1 = 0x80;
    const uint16_t CONTROLWORD_HALT = 0x100;

	const uint8_t MODES_OF_OPERATION_HOMING_MODE = 0x6;
	const uint8_t MODES_OF_OPERATION_PROFILE_POSITION_MODE = 0x1;
	const uint8_t MODES_OF_OPERATION_VELOCITY_MODE = 0x2;
	const uint8_t MODES_OF_OPERATION_PROFILE_VELOCITY_MODE = 0x3;
	const uint8_t MODES_OF_OPERATION_TORQUE_PROFILE_MODE = 0x4;
	const uint8_t MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE = 0x7;

	const int8_t IP_TIME_INDEX_MILLISECONDS = 0xFD;
	const int8_t IP_TIME_INDEX_HUNDREDMICROSECONDS = 0xFC;
	const uint8_t SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT = 0;

	void sendSDO(uint8_t CANid, SDOkey sdo);
	void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value);
	void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value);
	void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value);
	void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value);

	/***************************************************************/
	//		define PDO protocol functions
	/***************************************************************/	

	void initDeviceManagerThread(std::function<void ()> const& deviceManager);
	void deviceManager();

    void defaultPDOOutgoing(uint16_t CANid, double positionValue);
    void defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m);
    void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m);

	/***************************************************************/
	//		define functions for receiving data
	/***************************************************************/

	void initListenerThread(std::function<void ()> const& listener);
	void defaultListener();
}

#endif
