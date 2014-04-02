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
 *   Author: Thiago de Freitas, Tobias Sing, Eduard Herkel
 * \author
 *   Supervised by: Thiago de Freitas email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen driver.
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

#include <ipa_canopen_core/canopen.h>
#include <sstream>
#include <cstring>
#include <unordered_map>

namespace canopen
{

/***************************************************************/
//			define global variables and functions
/***************************************************************/
bool sdo_protect=false;
BYTE protect_msg[8];

std::chrono::milliseconds syncInterval;
std::string baudRate;
std::map<uint8_t, Device> devices;
std::map<std::string, DeviceGroup> deviceGroups;
HANDLE h;
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingDataHandlers { { STATUSWORD, sdo_incoming }, { DRIVERTEMPERATURE, sdo_incoming }, { MODES_OF_OPERATION_DISPLAY, sdo_incoming } };
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingErrorHandlers { { ERRORWORD, errorword_incoming }, { MANUFACTURER, errorword_incoming } };
std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingManufacturerDetails { {MANUFACTURERHWVERSION, manufacturer_incoming}, {MANUFACTURERDEVICENAME, manufacturer_incoming}, {MANUFACTURERSOFTWAREVERSION, manufacturer_incoming} };
std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;
bool recover_active;
bool halt_active;

bool halt_positive;
bool halt_negative;

bool use_limit_switch=false;

uint8_t operation_mode;
std::string operation_mode_param;

std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

std::chrono::duration<double> elapsed_seconds;


/***************************************************************/
//		define init and recover sequence
/***************************************************************/

bool atFirstInit = true;

bool openConnection(std::string devName, std::string baudrate)
{
    h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
    if (!h)
        return false;

    errno = CAN_Init(h, baudrates[baudrate], CAN_INIT_TYPE_ST);

    return true;
}

void pre_init()
{

    for (auto device : (canopen::devices))
    {
        /*********************************************/

        getErrors(device.second.getCANid());

        /***************************************************************/
        //		Manufacturer specific errors register
        /***************************************************************/
        readManErrReg(device.second.getCANid());

        /**************************
       * Hardware and Software Information
      *************************/
        canopen::uploadSDO(device.second.getCANid(), MANUFACTURERDEVICENAME);
    }
}

bool init(std::string deviceFile, std::chrono::milliseconds syncInterval)
{

    if(atFirstInit)
    {
        atFirstInit = false;

        CAN_Close(h);

        canopen::initDeviceManagerThread(canopen::deviceManager);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        bool connection_success;
        connection_success = canopen::openConnection(deviceFile, canopen::baudRate);
        
        if (!connection_success)
        {
            std::cout << "Cannot open CAN device; aborting." << std::endl;
            exit(EXIT_FAILURE);
        }
        else
        {

                canopen::initListenerThread(canopen::defaultListener);

                canopen::pre_init();

                while(sdo_protect)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                std::cout << "Resetting devices " << std::endl;
                canopen::sendNMT(0x00, canopen::NMT_RESET_NODE);
                
                for(auto device : devices)
                {
                    bool nmt_init = devices[device.second.getCANid()].getNMTInit();
                    std::cout << "Waiting for Node: " << (uint16_t)device.second.getCANid() << " to become available" << std::endl;
                    
                    while(!nmt_init)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        nmt_init = devices[device.second.getCANid()].getNMTInit();
                    }
                    std::cout << "Node: " << (uint16_t)device.second.getCANid() << " is now available" << std::endl;
                }
                
                canopen::sendNMT(0x00, canopen::NMT_START_REMOTE_NODE);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        for (auto device : devices)
        {

            std::cout << "Initialized the PDO mapping for Node:" << (uint16_t)device.second.getCANid() << std::endl;

            for(int pdo_object=0; pdo_object<=3; pdo_object++)
            {
                canopen::disableTPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::clearTPDOMapping(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::disableRPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::clearRPDOMapping(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }


            if(canopen::use_limit_switch)
            {

                std::vector<std::string> tpdo1_registers {"604100", "60FD00"};
                std::vector<int> tpdo1_sizes {0x10,0x20};

                canopen::makeTPDOMapping(0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));
            }
            else
            {
                std::vector<std::string> tpdo1_registers {"604100", "606100"};
                std::vector<int> tpdo1_sizes {0x10,0x08};

                canopen::makeTPDOMapping(0,tpdo1_registers, tpdo1_sizes, u_int8_t(0xFF));

            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            std::vector<std::string> tpdo4_registers {"606400", "606C00"};
            std::vector<int> tpdo4_sizes {0x20,0x20};

            canopen::makeTPDOMapping(3, tpdo4_registers, tpdo4_sizes, u_int8_t(0x01));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            std::vector<std::string> rpdo1_registers {"604000"};
            std::vector<int> rpdo1_sizes {0x10};

            std::vector<std::string> rpdo2_registers {"60C101"};
            std::vector<int> rpdo2_sizes {0x20};

            canopen::makeRPDOMapping(0, rpdo1_registers, rpdo1_sizes, u_int8_t(0xFF));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            canopen::makeRPDOMapping(1, rpdo2_registers, rpdo2_sizes, u_int8_t(0x01));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            for(int pdo_object=0; pdo_object<=3; pdo_object++)
            {
                canopen::enableTPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));

                canopen::enableRPDO(pdo_object);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));

        }
    }

     recover_active = false;
     canopen::setObjects();


    for (auto device : devices)
    {
        canopen::sendSync();
        if(device.second.getMotorState() == MS_OPERATION_ENABLED)
        {
            std::cout << "Node" << (uint16_t)device.second.getCANid() << "is already operational" << std::endl;
        }
        else
        {
            canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState((uint16_t)device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));


            canopen::controlPDO(device.second.getCANid(), canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            
            canopen::devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
            canopen::devices[device.second.getCANid()].setDesiredVel(0);
            //Necessary otherwise sometimes Schunk devices complain for Position Track Error
            sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());

            canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(device.second.getCANid(), DRIVERTEMPERATURE);
            canopen::uploadSDO(device.second.getCANid(), MODES_OF_OPERATION_DISPLAY);

        }

    }

    for (auto device : devices)
    {
        getErrors(device.second.getCANid());
        readManErrReg(device.second.getCANid());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if(device.second.getIPMode())
        {
            std::cout << "Concluded driver side init succesfully for Node" << (uint16_t)device.second.getCANid() << std::endl;
            canopen::devices[device.second.getCANid()].setInitialized(true);
        }
        else
        {
            std::cout << "Problems occured during driver side init for Node" << (uint16_t)device.second.getCANid()  << std::endl;
            canopen::devices[device.second.getCANid()].setInitialized(false);
            return false;
        }

    }

    return true;

}


bool recover(std::string deviceFile, std::chrono::milliseconds syncInterval)
{

    recover_active = true;

    for (auto device : devices)
    {

        if(device.second.getIPMode())
        {
            std::cout << "Node" << device.second.getCANid() << "is already operational" << std::endl;
        }
        else
        {

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_HALT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_DISABLE_INTERPOLATED, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(),canopen::CONTROLWORD_QUICKSTOP, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(device.second.getCANid(), canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(device.second.getCANid(), canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(device.second.getCANid(), canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(device.second.getCANid(), canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)device.second.getCANid(), canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)device.second.getCANid(), canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(device.second.getCANid(), canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(device.second.getCANid(), DRIVERTEMPERATURE);
            canopen::uploadSDO(device.second.getCANid(), MODES_OF_OPERATION_DISPLAY);

            getErrors(device.second.getCANid());
        }


        devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
        devices[device.second.getCANid()].setDesiredVel(0);

    }
    recover_active = false;

    for (auto device : devices)
    {

        if(device.second.getIPMode())
        {
            std::cout << "Concluded driver side recover succesfully" << std::endl;
        }
        else
        {
            std::cout << "Problems occured during driver side recover" << std::endl;
            return false;
        }
    }

    return true;

}



void halt(std::string deviceFile, std::chrono::milliseconds syncInterval)
{
    CAN_Close(h);

    NMTmsg.ID = 0;
    NMTmsg.MSGTYPE = 0x00;
    NMTmsg.LEN = 2;

    syncMsg.ID = 0x80;
    syncMsg.MSGTYPE = 0x00;

    syncMsg.LEN = 0x00;

    if (!canopen::openConnection(deviceFile, canopen::baudRate))
    {
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        // std::cout << "Connection to CAN bus established (recover)" << std::endl;
    }


    for (auto device : devices)
    {
        //std::cout << "Module with CAN-id " << (uint16_t)device.second.getCANid() << " connected (recover)" << std::endl;
    }

    for (auto device : devices)
    {


        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROLWORD_HALT);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROLWORD_DISABLE_INTERPOLATED);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen:: CONTROL_WORD_DISABLE_VOLTAGE);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(device.second.getCANid(), canopen::CONTROLWORD, canopen::CONTROLWORD_QUICKSTOP);
        canopen::uploadSDO(device.second.getCANid(), canopen::STATUSWORD);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

/***************************************************************/
//		define state machine functions
/***************************************************************/

void setNMTState(uint16_t CANid, std::string targetState)
{

}

void setMotorState(uint16_t CANid, std::string targetState)
{

    start = std::chrono::high_resolution_clock::now();

    while (devices[CANid].getMotorState() != targetState)
    {
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        if(elapsed_seconds.count() > 3)
            return;
        canopen::uploadSDO(CANid, canopen::STATUSWORD);
        if (devices[CANid].getMotorState() == MS_FAULT)
        {
            canopen::uploadSDO(CANid, canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if(!devices[CANid].getFault())
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_0, 0x00);
            }
            else
            {
                //std::this_thread::sleep_for(std::chrono::milliseconds(50));
                canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_1, 0x00);
            }

        }

        if (devices[CANid].getMotorState() == MS_NOT_READY_TO_SWITCH_ON)
        {
            canopen::uploadSDO(CANid, canopen::STATUSWORD);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
        }

        if (devices[CANid].getMotorState() == MS_SWITCHED_ON_DISABLED)
        {
            //canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SHUTDOWN);
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
        }

        if (devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED){
                canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            }
            else
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SWITCH_ON, 0x00);
            }
        }

        if (devices[CANid].getMotorState() == MS_SWITCHED_ON)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
            }
            else{
                //canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_ENABLE_OPERATION);
                canopen::controlPDO(CANid, canopen::CONTROLWORD_ENABLE_OPERATION, 0x00);
            }
        }

        if (devices[CANid].getMotorState() == MS_OPERATION_ENABLED)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROL_WORD_DISABLE_VOLTAGE);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_SHUTDOWN);
            }
            else
            {
                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen::CONTROLWORD_DISABLE_OPERATION);
            }
        }

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

void requestDataBlock1(uint8_t CANid)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x60;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);
}

void requestDataBlock2(uint8_t CANid)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x70;
    msg.DATA[1] = 0x00;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write(h, &msg);
}

void controlPDO(uint8_t CANid, u_int16_t control1, u_int16_t control2)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x200;
    msg.MSGTYPE = 0x00;
    msg.LEN = 2;
    msg.DATA[0] = control1;
    msg.DATA[1] = control2;
    CAN_Write(h, &msg);
}

void uploadSDO(uint8_t CANid, SDOkey sdo)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
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
    CAN_Write(h, &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, uint32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
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

void sendSDO(uint8_t CANid, SDOkey sdo, int32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
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

void sendSDO_unknown(uint8_t CANid, SDOkey sdo, int32_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid + 0x600;
    msg.LEN = 8;
    msg.DATA[0] = 0x22;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

void sendSDO(uint8_t CANid, SDOkey sdo, uint8_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
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
    CAN_Write(h, &msg);


}

void sendSDO(uint8_t CANid, SDOkey sdo, uint16_t value)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
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

void initDeviceManagerThread(std::function<void ()> const& deviceManager)
{
    std::thread device_manager_thread(deviceManager);
    device_manager_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void deviceManager()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;

    std::chrono::duration<double> elapsed_time;


    while (true)
    {
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        auto tic = std::chrono::high_resolution_clock::now();
        if (!recover_active)
        {
            for (auto device : canopen::devices)
            {
                if(elapsed_seconds.count() > 2)
                {
                    start = std::chrono::high_resolution_clock::now();
                    canopen::uploadSDO(device.second.getCANid(), DRIVERTEMPERATURE);
                    getErrors(device.second.getCANid());
                    readManErrReg(device.second.getCANid());
                }

                if (device.second.getInitialized())
                {
                    devices[device.first].updateDesiredPos();
                    sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
                }
            }
            canopen::sendSync();
            std::this_thread::sleep_for(syncInterval - (std::chrono::high_resolution_clock::now() - tic ));
        }

    }
}


std::function< void (uint16_t CANid, double velocityValue) > sendVel;
std::function< void (uint16_t CANid, double positionValue) > sendPos;
std::function< void (uint16_t CANid, double positionValue, double velocityValue) > sendPosPPMode;

void defaultPDOOutgoing_interpolated(uint16_t CANid, double positionValue)
{
    static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = 0x300 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 4;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[0] = mdegPos & 0xFF;
    msg.DATA[1] = (mdegPos >> 8) & 0xFF;
    msg.DATA[2] = (mdegPos >> 16) & 0xFF;
    msg.DATA[3] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

void defaultPDOOutgoing(uint16_t CANid, double positionValue)
{
    static const uint16_t myControlword = (CONTROLWORD_ENABLE_OPERATION | CONTROLWORD_ENABLE_IP_MODE);
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = 0x200 + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = myControlword & 0xFF;
    msg.DATA[1] = (myControlword >> 8) & 0xFF;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[4] = mdegPos & 0xFF;
    msg.DATA[5] = (mdegPos >> 8) & 0xFF;
    msg.DATA[6] = (mdegPos >> 16) & 0xFF;
    msg.DATA[7] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
}




void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m)
{


    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    // std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[0] << std::endl;


}

void defaultPDO_incoming_status(uint16_t CANid, const TPCANRdMsg m)
{

    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    int8_t mode_display = m.Msg.DATA[2];

    if(canopen::use_limit_switch)
    {


        uint16_t limit_switch_ = m.Msg.DATA[2];

        bool hardware_limit_positive = limit_switch_ & 0x02;
        bool hardware_limit_negative = limit_switch_ & 0x01;

        devices[CANid].setNegativeLimit(hardware_limit_negative);
        devices[CANid].setPositiveLimit(hardware_limit_positive);
    }

    bool ready_switch_on = mydata_low & 0x01;
    bool switched_on = mydata_low & 0x02;
    bool op_enable = mydata_low & 0x04;
    bool fault = mydata_low & 0x08;
    // std::cout << "fault PDO" << fault << std::endl;
    bool volt_enable = mydata_low & 0x10;
    bool quick_stop = mydata_low & 0x20;
    bool switch_on_disabled = mydata_low & 0x40;
    bool warning = mydata_low & 0x80;

    bool mode_specific = mydata_high & 0x01;
    bool remote = mydata_high & 0x02;
    bool target_reached = mydata_high & 0x04;
    bool internal_limit = mydata_high & 0x08;
    bool op_specific = mydata_high & 0x10;
    bool op_specific1 = mydata_high & 0x20;
    bool man_specific1 = mydata_high & 0x40;
    bool man_specific2 = mydata_high & 0x80;

    bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;


    if(!ready_switch_on)
    {
        if(fault)
        {
            devices[CANid].setMotorState(canopen::MS_FAULT);
        }
        else if(switch_on_disabled)
        {
            devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
        }
        else
            devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //if(volt_enable)
                // {
                devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                // }

            }
            else
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
        }
        else if(!quick_stop)
            devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

        else
            devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
        devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);


    devices[CANid].setFault(fault);
    devices[CANid].setIPMode(ip_mode);
    devices[CANid].setHoming(op_specific);
    devices[CANid].setOpSpec0(op_specific);
    devices[CANid].setOpSpec1(op_specific1);
    devices[CANid].setManSpec1(man_specific1);
    devices[CANid].setManSpec2(man_specific2);
    devices[CANid].setInternalLimits(internal_limit);
    devices[CANid].setTargetReached(target_reached);
    devices[CANid].setRemote(remote);
    devices[CANid].setModeSpec(mode_specific);
    devices[CANid].setWarning(warning);
    devices[CANid].setSwitchOnDisable(switch_on_disabled);
    devices[CANid].setQuickStop(quick_stop);
    devices[CANid].setOpEnable(op_enable);
    devices[CANid].setVoltageEnabled(volt_enable);
    devices[CANid].setReadySwitchON(ready_switch_on);
    devices[CANid].setSwitchON(switched_on);

    devices[CANid].setCurrentModeofOperation(mode_display);



    // std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
}

void defaultPDO_incoming_pos(uint16_t CANid, const TPCANRdMsg m)
{
    double newPos = mdeg2rad(m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24));
    double newVel = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24));

    //newPos = devices[CANid].getConversionFactor()*newPos; //TODO: conversion from yaml file
    //newVel = devices[CANid].getConversionFactor()*newVel;

    if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0))
    {
        auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
        auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
        double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
        double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
        devices[CANid].setActualVel(result);
        if (! devices[CANid].getInitialized())
        {
            devices[CANid].setDesiredPos(newPos);
        }
        //std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
    }

    devices[CANid].setActualPos(newPos);
    //devices[CANid].setActualVel(newVel);

    devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
    devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

}
void defaultPDO_incoming(uint16_t CANid, const TPCANRdMsg m)
{
    double newPos = mdeg2rad(m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24) );

    if (devices[CANid].getTimeStamp_msec() != std::chrono::milliseconds(0) || devices[CANid].getTimeStamp_usec() != std::chrono::microseconds(0))
    {
        auto deltaTime_msec = std::chrono::milliseconds(m.dwTime) - devices[CANid].getTimeStamp_msec();
        auto deltaTime_usec = std::chrono::microseconds(m.wUsec) - devices[CANid].getTimeStamp_usec();
        double deltaTime_double = static_cast<double>(deltaTime_msec.count()*1000 + deltaTime_usec.count()) * 0.000001;
        double result = (newPos - devices[CANid].getActualPos()) / deltaTime_double;
        devices[CANid].setActualVel(result);
        if (! devices[CANid].getInitialized())
        {
            devices[CANid].setDesiredPos(newPos);
        }
        //std::cout << "actualPos: " << devices[CANid].getActualPos() << "  desiredPos: " << devices[CANid].getDesiredPos() << std::endl;
    }


    devices[CANid].setActualPos(newPos);
    devices[CANid].setTimeStamp_msec(std::chrono::milliseconds(m.dwTime));
    devices[CANid].setTimeStamp_usec(std::chrono::microseconds(m.wUsec));

    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    bool ready_switch_on = mydata_low & 0x01;
    bool switched_on = mydata_low & 0x02;
    bool op_enable = mydata_low & 0x04;
    bool fault = mydata_low & 0x08;
    // std::cout << "fault PDO" << fault << std::endl;
    bool volt_enable = mydata_low & 0x10;
    bool quick_stop = mydata_low & 0x20;
    bool switch_on_disabled = mydata_low & 0x40;
    bool warning = mydata_low & 0x80;

    bool mode_specific = mydata_high & 0x01;
    bool remote = mydata_high & 0x02;
    bool target_reached = mydata_high & 0x04;
    bool internal_limit = mydata_high & 0x08;
    bool op_specific = mydata_high & 0x10;
    bool op_specific1 = mydata_high & 0x20;
    bool man_specific1 = mydata_high & 0x40;
    bool man_specific2 = mydata_high & 0x80;

    bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;


    if(!ready_switch_on)
    {
        if(fault)
        {
            devices[CANid].setMotorState(canopen::MS_FAULT);
        }
        else if(switch_on_disabled)
        {
            devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
        }
        else
            devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
    }

    else
    {
        if(switched_on)
        {
            if(op_enable)
            {

                //if(volt_enable)
                // {
                devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                // }

            }
            else
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
        }
        else if(!quick_stop)
            devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

        else
            devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

    }

    if(fault & op_enable & switched_on & ready_switch_on)
        devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);


    devices[CANid].setFault(fault);
    devices[CANid].setIPMode(ip_mode);
    devices[CANid].setHoming(op_specific);
    devices[CANid].setOpSpec0(op_specific);
    devices[CANid].setOpSpec1(op_specific1);
    devices[CANid].setManSpec1(man_specific1);
    devices[CANid].setManSpec2(man_specific2);
    devices[CANid].setInternalLimits(internal_limit);
    devices[CANid].setTargetReached(target_reached);
    devices[CANid].setRemote(remote);
    devices[CANid].setModeSpec(mode_specific);
    devices[CANid].setWarning(warning);
    devices[CANid].setSwitchOnDisable(switch_on_disabled);
    devices[CANid].setQuickStop(quick_stop);
    devices[CANid].setOpEnable(op_enable);
    devices[CANid].setVoltageEnabled(volt_enable);
    devices[CANid].setReadySwitchON(ready_switch_on);
    devices[CANid].setSwitchON(switched_on);

    // std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;


}

/***************************************************************/
//		define functions for receiving data
/***************************************************************/

void initListenerThread(std::function<void ()> const& listener)
{
    std::thread listener_thread(listener);
    listener_thread.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //std::cout << "Listener thread initialized" << std::endl;
}

void defaultListener()
{
    while(true)
    {
        //std::cout << "Reading incoming data" << std::endl;
        TPCANRdMsg m;
        errno = LINUX_CAN_Read(h, &m);
        if (errno)
            perror("LINUX_CAN_Read() error");

        // incoming SYNC
        else if (m.Msg.ID == 0x080)
        {
            // std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        // incoming EMCY
        //else if (m.Msg.ID >= 0x081 && m.Msg.ID <= 0x0FF)
        //{
         //   std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
          //  if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
           //     incomingEMCYHandlers[m.Msg.ID](m);
        //}

        // incoming TIME
        else if (m.Msg.ID == 0x100)
        {
            // std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        // incoming PD0
        else if (m.Msg.ID >= 0x180 && m.Msg.ID <= 0x4FF)
        {
            //std::cout << std::hex << "PDO received:  " << (m.Msg.ID - 0x180) << "  " << m.Msg.DATA[0] << " " << m.Msg.DATA[1] << " " << m.Msg.DATA[2] << " " << m.Msg.DATA[3] << " " << m.Msg.DATA[4] << " " << m.Msg.DATA[5] << " " << m.Msg.DATA[6] << " " <<  m.Msg.DATA[7] << " " << std::endl;
            //std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl;
            if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                incomingPDOHandlers[m.Msg.ID](m);
        }

        // incoming SD0
        else if (m.Msg.ID >= 0x580 && m.Msg.ID <= 0x5FF)
        {
            //std::cout << std::hex << "SDO received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
            SDOkey sdoKey(m);
            if(sdo_protect)
            {
                std::copy(std::begin(m.Msg.DATA), std::end(m.Msg.DATA), std::begin(protect_msg));
                sdo_protect = false;
            }
            else
            {
                if (incomingErrorHandlers.find(sdoKey) != incomingErrorHandlers.end())
                    incomingErrorHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
                else if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
                    incomingDataHandlers[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
                else if (incomingManufacturerDetails.find(sdoKey) != incomingManufacturerDetails.end())
                    incomingManufacturerDetails[sdoKey](m.Msg.ID - 0x580, m.Msg.DATA);
            }
        }

        // incoming NMT error control
        else if (m.Msg.ID >= 0x700 && m.Msg.ID <= 0x7FF)
        {
            uint16_t CANid = (uint16_t)(m.Msg.ID - 0x700);
            
            if (m.Msg.DATA[0] == 0x00)
            {          
                std::cout << "Bootup received. Node-ID =  " << CANid << std::endl;
                std::map<uint8_t,Device>::const_iterator search = devices.find(CANid);
                if(search != devices.end())
                {
                        std::cout << "Found " << (u_int16_t)search->first << "\n";
                        std::cout << "Initializing..." << "\n";
                        devices[CANid].setNMTInit(true);
                }
                else
                {
                       std::cout << "Not found" << std::endl;
                       std::cout << "Ignoring" << std::endl;
                }

            }
            else
            {
                std::cout << "NMT error control received:  " << (uint16_t)(m.Msg.ID - 0x700) << "  " << (uint16_t)m.Msg.DATA[0] << std::endl;
            }
        }
        else
        {
            //std::cout << "Received unknown message" << std::endl;
        }
    }
}

/******************************************************************************
 * Define get errors function
 *****************************************************************************/
void getErrors(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::ERRORWORD);
}

void manufacturer_incoming(uint8_t CANid, BYTE data[8])
{
    sdo_protect = true;

    if(data[1]+(data[2]<<8) == 0x1008)
    {
        std::vector<char> manufacturer_device_name = canopen::obtainManDevName(CANid, data[4]);

        devices[CANid].setManufacturerDevName(manufacturer_device_name);
    }
    /*
    else if(data[1]+(data[2]<<8) == 0x1009)
    {

    }
    */
}

void errorword_incoming(uint8_t CANid, BYTE data[8])
{
    std::stringstream str_stream;

    if(data[1]+(data[2]<<8) == 0x1001)
    {
        uint16_t error_register;
        error_register = data[4];

        str_stream << "error_register=0x" << std::hex << (int)error_register << ", categories:";

        if ( error_register & canopen::EMC_k_1001_GENERIC )
            str_stream << " generic,";
        if ( error_register & canopen::EMC_k_1001_CURRENT)
            str_stream << " current,";
        if ( error_register & canopen::EMC_k_1001_VOLTAGE )
            str_stream << " voltage,";
        if ( error_register & canopen::EMC_k_1001_TEMPERATURE )
            str_stream << " temperature,";
        if ( error_register & canopen::EMC_k_1001_COMMUNICATION )
            str_stream << " communication,";
        if ( error_register & canopen::EMC_k_1001_DEV_PROF_SPEC )
            str_stream << " device profile specific,";
        if ( error_register & canopen::EMC_k_1001_RESERVED )
            str_stream << " reserved,";
        if ( error_register & canopen::EMC_k_1001_MANUFACTURER)
            str_stream << " manufacturer specific";
        str_stream << "\n";

        devices[CANid].setErrorRegister(str_stream.str());
    }
    else if(data[1]+(data[2]<<8) == 0x1002)
    {
        uint16_t code = data[4];
        uint16_t classification = data[5];

        str_stream << "manufacturer_status_register=0x" << std::hex << int(classification) << int(code) <<
                     ": code=0x" << std::hex << int( code ) << " (" << errorsCode[int(code)] << "),"
                  << ", classification=0x" << std::hex << int( classification ) << std::dec;
        if ( classification == 0x88 )
            str_stream << " (CMD_ERROR)";
        if ( classification == 0x89 )
            str_stream << " (CMD_WARNING)";
        if ( classification == 0x8a )
            str_stream << " (CMD_INFO)";
        str_stream << "\n";

        devices[CANid].setManufacturerErrorRegister(str_stream.str());
    }
}

void readManErrReg(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::MANUFACTURER);
}

void readErrorsRegister(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::STATUSWORD);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m);

    canopen::uploadSDO(CANid, canopen::ERRORWORD);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen::processSingleSDO(CANid, m);

    uint16_t error_register;
    error_register = m->Msg.DATA[4];

    std::cout << "error_register=0x" << std::hex << (int)error_register << ", categories:";

    if ( error_register & canopen::EMC_k_1001_GENERIC )
        std::cout << " generic,";
    if ( error_register & canopen::EMC_k_1001_CURRENT)
        std::cout << " current,";
    if ( error_register & canopen::EMC_k_1001_VOLTAGE )
        std::cout << " voltage,";
    if ( error_register & canopen::EMC_k_1001_TEMPERATURE )
        std::cout << " temperature,";
    if ( error_register & canopen::EMC_k_1001_COMMUNICATION )
        std::cout << " communication,";
    if ( error_register & canopen::EMC_k_1001_DEV_PROF_SPEC )
        std::cout << " device profile specific,";
    if ( error_register & canopen::EMC_k_1001_RESERVED )
        std::cout << " reserved,";
    if ( error_register & canopen::EMC_k_1001_MANUFACTURER)
        std::cout << " manufacturer specific";
    std::cout << "\n";
}

std::vector<uint16_t> obtainVendorID(uint16_t CANid)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYVENDORID);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

std::vector<uint16_t> obtainProdCode(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYPRODUCTCODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<uint16_t> product_code;

    canopen::processSingleSDO(CANid, m);

    uint16_t id4 = m->Msg.DATA[4];
    uint16_t id3 = m->Msg.DATA[5];
    uint16_t id2 = m->Msg.DATA[6];
    uint16_t id1 = m->Msg.DATA[7];

    product_code.push_back(id1);
    product_code.push_back(id2);
    product_code.push_back(id3);
    product_code.push_back(id4);

    return product_code;

}

uint16_t obtainRevNr(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::IDENTITYREVNUMBER);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    canopen::processSingleSDO(CANid, m);

    uint16_t rev_number = m->Msg.DATA[4];

    return rev_number;

}

std::vector<char> obtainManDevName(uint16_t CANid, int size_name)
{

    std::vector<char> manufacturer_device_name;

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (auto it : protect_msg)
    {
        if(manufacturer_device_name.size() <= size_name)
            manufacturer_device_name.push_back(it);
    }

    return manufacturer_device_name;

}




std::vector<char> obtainManHWVersion(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    canopen::uploadSDO(CANid, canopen::MANUFACTURERHWVERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<char> manufacturer_hw_version;

    canopen::processSingleSDO(CANid, m);

    int size = m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_hw_version.size() <= size)
            manufacturer_hw_version.push_back(it);
    }

    return manufacturer_hw_version;
}

std::vector<char> obtainManSWVersion(uint16_t CANid, std::shared_ptr<TPCANRdMsg> m)
{
    std::vector<char> manufacturer_sw_version;

    canopen::uploadSDO(CANid, canopen::MANUFACTURERSOFTWAREVERSION);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);

    int size = (uint8_t)m->Msg.DATA[4];

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }


    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock1(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    canopen::requestDataBlock2(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::processSingleSDO(CANid, m);


    for (auto it : m->Msg.DATA)
    {
        if(manufacturer_sw_version.size() <= size)
            manufacturer_sw_version.push_back(it);
    }

    return manufacturer_sw_version;

}



void sdo_incoming(uint8_t CANid, BYTE data[8])
{

    if(data[1]+(data[2]<<8) == 0x6041)
    { //The incoming message is a result from a statusWord Request
        uint16_t mydata_low = data[4];
        uint16_t mydata_high = data[5];

        bool ready_switch_on = mydata_low & 0x01;
        bool switched_on = mydata_low & 0x02;
        bool op_enable = mydata_low & 0x04;
        bool fault = mydata_low & 0x08;
        bool volt_enable = mydata_low & 0x10;
        bool quick_stop = mydata_low & 0x20;
        bool switch_on_disabled = mydata_low & 0x40;
        bool warning = mydata_low & 0x80;

        bool mode_specific = mydata_high & 0x01;
        bool remote = mydata_high & 0x02;
        bool target_reached = mydata_high & 0x04;
        bool internal_limit = mydata_high & 0x08;
        bool op_specific = mydata_high & 0x10;
        bool op_specific1 = mydata_high & 0x20;
        bool man_specific1 = mydata_high & 0x40;
        bool man_specific2 = mydata_high & 0x80;


        bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;


        if(!ready_switch_on)
        {
            if(fault)
            {
                devices[CANid].setMotorState(canopen::MS_FAULT);
            }
            else if(switch_on_disabled)
            {
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
            }
            else
                devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
        }

        else
        {
            if(switched_on)
            {
                if(op_enable)
                {

                    //if(volt_enable)
                    // {
                    devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                    // }

                }
                else
                    devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
            }
            else if(!quick_stop)
                devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

            else
                devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

        }

        if(fault & op_enable & switched_on & ready_switch_on)
            devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);



        devices[CANid].setFault(fault);
        devices[CANid].setHoming(op_specific);
        devices[CANid].setOpSpec0(op_specific);
        devices[CANid].setOpSpec1(op_specific1);
        devices[CANid].setManSpec1(man_specific1);
        devices[CANid].setManSpec2(man_specific2);
        devices[CANid].setInternalLimits(internal_limit);
        devices[CANid].setTargetReached(target_reached);
        devices[CANid].setRemote(remote);
        devices[CANid].setModeSpec(mode_specific);
        devices[CANid].setWarning(warning);
        devices[CANid].setSwitchOnDisable(switch_on_disabled);
        devices[CANid].setQuickStop(quick_stop);
        devices[CANid].setOpEnable(op_enable);
        devices[CANid].setVoltageEnabled(volt_enable);
        devices[CANid].setReadySwitchON(ready_switch_on);
        devices[CANid].setSwitchON(switched_on);

        //std::cout << "Motor State of Device with CANid " << (uint16_t)CANid << " is: " << devices[CANid].getMotorState() << std::endl;
    }

    else if(data[1]+(data[2]<<8) == 0x22A2)
    { //This is a result from a temperature register request
        devices[CANid].setDriverTemperature(data[4]);
    }

    else if(data[1]+(data[2]<<8) == 0x6061)
    { //Incoming message is a mode of operation display
         devices[CANid].setCurrentModeofOperation(data[4]);
    }

}

void processSingleSDO(uint8_t CANid, std::shared_ptr<TPCANRdMsg> message)
{
    message->Msg.ID = 0x00;

    while (message->Msg.ID != (0x580+CANid))
    {
        LINUX_CAN_Read(canopen::h, message.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void pdoChanged()
{
    for (auto device : devices)
    {
        TPCANMsg* mes;
        //////////////////// Enable tpdo4
        mes->ID =device.second.getCANid() + 0x600;
        mes->MSGTYPE = 0x00;
        mes->LEN = 8;
        mes->DATA[0] = 0x2F;
        mes->DATA[1] = 0x20;
        mes->DATA[2] = 0x2F;
        mes->DATA[3] = 0x04;
        mes->DATA[4] = 0x00;
        mes->DATA[5] = 0x00;
        mes->DATA[6] = 0x00;
        mes->DATA[7] = 0x01;
        CAN_Write(canopen::h, mes);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void disableRPDO(int object)
{
    for (auto device : devices)
    {
        if(object == 0)
        {
            int32_t data = (canopen::RPDO1_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);

        }

        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }

        else if(object == 3)

        {
            int32_t data = (canopen::RPDO4_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////

    }
}

void setObjects()
{
    for (auto device : devices)
    {
        int32_t data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x6081,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x607f,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x6083,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x60c5,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x60c6,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(device.second.getCANid(), SDOkey(0x6082,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

void clearRPDOMapping(int object)
{
    for (auto device : devices)
    {
        int32_t data = (0x00 << 16) + (0x80 << 24);

        sendSDO_unknown(device.second.getCANid(), SDOkey(RPDO_map.index+object,0x00), data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void makeRPDOMapping(int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type)
{
    for (auto device : devices)
    {
        int ext_counter=0;
        for(int counter=0; counter < registers.size();counter++)
        {
            /////////////////////////
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            str_stream.str( std::string() );
            str_stream.clear();

            /////////////////////////
            /// \brief data
            ///
            int32_t data = (sizes[counter]) + (index_data << 8);

            sendSDO(device.second.getCANid(), SDOkey(RPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(device.second.getCANid(), SDOkey(RPDO_map.index+object,0x00), u_int8_t(ext_counter));

    }
}

void enableRPDO(int object)
{
    for (auto device : devices)
    {
        if(object ==0)
        {
            int32_t data = (canopen::RPDO1_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::RPDO4_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////
    }
}


/*****************************
 *
 * Mapping for PDO1
 **/

void disableTPDO(int object)
{

    for(auto device : devices)
    {

        //////////////////// Disable tpdo4

        if(object == 0)
        {
            int32_t data = (canopen::TPDO1_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::TPDO2_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);

        }

        else if(object == 2)
        {
            int32_t data = (canopen::TPDO3_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }

        else if(object == 3)

        {
            int32_t data = (canopen::TPDO4_msg + device.second.getCANid())  + (0x00 << 16) + (0x80 << 24);
            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }

        else
            std::cout << "Incorrect object for mapping" << std::endl;



        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////
    }


}

void clearTPDOMapping(int object)
{
    for (auto device : devices)
    {
        //////////////////// clear mapping
        ///
        //int32_t data = (0x00 << 16) + (0x00 << 24);
        sendSDO(device.second.getCANid(), SDOkey(TPDO_map.index+object,0x00), u_int8_t(0x00));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void makeTPDOMapping(int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type)
{
    for (auto device : devices)
    {
        //////////////////// sub ind1=63
        ///
        ///
        ///
        int ext_counter=0;
        for(int counter=0; counter < registers.size();counter++)
        {
            /////////////////////////
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            str_stream.str( std::string() );
            str_stream.clear();

            /////////////////////////
            /// \brief data
            ///
            int32_t data = (sizes[counter]) + (index_data << 8);

            sendSDO(device.second.getCANid(), SDOkey(TPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(device.second.getCANid(), SDOkey(TPDO_map.index+object,0x00), u_int8_t(ext_counter));
    }

}

void enableTPDO(int object)
{
    for (auto device : devices)
    {
        //////////////////// Enable tpdo4
        ///
        ///
        if(object ==0)
        {
            int32_t data = (canopen::TPDO1_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::TPDO2_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::TPDO3_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::TPDO4_msg + device.second.getCANid()) + (0x00 << 16) + (0x00 << 24);

            sendSDO(device.second.getCANid(), SDOkey(TPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    /////////////////////////
}




}
