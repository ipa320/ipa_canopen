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
#include<algorithm>


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
std::vector<std::thread> managerThreads;
std::vector<std::string> openDeviceFiles;
bool atFirstInit=true;
int initTrials=0;
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

std::string operation_mode_param;

std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

std::chrono::duration<double> elapsed_seconds;


/***************************************************************/
//		define init and recover sequence
/***************************************************************/

bool openConnection(std::string devName, std::string baudrate)
{
    h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
    if (!h)
        return false;

    errno = CAN_Init(h, baudrates[baudrate], CAN_INIT_TYPE_ST);

    return true;
}

void pre_init(std::string chainName)
{

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        /*********************************************/

        getErrors(id);

        /***************************************************************/
        //		Manufacturer specific errors register
        /***************************************************************/
        readManErrReg(id);

        /**************************
       * Hardware and Software Information
      *************************/
        canopen::uploadSDO(id, MANUFACTURERDEVICENAME);
    }
}

void pdo_map(std::string chain_name, int pdo_id,
         std::vector<std::string> tpdo_registers, std::vector<int> tpdo_sizes, u_int8_t tsync_type,
         std::vector<std::string> rpdo_registers, std::vector<int> rpdo_sizes, u_int8_t rsync_type)
{
    // clear all mappings for given pdo id
    canopen::disableTPDO(chain_name, pdo_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    canopen::clearTPDOMapping(chain_name, pdo_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    canopen::disableRPDO(chain_name, pdo_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    canopen::clearRPDOMapping(chain_name, pdo_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if(!tpdo_registers.empty())
    {
        canopen::makeTPDOMapping(chain_name, pdo_id-1, tpdo_registers, tpdo_sizes, tsync_type);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        canopen::enableTPDO(chain_name, pdo_id-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(!rpdo_registers.empty())
    {
        canopen::makeRPDOMapping(chain_name,pdo_id-1, rpdo_registers, rpdo_sizes, rsync_type);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        canopen::enableRPDO(chain_name, pdo_id-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

//////////////////////////////////////
/////////////
bool init(std::string deviceFile, std::string chainName, const int8_t mode_of_operation)
{
    initTrials++;

    if(initTrials == 4)
    {
        std::cout << "There are still problems with the devices. Trying a complete reset " << std::endl;
        canopen::sendNMT(0x00, canopen::NMT_RESET_NODE);

        initTrials=0;
    }

    if(canopen::atFirstInit)
    {

        canopen::atFirstInit = false;

        bool connection_success;

        bool connection_is_available = std::find(canopen::openDeviceFiles.begin(), canopen::openDeviceFiles.end(), deviceFile) != canopen::openDeviceFiles.end();

        if(!connection_is_available)
        {

            CAN_Close(canopen::h);
            connection_success = canopen::openConnection(deviceFile, canopen::baudRate);

            if (!connection_success)
            {
                std::cout << "Cannot open CAN device "<< deviceFile << "; aborting." << std::endl;
                exit(EXIT_FAILURE);
            }
            canopen::initListenerThread(canopen::defaultListener);
            canopen::openDeviceFiles.push_back(deviceFile);
        }

        std::cout << "Resetting communication with the devices " << std::endl;
        canopen::sendNMT(0x00, canopen::NMT_RESET_COMMUNICATION);

    }


    if(canopen::deviceGroups[chainName].getFirstInit())
    {

        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
        time_start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds;
        
        canopen::initDeviceManagerThread(chainName,canopen::deviceManager);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "Initializing " << chainName << std::endl;

        canopen::deviceGroups[chainName].setFirstInit(false);

        canopen::pre_init(chainName);

        while(sdo_protect)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
         elapsed_seconds = time_end - time_start;

            if(elapsed_seconds.count() > 5.0)
                {
                    std::cout << "not ready for operation. Probably due to communication problems with the Master." << std::endl;
                    return false;
                }
                time_end = std::chrono::high_resolution_clock::now();
        }

        time_start = std::chrono::high_resolution_clock::now();

        
        
        for(auto id : canopen::deviceGroups[chainName].getCANids())
        {

            bool nmt_init = devices[id].getNMTInit();
            std::cout << "Waiting for Node: " << (uint16_t)id << " to become available" << std::endl;


            while(!nmt_init)
            {
                elapsed_seconds = time_end - time_start;

                if(elapsed_seconds.count() > 25.0)
                {
                    std::cout << "Node: " << (uint16_t)id << " is not ready for operation. Please check for eventual problems." << std::endl;
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                nmt_init = devices[id].getNMTInit();
                time_end = std::chrono::high_resolution_clock::now();
            }

            std::cout << "Node: " << (uint16_t)id << " is now available" << std::endl;

            canopen::sendNMT((u_int8_t)id, canopen::NMT_START_REMOTE_NODE);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Configure PDO channels
        for (auto id : canopen::deviceGroups[chainName].getCANids())
        {
            for (int pdo_channel = 1; pdo_channel <= 4; ++pdo_channel)
            {
                std::vector<std::string> tpdo_registers, rpdo_registers;
                std::vector<int> tpdo_sizes, rpdo_sizes;
                u_int8_t tsync_type, rsync_type;

                switch(pdo_channel)
                {
                    case 1:
                        // Status word
                        tpdo_registers.push_back("604100");
                        tpdo_sizes.push_back(0x10);

                        // Mode of operation display
                        tpdo_registers.push_back("606100");
                        tpdo_sizes.push_back(0x08);

                        if(canopen::use_limit_switch)
                        {
                            // Digital Inputs
                            tpdo_registers.push_back("60FD00");
                            tpdo_sizes.push_back(0x20);
                        }

                        // Control word
                        rpdo_registers.push_back("604000");
                        rpdo_sizes.push_back(0x10);

                        tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                    case 2:
                        // Position Actual Value
                        tpdo_registers.push_back("606400");
                        tpdo_sizes.push_back(0x20);

                        // Velocity Actual Value
                        tpdo_registers.push_back("606C00");
                        tpdo_sizes.push_back(0x20);

                        // Position Target Value
                        switch(mode_of_operation)
                        {
                            case MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE:
                                rpdo_registers.push_back("60C101");
                                rpdo_sizes.push_back(0x20);
                                rsync_type = SYNC_TYPE_CYCLIC;
                                break;
                            case MODES_OF_OPERATION_PROFILE_POSITION_MODE:
                                rpdo_registers.push_back("607A00");
                                rpdo_sizes.push_back(0x20);
                                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                                break;
                        }

                        tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    default:
                        std::cout << "ERROR: There are only 4 PDO channels" << std::endl;
                        break;
                }
                pdo_map(chainName, pdo_channel, tpdo_registers, tpdo_sizes, tsync_type, rpdo_registers, rpdo_sizes, rsync_type);
            }
            std::cout << "Initialized the PDO mapping for Node:" << (uint16_t)id << std::endl;
        }
    }
    recover_active = false;

    canopen::setObjects(chainName);


    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        bool set_operation_mode = canopen::setOperationMode(id, mode_of_operation);
        if(!set_operation_mode)
            return false;
        canopen::setMotorState((uint16_t)id, canopen::MS_OPERATION_ENABLED);

        //Necessary otherwise sometimes Schunk devices complain for Position Track Error
        canopen::devices[id].setDesiredPos((double)devices[id].getActualPos());
        canopen::devices[id].setDesiredVel(0);

        sendPos((uint16_t)id, (double)devices[id].getDesiredPos());

        canopen::controlPDO(id, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        getErrors(id);
        readManErrReg(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if(devices[id].getIPMode())
        {
            std::cout << "Concluded driver side init succesfully for Node" << (uint16_t)id << std::endl;
            canopen::devices[id].setInitialized(true);
        }
        else
        {
            std::cout << "Problems occured during driver side init for Node" << (uint16_t)id  << std::endl;
            canopen::devices[id].setInitialized(false);
            return false;
        }

    }

    return true;
}

bool init(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
{
    bool initialized = init(deviceFile, chainName, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        sendSDO((uint16_t)id, canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sendSDO((uint16_t)id, canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sendSDO((uint16_t)id, canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return initialized;
}


bool recover(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
{

    recover_active = true;

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {

        if(devices[id].getIPMode())
        {
            std::cout << "Node" << id << "is already operational" << std::endl;
        }
        else
        {

            canopen::controlPDO(id,canopen::CONTROLWORD_HALT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROLWORD_DISABLE_INTERPOLATED, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROL_WORD_DISABLE_VOLTAGE, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id,canopen::CONTROLWORD_QUICKSTOP, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::sendSDO(id, canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(id, canopen::MS_SWITCHED_ON_DISABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(id, canopen::MS_READY_TO_SWITCH_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(id, canopen::MS_SWITCHED_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::setMotorState(id, canopen::MS_OPERATION_ENABLED);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            sendSDO((uint16_t)id, canopen::IP_TIME_UNITS, (uint8_t) syncInterval.count() );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)id, canopen::IP_TIME_INDEX, (uint8_t)canopen::IP_TIME_INDEX_MILLISECONDS);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sendSDO((uint16_t)id, canopen::SYNC_TIMEOUT_FACTOR, (uint8_t)canopen::SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            canopen::controlPDO(id, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            canopen::uploadSDO(id, canopen::STATUSWORD);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canopen::uploadSDO(id, DRIVERTEMPERATURE);
            canopen::uploadSDO(id, MODES_OF_OPERATION_DISPLAY);

            getErrors(id);
        }


        devices[id].setDesiredPos((double)devices[id].getActualPos());
        devices[id].setDesiredVel(0);

    }
    recover_active = false;

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {

        if(devices[id].getIPMode())
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
///////////////////////////////7
///


void halt(std::string deviceFile, std::string chainName, std::chrono::milliseconds syncInterval)
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


    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //std::cout << "Module with CAN-id " << (uint16_t)id << " connected (recover)" << std::endl;
    }

    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {


        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROLWORD_HALT);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROLWORD_DISABLE_INTERPOLATED);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(id, canopen::CONTROLWORD, canopen:: CONTROL_WORD_DISABLE_VOLTAGE);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::sendSDO(id, canopen::CONTROLWORD, canopen::CONTROLWORD_QUICKSTOP);
        canopen::uploadSDO(id, canopen::STATUSWORD);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

/***************************************************************/
//		define state machine functions
/***************************************************************/

void setNMTState(uint16_t CANid, std::string targetState)
{

}

bool setOperationMode(uint16_t CANid, const int8_t targetMode, double timeout)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;

    time_start = std::chrono::high_resolution_clock::now();

    // check if motor is in a legitimate state to change operation mode
    if (    devices[CANid].getMotorState() != MS_READY_TO_SWITCH_ON &&
            devices[CANid].getMotorState() != MS_SWITCHED_ON_DISABLED &&
            devices[CANid].getMotorState() != MS_SWITCHED_ON)
    {
        std::cout << "Found motor in state " << devices[CANid].getMotorState() << ", need to adjust state to SWITCHED_ON" << std::endl;
        setMotorState(CANid, canopen::MS_SWITCHED_ON);
    }

    // change operation mode until correct mode is returned
    while (devices[CANid].getCurrentModeofOperation() != targetMode)
    {
        canopen::sendSDO(CANid, canopen::MODES_OF_OPERATION, (uint8_t)targetMode);
        canopen::uploadSDO(CANid, canopen::MODES_OF_OPERATION_DISPLAY);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // timeout check
        time_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = time_end-time_start;

        if(elapsed_seconds.count() > timeout)
        {
            std::cout << "setting operation mode failed" << std::endl;
            return false;
        }
    }

    return true;
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
            canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN, 0x00);
        }

        if (devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
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
            else
            {
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
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

void initDeviceManagerThread(std::string chainName, std::function<void (std::string)> const& deviceManager)
{
    std::thread device_manager_thread(deviceManager, chainName);
    device_manager_thread.detach();
    //managerThreads.push_back(device_manager_thread);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void deviceManager(std::string chainName)
{
    std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;


    while (true)
    {
        time_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = time_end-time_start;

        auto tic = std::chrono::high_resolution_clock::now();
        if (!recover_active)
        {
            for (auto id : canopen::deviceGroups[chainName].getCANids())
            {
                if(elapsed_seconds.count() > 2)
                {
                    time_start = std::chrono::high_resolution_clock::now();
                    canopen::uploadSDO(id, DRIVERTEMPERATURE);
                    getErrors(id);
                    readManErrReg(id);
                }

                if (devices[id].getInitialized())
                {
                    devices[id].updateDesiredPos();
                    sendPos((uint16_t)id, (double)devices[id].getDesiredPos());
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
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = COB_PDO2_RX + CANid;
    msg.MSGTYPE = 0x00;
    msg.LEN = 4;
    int32_t mdegPos = rad2mdeg(positionValue);
    msg.DATA[0] = mdegPos & 0xFF;
    msg.DATA[1] = (mdegPos >> 8) & 0xFF;
    msg.DATA[2] = (mdegPos >> 16) & 0xFF;
    msg.DATA[3] = (mdegPos >> 24) & 0xFF;
    CAN_Write(h, &msg);
}

void defaultEMCY_incoming(uint16_t CANid, const TPCANRdMsg m)
{


    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    //std::cout << "EMCY" << (uint16_t)CANid << " is: " << (uint16_t)m.Msg.DATA[0] << " "<< (uint16_t)m.Msg.DATA[1]<< " " << (uint16_t)m.Msg.DATA[2]<< " "<< (uint16_t)m.Msg.DATA[3]<< " "<< (uint16_t)m.Msg.DATA[4]<< " "<< (uint16_t)m.Msg.DATA[5]<< " "<< (uint16_t)m.Msg.DATA[6]<< " "<< (uint16_t)m.Msg.DATA[7]<< " "<< (uint16_t)m.Msg.DATA[8]<< std::endl;


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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        else if (m.Msg.ID == COB_SYNC)
        {
            // std::cout << std::hex << "SYNC received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        // incoming EMCY
        else if (m.Msg.ID >= COB_EMERGENCY && m.Msg.ID < COB_TIME_STAMP)
        {
        //   std::cout << std::hex << "EMCY received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
          if (incomingEMCYHandlers.find(m.Msg.ID) != incomingEMCYHandlers.end())
             incomingEMCYHandlers[m.Msg.ID](m);
        }

        // incoming TIME
        else if (m.Msg.ID == COB_TIME_STAMP)
        {
            // std::cout << std::hex << "TIME received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " << (uint16_t)m.Msg.DATA[7] << std::endl;
        }

        // incoming PD0
        else if (m.Msg.ID >= COB_PDO1_TX && m.Msg.ID < COB_PDO4_RX)
        {
            //std::cout << std::hex << "PDO received:  " << (m.Msg.ID - 0x180) << "  " << m.Msg.DATA[0] << " " << m.Msg.DATA[1] << " " << m.Msg.DATA[2] << " " << m.Msg.DATA[3] << " " << m.Msg.DATA[4] << " " << m.Msg.DATA[5] << " " << m.Msg.DATA[6] << " " <<  m.Msg.DATA[7] << " " << std::endl;
            //std::cout << std::hex << "PDO received:  " << (uint16_t)(m.Msg.ID - 0x180) << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << " " << (uint16_t)m.Msg.DATA[2] << " " << (uint16_t)m.Msg.DATA[3] << " " << (uint16_t)m.Msg.DATA[4] << " " << (uint16_t)m.Msg.DATA[5] << " " << (uint16_t)m.Msg.DATA[6] << " " <<  (uint16_t)m.Msg.DATA[7] << " " << std::endl;
            if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                incomingPDOHandlers[m.Msg.ID](m);
        }

        // incoming SD0
        else if (m.Msg.ID >= COB_SDO_TX && m.Msg.ID < COB_NODEGUARD)
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
                    incomingErrorHandlers[sdoKey](m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
                else if (incomingDataHandlers.find(sdoKey) != incomingDataHandlers.end())
                    incomingDataHandlers[sdoKey](m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
                else if (incomingManufacturerDetails.find(sdoKey) != incomingManufacturerDetails.end())
                    incomingManufacturerDetails[sdoKey](m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
            }
        }

        // incoming NMT error control
        else if (m.Msg.ID >= COB_NODEGUARD && m.Msg.ID < COB_MAX)
        {
            //std::cout << std::hex << "NMT received:  " << (uint16_t)m.Msg.ID << "  " << (uint16_t)m.Msg.DATA[0] << " " << (uint16_t)m.Msg.DATA[1] << std::endl;
            uint16_t CANid = (uint16_t)(m.Msg.ID - COB_NODEGUARD);
            
            std::cout << "Bootup received. Node-ID =  " << CANid << std::endl;
            std::map<uint8_t,Device>::const_iterator search = devices.find(CANid);
            if(search != devices.end())
            {
                std::cout << "Found " << (u_int16_t)search->first << "\n";
                devices[CANid].setNMTInit(true);
            }
            else
            {
                std::cout << "Node:" << CANid << " could not be found on the required devices list." << std::endl;
                std::cout << "Ignoring" << std::endl;
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
    uint16_t SDOid = data[1]+(data[2]<<8);

    if(SDOid == STATUSWORD.index) //The incoming message is a result from a statusWord Request
    {
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
    else if(SDOid == DRIVERTEMPERATURE.index) //This is a result from a temperature register request
    {
        devices[CANid].setDriverTemperature(data[4]);
    }
    else if(SDOid == MODES_OF_OPERATION_DISPLAY.index) //Incoming message is a mode of operation display
    {
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

void pdoChanged(std::string chainName)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        TPCANMsg* mes;
        //////////////////// Enable tpdo4
        mes->ID =id + 0x600;
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

void disableRPDO(std::string chainName, int object)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        if(object == 0)
        {
            int32_t data = (canopen::RPDO1_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);

        }

        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        else if(object == 3)

        {
            int32_t data = (canopen::RPDO4_msg + id)  + (0x00 << 16) + (0x80 << 24);
            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////

    }
}

void setObjects(std::string chainName)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        int32_t data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6081,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x607f,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6083,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x60c5,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x60c6,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data = 0x1388 + (0x00 << 16) + (0x00 << 24);
        sendSDO_unknown(id, SDOkey(0x6082,0x00), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

void clearRPDOMapping(std::string chainName, int object)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        int32_t data = (0x00 << 16) + (0x80 << 24);

        sendSDO_unknown(id, SDOkey(RPDO_map.index+object,0x00), data);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void makeRPDOMapping(std::string chainName, int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
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

            sendSDO(id, SDOkey(RPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(id, SDOkey(RPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(id, SDOkey(RPDO_map.index+object,0x00), u_int8_t(ext_counter));

    }
}

void enableRPDO(std::string chainName, int object)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        if(object ==0)
        {
            int32_t data = (canopen::RPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::RPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::RPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::RPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /////////////////////////
    }
}


/*****************************
 *
 * Mapping for PDO1
 **/

void disableTPDO(std::string chainName,int object)
{
    int32_t data;
    for(auto id : canopen::deviceGroups[chainName].getCANids())
    {
        switch(object)
        {
            case 0:
                data = (canopen::TPDO1_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 1:
                data = (canopen::TPDO2_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 2:
                data = (canopen::TPDO3_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 3:
                data = (canopen::TPDO4_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            default:
                std::cout << "Incorrect object for mapping" << std::endl;
                return;
        }
        sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void clearTPDOMapping(std::string chainName, int object)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //////////////////// clear mapping
        ///
        //int32_t data = (0x00 << 16) + (0x00 << 24);
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(0x00));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void makeTPDOMapping(std::string chainName, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
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

            sendSDO(id, SDOkey(TPDO_map.index+object,counter+1), data);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ext_counter++;
        }
        /////////////////////////
        //////////////////// ASync

        sendSDO(id, SDOkey(TPDO.index+object,0x02), u_int8_t(sync_type));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //////////////////////
        ///
        ///
        /////////////////////// Mapping x objects
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), u_int8_t(ext_counter));
    }

}

void enableTPDO(std::string chainName, int object)
{
    for (auto id : canopen::deviceGroups[chainName].getCANids())
    {
        //////////////////// Enable tpdo4
        ///
        ///
        if(object ==0)
        {
            int32_t data = (canopen::TPDO1_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 1)
        {
            int32_t data = (canopen::TPDO2_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 2)
        {
            int32_t data = (canopen::TPDO3_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }
        else if(object == 3)
        {
            int32_t data = (canopen::TPDO4_msg + id) + (0x00 << 16) + (0x00 << 24);

            sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    /////////////////////////
}




}

