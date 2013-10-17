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
 *   Moves the motor to an specific position using the Elmo controller
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
#include "canopen.h"
#include <sstream>

int main(int argc, char *argv[])
{

    if (argc != 3) {
        std::cout << "Arguments:" << std::endl
        << "(1) device file" << std::endl
        << "(2) CAN deviceID" << std::endl
        << "Example: ./elmo_test /dev/pcan32 12" << std::endl;
        return -1;
    }



    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    std::string deviceFile = std::string(argv[1]);

    if (!canopen::openConnection(deviceFile)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    uint16_t CANid = std::stoi(std::string(argv[2]));

    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //canopen::sendNMT(0, canopen::NMT_RESET_NODE);
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    TPCANMsg m;

 //////////////////// pp mode
    m.ID = CANid + 0x600;//CANid + CANid + 0x600;
    m.MSGTYPE = 0x00;
    m.LEN = 8;
    m.DATA[0] = 0x22;
    m.DATA[1] = 0x60;
    m.DATA[2] = 0x60;
    m.DATA[3] = 0x00;
    m.DATA[4] = 0x01;
    m.DATA[5] = 0x00;
    m.DATA[6] = 0x00;
    m.DATA[7] = 0x00;
    CAN_Write(canopen::h, &m);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    /////////////////////////

    //////////////////// get mode display
       m.ID = CANid + 0x600;//CANid + CANid + 0x600;
       m.MSGTYPE = 0x00;
       m.LEN = 8;
       m.DATA[0] = 0x40;
       m.DATA[1] = 0x61;
       m.DATA[2] = 0x60;
       m.DATA[3] = 0x00;
       m.DATA[4] = 0x01;
       m.DATA[5] = 0x00;
       m.DATA[6] = 0x00;
       m.DATA[7] = 0x00;
       CAN_Write(canopen::h, &m);

       std::this_thread::sleep_for(std::chrono::milliseconds(10));


       /////////////////////////

       //////////////////// get mode display
          m.ID = CANid + 0x600;//CANid + CANid + 0x600;
          m.MSGTYPE = 0x00;
          m.LEN = 8;
          m.DATA[0] = 0x40;
          m.DATA[1] = 0x61;
          m.DATA[2] = 0x60;
          m.DATA[3] = 0x00;
          m.DATA[4] = 0x01;
          m.DATA[5] = 0x00;
          m.DATA[6] = 0x00;
          m.DATA[7] = 0x00;
          CAN_Write(canopen::h, &m);

          std::this_thread::sleep_for(std::chrono::milliseconds(10));


          /////////////////////////

          //////////////////// Ready to switch on
             m.ID = CANid + 0x600;//CANid + CANid + 0x600;
             m.MSGTYPE = 0x00;
             m.LEN = 8;
             m.DATA[0] = 0x22;
             m.DATA[1] = 0x40;
             m.DATA[2] = 0x60;
             m.DATA[3] = 0x00;
             m.DATA[4] = 0x06;
             m.DATA[5] = 0x00;
             m.DATA[6] = 0x00;
             m.DATA[7] = 0x00;
             CAN_Write(canopen::h, &m);

             std::this_thread::sleep_for(std::chrono::milliseconds(10));


             /////////////////////////

             //////////////////// Read status
                m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                m.MSGTYPE = 0x00;
                m.LEN = 8;
                m.DATA[0] = 0x40;
                m.DATA[1] = 0x41;
                m.DATA[2] = 0x60;
                m.DATA[3] = 0x00;
                m.DATA[4] = 0x00;
                m.DATA[5] = 0x00;
                m.DATA[6] = 0x00;
                m.DATA[7] = 0x00;
                CAN_Write(canopen::h, &m);

                std::this_thread::sleep_for(std::chrono::milliseconds(10));


                /////////////////////////

                //////////////////// Switch on
                   m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                   m.MSGTYPE = 0x00;
                   m.LEN = 8;
                   m.DATA[0] = 0x22;
                   m.DATA[1] = 0x40;
                   m.DATA[2] = 0x60;
                   m.DATA[3] = 0x00;
                   m.DATA[4] = 0x07;
                   m.DATA[5] = 0x00;
                   m.DATA[6] = 0x00;
                   m.DATA[7] = 0x00;
                   CAN_Write(canopen::h, &m);

                   std::this_thread::sleep_for(std::chrono::milliseconds(10));


                   /////////////////////////

                   //////////////////// Read status
                      m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                      m.MSGTYPE = 0x00;
                      m.LEN = 8;
                      m.DATA[0] = 0x40;
                      m.DATA[1] = 0x41;
                      m.DATA[2] = 0x60;
                      m.DATA[3] = 0x00;
                      m.DATA[4] = 0x00;
                      m.DATA[5] = 0x00;
                      m.DATA[6] = 0x00;
                      m.DATA[7] = 0x00;
                      CAN_Write(canopen::h, &m);

                      std::this_thread::sleep_for(std::chrono::milliseconds(10));


                      /////////////////////////

                      //////////////////// Start mo=1
                         m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                         m.MSGTYPE = 0x00;
                         m.LEN = 8;
                         m.DATA[0] = 0x22;
                         m.DATA[1] = 0x40;
                         m.DATA[2] = 0x60;
                         m.DATA[3] = 0x00;
                         m.DATA[4] = 0x0f;
                         m.DATA[5] = 0x00;
                         m.DATA[6] = 0x00;
                         m.DATA[7] = 0x00;
                         CAN_Write(canopen::h, &m);

                         std::this_thread::sleep_for(std::chrono::milliseconds(10));


                         /////////////////////////

                         //////////////////// Read status
                            m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                            m.MSGTYPE = 0x00;
                            m.LEN = 8;
                            m.DATA[0] = 0x40;
                            m.DATA[1] = 0x41;
                            m.DATA[2] = 0x60;
                            m.DATA[3] = 0x00;
                            m.DATA[4] = 0x00;
                            m.DATA[5] = 0x00;
                            m.DATA[6] = 0x00;
                            m.DATA[7] = 0x00;
                            CAN_Write(canopen::h, &m);

                            std::this_thread::sleep_for(std::chrono::milliseconds(10));


                            /////////////////////////


                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                            //////////////////// Set speed(10000)
                               m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                               m.MSGTYPE = 0x00;
                               m.LEN = 8;
                               m.DATA[0] = 0x22;
                               m.DATA[1] = 0x81;
                               m.DATA[2] = 0x60;
                               m.DATA[3] = 0x00;
                               m.DATA[4] = 0x10;
                               m.DATA[5] = 0x27;
                               m.DATA[6] = 0x00;
                               m.DATA[7] = 0x00;
                               CAN_Write(canopen::h, &m);

                               std::this_thread::sleep_for(std::chrono::milliseconds(10));


                               /////////////////////////

                               //////////////////// Set target position(166890)
                                  m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                                  m.MSGTYPE = 0x00;
                                  m.LEN = 8;
                                  m.DATA[0] = 0x22;
                                  m.DATA[1] = 0x7a;
                                  m.DATA[2] = 0x60;
                                  m.DATA[3] = 0x00;
                                  m.DATA[4] = 0xea;
                                  m.DATA[5] = 0x8b;
                                  m.DATA[6] = 0x02;
                                  m.DATA[7] = 0x00;
                                  CAN_Write(canopen::h, &m);

                                  std::this_thread::sleep_for(std::chrono::milliseconds(10));


                                  /////////////////////////



                                     m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                                     m.MSGTYPE = 0x00;
                                     m.LEN = 8;
                                     m.DATA[0] = 0x22;
                                     m.DATA[1] = 0x40;
                                     m.DATA[2] = 0x60;
                                     m.DATA[3] = 0x00;
                                     m.DATA[4] = 0x1f;
                                     m.DATA[5] = 0x00;
                                     m.DATA[6] = 0x00;
                                     m.DATA[7] = 0x00;
                                     CAN_Write(canopen::h, &m);

                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));


                                     /////////////////////////
                                     //////////////////// Set point absolute
                                        m.ID = CANid + 0x600;//CANid + CANid + 0x600;
                                        m.MSGTYPE = 0x00;
                                        m.LEN = 8;
                                        m.DATA[0] = 0x22;
                                        m.DATA[1] = 0x40;
                                        m.DATA[2] = 0x60;
                                        m.DATA[3] = 0x00;
                                        m.DATA[4] = 0x0f;
                                        m.DATA[5] = 0x00;
                                        m.DATA[6] = 0x00;
                                        m.DATA[7] = 0x00;
                                        CAN_Write(canopen::h, &m);

                                        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

                                        //////////////////// Ready to switch on
             m.ID = CANid + 0x600;//CANid + CANid + 0x600;
             m.MSGTYPE = 0x00;
             m.LEN = 8;
             m.DATA[0] = 0x22;
             m.DATA[1] = 0x40;
             m.DATA[2] = 0x60;
             m.DATA[3] = 0x00;
             m.DATA[4] = 0x06;
             m.DATA[5] = 0x00;
             m.DATA[6] = 0x00;
             m.DATA[7] = 0x00;
             CAN_Write(canopen::h, &m);

             std::this_thread::sleep_for(std::chrono::milliseconds(10));

              canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen:: CONTROLWORD_HALT);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                canopen::sendSDO(CANid, canopen::CONTROLWORD, canopen:: CONTROL_WORD_DISABLE_VOLTAGE);

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                                     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}
