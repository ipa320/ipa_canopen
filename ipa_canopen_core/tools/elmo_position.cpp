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
 * \date Date of creation: July 2013
 *
 * \brief
 *   Get errors from the canopen device
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
    canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));


    TPCANMsg m;

 //////////////////// pp mode
    m.ID = 0x60B;//CANid + 0x60B;
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

    //std::cin.get();
    /////////////////////////

    //////////////////// get mode display
       m.ID = 0x60B;//CANid + 0x60B;
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

       //std::cin.get();
       /////////////////////////

       //////////////////// get mode display
          m.ID = 0x60B;//CANid + 0x60B;
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

          //std::cin.get();
          /////////////////////////

          //////////////////// Ready to switch on
             m.ID = 0x60B;//CANid + 0x60B;
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

             //std::cin.get();
             /////////////////////////

             //////////////////// Read status
                m.ID = 0x60B;//CANid + 0x60B;
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

                //std::cin.get();
                /////////////////////////

                //////////////////// Switch on
                   m.ID = 0x60B;//CANid + 0x60B;
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

                   //std::cin.get();
                   /////////////////////////

                   //////////////////// Read status
                      m.ID = 0x60B;//CANid + 0x60B;
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

                      //std::cin.get();
                      /////////////////////////

                      //////////////////// Start mo=1
                         m.ID = 0x60B;//CANid + 0x60B;
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

                         //std::cin.get();
                         /////////////////////////

                         //////////////////// Read status
                            m.ID = 0x60B;//CANid + 0x60B;
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

                            //std::cin.get();
                            /////////////////////////


                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                            //////////////////// Set speed(10000)
                               m.ID = 0x60B;//CANid + 0x60B;
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

                               //std::cin.get();
                               /////////////////////////

                               //////////////////// Set target position(10000)
                                  m.ID = 0x60B;//CANid + 0x60B;
                                  m.MSGTYPE = 0x00;
                                  m.LEN = 8;
                                  m.DATA[0] = 0x22;
                                  m.DATA[1] = 0x7a;
                                  m.DATA[2] = 0x60;
                                  m.DATA[3] = 0x00;
                                  m.DATA[4] = 0x00;
                                  m.DATA[5] = 0x00;
                                  m.DATA[6] = 0x00;
                                  m.DATA[7] = 0x00;
                                  CAN_Write(canopen::h, &m);

                                  std::this_thread::sleep_for(std::chrono::milliseconds(10));

                                  //std::cin.get();
                                  /////////////////////////



                                     m.ID = 0x60B;//CANid + 0x60B;
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

                                     //std::cin.get();
                                     /////////////////////////
                                     //////////////////// Set point absolute
                                        m.ID = 0x60B;//CANid + 0x60B;
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

                                        //std::cin.get();
                                     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                            //////////////////// Set speed(10000)
                               m.ID = 0x60B;//CANid + 0x60B;
                               m.MSGTYPE = 0x00;
                               m.LEN = 8;
                               m.DATA[0] = 0x22;
                               m.DATA[1] = 0x81;
                               m.DATA[2] = 0x60;
                               m.DATA[3] = 0x00;
                               m.DATA[4] = 0x10;
                               m.DATA[5] = 0x10;
                               m.DATA[6] = 0x00;
                               m.DATA[7] = 0x00;
                               CAN_Write(canopen::h, &m);

                               std::this_thread::sleep_for(std::chrono::milliseconds(10));

                               //std::cin.get();
                               /////////////////////////

                               //////////////////// Set target position(3000)
                                  m.ID = 0x60B;//CANid + 0x60B;
                                  m.MSGTYPE = 0x00;
                                  m.LEN = 8;
                                  m.DATA[0] = 0x22;
                                  m.DATA[1] = 0x7a;
                                  m.DATA[2] = 0x60;
                                  m.DATA[3] = 0x00;
                                  m.DATA[4] = 0xB8;
                                  m.DATA[5] = 0x0B;
                                  m.DATA[6] = 0x00;
                                  m.DATA[7] = 0x00;
                                  CAN_Write(canopen::h, &m);

                                  std::this_thread::sleep_for(std::chrono::milliseconds(10));

                                  //std::cin.get();
                                  /////////////////////////

                                  //////////////////// Set point absolute
                                     m.ID = 0x60B;//CANid + 0x60B;
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

                                     //std::cin.get();

                                     m.ID = 0x60B;//CANid + 0x60B;
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

                                     //std::cin.get();
                                     /////////////////////////
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                            ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                            //////////////////// Set speed(10000)
                               m.ID = 0x60B;//CANid + 0x60B;
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

                               //std::cin.get();
                               /////////////////////////

                               //////////////////// Set target position(7000)
                                  m.ID = 0x60B;//CANid + 0x60B;
                                  m.MSGTYPE = 0x00;
                                  m.LEN = 8;
                                  m.DATA[0] = 0x22;
                                  m.DATA[1] = 0x7a;
                                  m.DATA[2] = 0x60;
                                  m.DATA[3] = 0x00;
                                  m.DATA[4] = 0x58;
                                  m.DATA[5] = 0x1B;
                                  m.DATA[6] = 0x00;
                                  m.DATA[7] = 0x00;
                                  CAN_Write(canopen::h, &m);

                                  std::this_thread::sleep_for(std::chrono::milliseconds(10));

                                  //std::cin.get();
                                  /////////////////////////

                                  //////////////////// Set point absolute
                                     m.ID = 0x60B;//CANid + 0x60B;
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

                                     //std::cin.get();

                                     m.ID = 0x60B;//CANid + 0x60B;
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

                                     //std::cin.get();
                                     /////////////////////////
                            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            
                            //0x67F 0x40 0xA0 0x20 0x00 0x00 0x00 0x00 0x00
                            
                             m.ID = 0x60B;//CANid + 0x60B;
                                     m.MSGTYPE = 0x00;
                                     m.LEN = 8;
                                     m.DATA[0] = 0x40;
                                     m.DATA[1] = 0xA0;
                                     m.DATA[2] = 0x20;
                                     m.DATA[3] = 0x00;
                                     m.DATA[4] = 0x00;
                                     m.DATA[5] = 0x00;
                                     m.DATA[6] = 0x00;
                                     m.DATA[7] = 0x00;
                                     CAN_Write(canopen::h, &m);

                                     std::this_thread::sleep_for(std::chrono::milliseconds(10));

                            TPCANRdMsg ms;
            errno = LINUX_CAN_Read(canopen::h, &ms);
            if (errno)
                perror("LINUX_CAN_Read() error");
                            


}
