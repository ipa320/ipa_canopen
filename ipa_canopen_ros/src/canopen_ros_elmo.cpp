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
 *   Author: Thiago de Freitas
 * \author
 *   Supervised by: Thiago de Freitas, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: September 2013
 *
 * \brief
 *   This ros node is specific to the Elmo motor controller
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
#include "ros/ros.h"
#include <urdf/model.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopen.h>
#include <XmlRpcValue.h>
#include <JointLimits.h>

typedef boost::function<bool(cob_srvs::Trigger::Request&, cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<void(const brics_actuator::JointVelocities&)> JointVelocitiesType;
typedef boost::function<bool(cob_srvs::SetOperationMode::Request&, cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

struct BusParams
{
    std::string baudrate;
    uint32_t syncInterval;
};

std::map<std::string, BusParams> buses;

std::string deviceFile;

JointLimits* joint_limits_;
std::vector<std::string> chainNames;
std::vector<std::string> jointNames;

bool CANopenInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{
    ROS_INFO("Initializing modules");
    canopen::init_elmo(deviceFile, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));


//    for (auto device : canopen::devices)
//    {

//        canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
//        std::cout << "Setting IP mode for: " << (uint16_t)device.second.getCANid() << std::endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    canopen::initDeviceManagerThread(canopen::deviceManager_elmo);

    for (auto device : canopen::devices)
    {
        canopen::devices[(uint16_t)device.second.getCANid()].setInitialized(true);

       // if(device.second.getHomingError())
         //   return false;

    }


    res.success.data = true;
    res.error_message.data = "";
    ROS_INFO("Init concluded");
    return true;
}


bool CANopenRecover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName)
{


    ROS_INFO("Recovering modules");
    canopen::recover_elmo(deviceFile, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));



    for (auto device : canopen::devices)
    {
        canopen::devices[device.second.getCANid()].setDesiredPos((double)device.second.getActualPos());
        canopen::devices[device.second.getCANid()].setDesiredVel(0);

        canopen::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());
        canopen::sendPos((uint16_t)device.second.getCANid(), (double)device.second.getDesiredPos());

        device.second.setInitialized(true);
    }

    res.success.data = true;
    res.error_message.data = "";
    ROS_INFO("Recover concluded");
    return true;
}


bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res, std::string chainName)
{
    res.success.data = true;  // for now this service is just a dummy, not used elsewhere
    return true;
}

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName)
{
    if (!canopen::atFirstInit & !canopen::recover_active)
    {
        std::vector<double> velocities;
        std::vector<double> positions;

        double velocity;

        for (auto it : msg.velocities)
        {
            velocity = it.value;


            if(velocity > 0)
            {
                if(canopen::halt_positive)
                {
                    velocity = 0;
                    ROS_WARN("Current position is extreme positive. Can not move more in this direction.");
                    //canopen::elmo_halt(deviceFile, canopen::syncInterval);
                }
            }


            if(velocity < 0)
            {
                if(canopen::halt_negative)
                {
                    velocity = 0;

                    ROS_WARN("Current position is extreme negative. Can not move more in this direction.");
                    //canopen::elmo_halt(deviceFile, canopen::syncInterval);
                }
            }


            velocities.push_back( velocity);
        }

        for (auto device : canopen::devices)
        {
            positions.push_back((double)device.second.getDesiredPos());
        }

        //joint_limits_->checkVelocityLimits(velocities);
        //joint_limits_->checkPositionLimits(velocities, positions);

        canopen::deviceGroups[chainName].setVel(velocities);
    }
}

void readParamsFromParameterServer(ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue busParams;

    if (!n.hasParam("devices") || !n.hasParam("chains"))
    {
        ROS_ERROR("Missing parameters on parameter server; shutting down node.");
        ROS_ERROR("Please consult the user manual for necessary parameter settings.");
        n.shutdown();
    }

    n.getParam("devices", busParams);
    for (int i=0; i<busParams.size(); i++)
    {
        BusParams busParam;
        auto name = static_cast<std::string>(busParams[i]["name"]);
        busParam.baudrate = static_cast<std::string>(busParams[i]["baudrate"]);
        busParam.syncInterval = static_cast<int>(busParams[i]["sync_interval"]);
        buses[name] = busParam;
    }

    XmlRpc::XmlRpcValue chainNames_XMLRPC;
    n.getParam("chains", chainNames_XMLRPC);

    for (int i=0; i<chainNames_XMLRPC.size(); i++)
        chainNames.push_back(static_cast<std::string>(chainNames_XMLRPC[i]));

    for (auto chainName : chainNames) {
        XmlRpc::XmlRpcValue jointNames_XMLRPC;
        n.getParam("/" + chainName + "/joint_names", jointNames_XMLRPC);

        for (int i=0; i<jointNames_XMLRPC.size(); i++)
            jointNames.push_back(static_cast<std::string>(jointNames_XMLRPC[i]));

        XmlRpc::XmlRpcValue moduleIDs_XMLRPC;
        n.getParam("/" + chainName + "/module_ids", moduleIDs_XMLRPC);
        std::vector<uint8_t> moduleIDs;
        for (int i=0; i<moduleIDs_XMLRPC.size(); i++)
            moduleIDs.push_back(static_cast<int>(moduleIDs_XMLRPC[i]));

        XmlRpc::XmlRpcValue devices_XMLRPC;
        n.getParam("/" + chainName + "/devices", devices_XMLRPC);
        std::vector<std::string> devices;
        for (int i=0; i<devices_XMLRPC.size(); i++)
            devices.push_back(static_cast<std::string>(devices_XMLRPC[i]));

        for (unsigned int i=0; i<jointNames.size(); i++)
            canopen::devices[ moduleIDs[i] ] = canopen::Device(moduleIDs[i], jointNames[i], chainName, devices[i]);

        canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(moduleIDs, jointNames);

    }

}

void setJointConstraints(ros::NodeHandle n)
{
    /******************************************
     *
     *
     *
     */

    /// Get robot_description from ROS parameter server
      joint_limits_ = new JointLimits();
      int DOF = jointNames.size();

      std::string param_name = "/robot_description";
      std::string full_param_name;
      std::string xml_string;

      n.searchParam(param_name, full_param_name);
      if (n.hasParam(full_param_name))
      {
          n.getParam(full_param_name.c_str(), xml_string);
      }

      else
      {
          ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
          n.shutdown();
      }

      if (xml_string.size() == 0)
      {
          ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
          n.shutdown();
      }
      //ROS_INFO("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

      /// Get urdf model out of robot_description
      urdf::Model model;

      if (!model.initString(xml_string))
      {
          ROS_ERROR("Failed to parse urdf file");
          n.shutdown();
      }
      ROS_INFO("Successfully parsed urdf file");

      /// Get max velocities out of urdf model
      std::vector<double> MaxVelocities(DOF);
      for (int i = 0; i < DOF; i++)
      {
          MaxVelocities[i] = model.getJoint(jointNames[i].c_str())->limits->velocity;
      }

      /// Get lower limits out of urdf model
      std::vector<double> LowerLimits(DOF);
      for (int i = 0; i < DOF; i++)
      {
          LowerLimits[i] = model.getJoint(jointNames[i].c_str())->limits->lower;
      }

      // Get upper limits out of urdf model
      std::vector<double> UpperLimits(DOF);
      for (int i = 0; i < DOF; i++)
      {
          UpperLimits[i] = model.getJoint(jointNames[i].c_str())->limits->upper;
      }

      /// Get offsets out of urdf model
      std::vector<double> Offsets(DOF);
      for (int i = 0; i < DOF; i++)
      {
          Offsets[i] = model.getJoint(jointNames[i].c_str())->calibration->rising.get()[0];
      }

      /// Set parameters

      joint_limits_->setDOF(DOF);
      joint_limits_->setUpperLimits(UpperLimits);
      joint_limits_->setLowerLimits(LowerLimits);
      joint_limits_->setMaxVelocities(MaxVelocities);
      joint_limits_->setOffsets(Offsets);

     /********************************************
     *
     *
     ********************************************/
}


int main(int argc, char **argv)
{
    // todo: allow identical module IDs of modules when they are on different CAN buses


    ros::init(argc, argv, "canopen_ros");
    ros::NodeHandle n(""); // ("~");

    readParamsFromParameterServer(n);
    canopen::operation_mode = canopen::MODES_OF_OPERATION_PROFILE_VELOCITY_MODE;

    std::cout << "Sync Interval" << buses.begin()->second.syncInterval << std::endl;
    canopen::syncInterval = std::chrono::milliseconds( buses.begin()->second.syncInterval );
    // ^ todo: this only works with a single CAN bus; add support for more buses!
    deviceFile = buses.begin()->first;
    std::cout << "Opening device..." << deviceFile << std::endl;
    // ^ todo: this only works with a single CAN bus; add support for more buses!

    if (!canopen::openConnection(deviceFile))
    {
        ROS_ERROR("Cannot open CAN device; aborting.");
        exit(EXIT_FAILURE);
    }
    else
    {
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    canopen::pre_init();

    /********************************************/

    // add custom PDOs:
    canopen::sendVel = canopen::defaultPDOOutgoing_elmo;
    for (auto it : canopen::devices) {
        canopen::incomingPDOHandlers[ 0x180 + it.first] = [it](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status_elmo( it.first, m ); };
        canopen::incomingPDOHandlers[ 0x480 + it.first] = [it](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos_elmo( it.first, m ); };
       // canopen::incomingEMCYHandlers[ 0x081 + it.first ] = [it](const TPCANRdMsg mE) { canopen::defaultEMCY_incoming( it.first, mE ); };
    }

    // set up services, subscribers, and publishers for each of the chains:
    std::vector<TriggerType> initCallbacks;
    std::vector<ros::ServiceServer> initServices;
    std::vector<TriggerType> recoverCallbacks;
    std::vector<ros::ServiceServer> recoverServices;
    std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
    std::vector<ros::ServiceServer> setOperationModeServices;

    std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
    std::vector<ros::Subscriber> jointVelocitiesSubscribers;
    std::map<std::string, ros::Publisher> currentOperationModePublishers;
    std::map<std::string, ros::Publisher> statePublishers;
    ros::Publisher jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

    for (auto it : canopen::deviceGroups)
    {
        ROS_INFO("Configuring %s", it.first.c_str());

        initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
        initServices.push_back( n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );
        recoverCallbacks.push_back( boost::bind(CANopenRecover, _1, _2, it.first) );
        recoverServices.push_back( n.advertiseService("/" + it.first + "/recover", recoverCallbacks.back()) );
        setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
        setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

        jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
        jointVelocitiesSubscribers.push_back( n.subscribe<brics_actuator::JointVelocities>("/" + it.first + "/command_vel", 1, jointVelocitiesCallbacks.back()) );

        currentOperationModePublishers[it.first] = n.advertise<std_msgs::String>("/" + it.first + "/current_operationmode", 1);

        statePublishers[it.first] = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/" + it.first + "/state", 1);
    }

    double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();

    ros::Rate loop_rate(lr);

    //setJointConstraints(n);

    while (ros::ok())
    {

    // iterate over all chains, get current pos and vel and publish as topics:
        for (auto dg : (canopen::deviceGroups))
        {
            sensor_msgs::JointState js;
            js.name = dg.second.getNames();
            js.header.stamp = ros::Time::now(); // todo: possibly better use timestamp of hardware msg?
            js.position = dg.second.getActualPos();
            //std::cout << "Position" << js.position[0] << std::endl;
            js.velocity = dg.second.getActualVel();
            js.effort = std::vector<double>(dg.second.getNames().size(), 0.0);
            jointStatesPublisher.publish(js);

            pr2_controllers_msgs::JointTrajectoryControllerState jtcs;
            jtcs.header.stamp = js.header.stamp;
            jtcs.actual.positions = js.position;
            jtcs.actual.velocities = js.velocity;
            jtcs.desired.positions = dg.second.getDesiredPos();
            jtcs.desired.velocities = dg.second.getDesiredVel();
            statePublishers[dg.first].publish(jtcs);

            std_msgs::String opmode;
            opmode.data = "velocity";
            currentOperationModePublishers[dg.first].publish(opmode);
        }

        // publishing diagnostic messages
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostic_msgs::DiagnosticStatus diagstatus;
        std::vector<diagnostic_msgs::DiagnosticStatus> diagstatus_msg;
        diagnostic_msgs::KeyValue keyval;

        std::vector<diagnostic_msgs::KeyValue> keyvalues;



        diagnostics.status.resize(1);

    for (auto dg : (canopen::devices))
    {
        std::string name = dg.second.getName();
        //ROS_INFO("Name %s", name.c_str() );

        keyval.key = "Node ID";
        uint16_t node_id = dg.second.getCANid();
        std::stringstream result;
        result << node_id;
        keyval.value = result.str().c_str();
        keyvalues.push_back(keyval);

        keyval.key = "Hardware Version";
        std::vector<char> manhw = dg.second.getManufacturerHWVersion();
        keyval.value = std::string(manhw.begin(), manhw.end());
        keyvalues.push_back(keyval);

        keyval.key = "Software Version";
        std::vector<char> mansw = dg.second.getManufacturerSWVersion();
        keyval.value = std::string(mansw.begin(), mansw.end());
        keyvalues.push_back(keyval);

        keyval.key = "Device Name";
        std::vector<char> dev_name = dg.second.getManufacturerDevName();
        keyval.value = std::string(dev_name.begin(), dev_name.end());
        keyvalues.push_back(keyval);

        keyval.key = "Vendor ID";
        std::vector<uint16_t> vendor_id = dg.second.getVendorID();
        std::stringstream result1;
        for (auto it : vendor_id)
        {
           result1 <<  std::hex << it;
        }
        keyval.value = result1.str().c_str();
        keyvalues.push_back(keyval);

        keyval.key = "Revision Number";
        uint16_t rev_number = dg.second.getRevNumber();
        std::stringstream result2;
        result2 << rev_number;
        keyval.value = result2.str().c_str();
        keyvalues.push_back(keyval);

        keyval.key = "Product Code";
        std::vector<uint16_t> prod_code = dg.second.getProdCode();
        std::stringstream result3;
        std::copy(prod_code.begin(), prod_code.end(), std::ostream_iterator<uint16_t>(result3, " "));
        keyval.value = result3.str().c_str();
        keyvalues.push_back(keyval);

        bool error_ = dg.second.getFault();
        bool initialized_ = dg.second.getInitialized();

        //ROS_INFO("Fault: %d", error_);
        //ROS_INFO("Referenced: %d", initialized_);

        // set data to diagnostics
        if(error_)
        {
          diagstatus.level = 2;
          diagstatus.name = chainNames[0];
          diagstatus.message = "Fault occured.";
          diagstatus.values = keyvalues;
          break;
        }
        else
        {
          if (initialized_)
          {
            diagstatus.level = 0;
            diagstatus.name = chainNames[0];
            diagstatus.message = "canopen_elmo chain initialized and running";
            diagstatus.values = keyvalues;
          }
          else
          {
            diagstatus.level = 1;
            diagstatus.name = chainNames[0];
            diagstatus.message = "canopen_elmo chain not initialized";
            diagstatus.values = keyvalues;
            break;
          }
        }
    }
        diagstatus_msg.push_back(diagstatus);
        // publish diagnostic message
        diagnostics.status = diagstatus_msg;
        diagnostics.header.stamp = ros::Time::now();
        diagnosticsPublisher.publish(diagnostics);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

