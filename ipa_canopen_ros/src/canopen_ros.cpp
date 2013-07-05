#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
#include <diagnostic_msgs/DiagnosticArray.h>
// #include "ros_canopen/posmsg.h"
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopen.h>
// #include "yaml-cpp/yaml.h"
#include <XmlRpcValue.h>

typedef boost::function<bool(cob_srvs::Trigger::Request&, cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<void(const brics_actuator::JointVelocities&)> JointVelocitiesType;
typedef boost::function<bool(cob_srvs::SetOperationMode::Request&, cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

struct BusParams {
	std::string baudrate;
	uint32_t syncInterval;
};
std::map<std::string, BusParams> buses;

std::string deviceFile;

bool CANopenInit(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName) {

	canopen::init(deviceFile, canopen::syncInterval);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

 	for (auto device : canopen::devices){
  		canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	canopen::initDeviceManagerThread(canopen::deviceManager);

	for (auto device : canopen::devices) {
		device.second.setInitialized(true);
	}

	res.success.data = true;
	res.error_message.data = "";
	return true;
}


bool CANopenRecover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res, std::string chainName) {
	

	canopen::recover(deviceFile, canopen::syncInterval);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));


	for (auto device : canopen::devices){
		canopen::sendSDO(device.second.getCANid(), canopen::MODES_OF_OPERATION, canopen::MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	//canopen::initDeviceManagerThread(canopen::deviceManager);

	for (auto device : canopen::devices){
		device.second.setInitialized(true);
	}

	res.success.data = true;
	res.error_message.data = "";
	return true;
}


bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res, std::string chainName) {
	res.success.data = true;  // for now this service is just a dummy, not used elsewhere
	return true;
}

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName) {
	if (!canopen::atFirstInit) {
		std::vector<double> velocities;
		for (auto it : msg.velocities)
			velocities.push_back( it.value); 
		canopen::deviceGroups[chainName].setVel(velocities);
  	}
}

void readParamsFromParameterServer(ros::NodeHandle n) {
	XmlRpc::XmlRpcValue busParams;

	if (!n.hasParam("devices") || !n.hasParam("chains")) {
		ROS_ERROR("Missing parameters on parameter server; shutting down node.");
		ROS_ERROR("Please consult the user manual for necessary parameter settings.");
		n.shutdown();
	}

	n.getParam("devices", busParams);
	for (int i=0; i<busParams.size(); i++) {
		BusParams busParam;
		auto name = static_cast<std::string>(busParams[i]["name"]);
		busParam.baudrate = static_cast<std::string>(busParams[i]["baudrate"]);
		busParam.syncInterval = static_cast<int>(busParams[i]["sync_interval"]);
		buses[name] = busParam;
	}
  
	XmlRpc::XmlRpcValue chainNames_XMLRPC;
	n.getParam("chains", chainNames_XMLRPC);
	std::vector<std::string> chainNames;
	for (int i=0; i<chainNames_XMLRPC.size(); i++) 
		chainNames.push_back(static_cast<std::string>(chainNames_XMLRPC[i]));

	for (auto chainName : chainNames) {
		XmlRpc::XmlRpcValue jointNames_XMLRPC;
		n.getParam("/" + chainName + "/joint_names", jointNames_XMLRPC);
		std::vector<std::string> jointNames;
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

		for (int i=0; i<jointNames.size(); i++)
			canopen::devices[ moduleIDs[i] ] = canopen::Device(moduleIDs[i], jointNames[i], chainName, devices[i]);

		canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(moduleIDs, jointNames);

	}
}


int main(int argc, char **argv)
{
	// todo: allow identical module IDs of modules when they are on different CAN buses

	ros::init(argc, argv, "canopen_ros");
	ros::NodeHandle n; // ("~");
  
	readParamsFromParameterServer(n);


	std::cout << buses.begin()->second.syncInterval << std::endl;
	canopen::syncInterval = std::chrono::milliseconds( buses.begin()->second.syncInterval );
	// ^ todo: this only works with a single CAN bus; add support for more buses!
	deviceFile = buses.begin()->first;
	std::cout << deviceFile << std::endl;
	// ^ todo: this only works with a single CAN bus; add support for more buses!

    if (!canopen::openConnection(deviceFile)){
        std::cout << "Cannot open CAN device; aborting." << std::endl;
        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    for (auto dg : (canopen::devices)){
        std::cout << "Module with CAN-id " << (uint16_t)dg.second.getCANid() << " connected" << std::endl;
        canopen::getErrors(dg.second.getCANid());
    }

	// add custom PDOs:
    canopen::sendPos = canopen::schunkDefaultPDOOutgoing;
    for (auto it : canopen::devices) {
        canopen::incomingPDOHandlers[ 0x180 + it.first ] = [it](const TPCANRdMsg m) { canopen::schunkDefaultPDO_incoming( it.first, m ); };
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
	ros::Publisher jointStatesPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 100);
    ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  
	for (auto it : canopen::deviceGroups) {
		std::cout << it.first << std::endl;

		initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
		initServices.push_back( n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );
        recoverCallbacks.push_back( boost::bind(CANopenRecover, _1, _2, it.first) );
    		recoverServices.push_back( n.advertiseService("/" + it.first + "/recover", recoverCallbacks.back()) );
    		setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
    		setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

    		jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
    		jointVelocitiesSubscribers.push_back( n.subscribe<brics_actuator::JointVelocities>("/" + it.first + "/command_vel", 100, jointVelocitiesCallbacks.back()) );

		currentOperationModePublishers[it.first] = n.advertise<std_msgs::String>("/" + it.first + "/current_operationmode", 1000);
    
    		statePublishers[it.first] = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/" + it.first + "/state", 1000);
	}

	double lr = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(canopen::syncInterval).count();
	std::cout << "Loop rate: " << lr << std::endl;
	ros::Rate loop_rate(lr); 

	while (ros::ok()) {
    
	// iterate over all chains, get current pos and vel and publish as topics:
		for (auto dg : (canopen::deviceGroups)) { 
			sensor_msgs::JointState js;  
			js.name = dg.second.getNames();
			js.header.stamp = ros::Time::now(); // todo: possibly better use timestamp of hardware msg?
			js.position = dg.second.getActualPos();
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
        diagnostics.status.resize(1);

    for (auto dg : (canopen::devices)) {
        std::string name = dg.second.getName();
        //ROS_INFO("Name %s", name.c_str() );
        bool error_ = dg.second.getFault();
        bool initialized_ = dg.second.getInitialized();

        //ROS_INFO("Fault: %d", error_);
        //ROS_INFO("Referenced: %d", initialized_);

        // set data to diagnostics
        if(error_)
        {
          diagnostics.status[0].level = 2;
          diagnostics.status[0].name = n.getNamespace();
          diagnostics.status[0].message = "Fault occured.";
          break;
        }
        else
        {
          if (initialized_)
          {
            diagnostics.status[0].level = 0;
            diagnostics.status[0].name = n.getNamespace();
            diagnostics.status[0].message = "powerball chain initialized and running";
          }
          else
          {
            diagnostics.status[0].level = 1;
            diagnostics.status[0].name = n.getNamespace();
            diagnostics.status[0].message = "powerball chain not initialized";
            break;
          }
        }
    }
        // publish diagnostic message
        diagnosticsPublisher.publish(diagnostics);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

