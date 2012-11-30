#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "cob_srvs/Trigger.h"
#include "cob_srvs/SetOperationMode.h"
// #include "ros_canopen/posmsg.h"
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <canopen.h>
// #include "yaml-cpp/yaml.h"
#include <XmlRpcValue.h>

typedef boost::function<
  bool(cob_srvs::Trigger::Request&,
       cob_srvs::Trigger::Response&)> TriggerType;
typedef boost::function<
  void(const brics_actuator::JointVelocities&)> JointVelocitiesType;
typedef boost::function<
  bool(cob_srvs::SetOperationMode::Request&,
       cob_srvs::SetOperationMode::Response&)> SetOperationModeCallbackType;

struct BusParams {
  std::string baudrate;
  uint32_t syncInterval;
};
std::map<std::string, BusParams> buses;

std::string deviceFile;

bool CANopenInit(cob_srvs::Trigger::Request &req,
		 cob_srvs::Trigger::Response &res, std::string chainName) {
  // canopen::chainMap[chainName]->CANopenInit();
  std::cout << "init service called" << std::endl;
  canopen::init(deviceFile, canopen::syncInterval);
  // if (canopen::atFirstInit) {
    for (auto device : canopen::devices)
      canopen::sendSDO(device.second.CANid_, canopen::modes_of_operation,
		       canopen::modes_of_operation_interpolated_position_mode);
    // canopen::initDeviceManagerThread(canopen::deviceManager);
    // }
  res.success.data = true;
  res.error_message.data = "";
  std::cout << "init service ended" << std::endl;
  return true;
}

bool setOperationModeCallback(cob_srvs::SetOperationMode::Request &req,
			      cob_srvs::SetOperationMode::Response &res, std::string chainName) {
  res.success.data = true;  // for now this service is just a dummy, not used elsewhere
  // res.error_message.data = "";
  return true;
}

void setVel(const brics_actuator::JointVelocities &msg, std::string chainName) {
  std::cout << "setVel callback!" << std::endl;
  if (!canopen::atFirstInit) {
    std::vector<double> velocities;
    for (auto it : msg.velocities) {
      std::cout << it.value << "  ";
      velocities.push_back( it.value); 
    }
    std::cout << std::endl;
    canopen::deviceGroups[chainName].setVel(velocities); 
  }
}

void readParamsFromParameterServer(ros::NodeHandle n) {
  XmlRpc::XmlRpcValue busParams;
  n.getParam("/CANopen/buses", busParams);
  for (int i=0; i<busParams.size(); i++) {
    BusParams busParam;
    auto name = static_cast<std::string>(busParams[i]["name"]);
    busParam.baudrate = static_cast<std::string>(busParams[i]["baudrate"]);
    busParam.syncInterval = static_cast<int>(busParams[i]["sync_interval"]);
    buses[name] = busParam;
  }
  
  XmlRpc::XmlRpcValue deviceParams;
  n.getParam("/CANopen/devices", deviceParams);
  for (int i=0; i<deviceParams.size(); i++) {
    auto name = static_cast<std::string>(deviceParams[i]["name"]);
    auto group = static_cast<std::string>(deviceParams[i]["group"]);
    auto CANid = static_cast<int>(deviceParams[i]["id"]);
    auto bus = static_cast<std::string>(deviceParams[i]["bus"]);
    canopen::devices[ CANid ] = canopen::Device(CANid, name, group, bus);
  }

  for (auto it : canopen::devices) {
    if (canopen::deviceGroups.find(it.second.group_) == canopen::deviceGroups.end())
      canopen::deviceGroups[it.second.group_] = canopen::DeviceGroup({it.first}, {it.second.name_});
    else {
      canopen::deviceGroups[it.second.group_].CANids_.push_back(it.first);
      canopen::deviceGroups[it.second.group_].names_.push_back(it.second.name_);
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "canopen_ros");
  ros::NodeHandle n("~");
  readParamsFromParameterServer(n);

  std::cout << buses.begin()->second.syncInterval << std::endl;
  canopen::syncInterval = std::chrono::milliseconds( buses.begin()->second.syncInterval );
  // ^ todo: this only works with a single CAN bus; add support for more buses!
  deviceFile = buses.begin()->first;
  std::cout << deviceFile << std::endl;
  // ^ todo: this only works with a single CAN bus; add support for more buses!
  
  // add custom PDOs:
  canopen::sendPos = canopen::schunkDefaultPDOOutgoing;
  for (auto it : canopen::devices) {
    canopen::incomingPDOHandlers[ 0x180 + it.first ] = 
      [it](const TPCANRdMsg m) { canopen::schunkDefaultPDO_incoming( it.first, m ); };
  }

  /*
  uint8_t CANid = 0xC;
  canopen::devices[ CANid ] = canopen::Device(CANid);
  canopen::incomingPDOHandlers[ 0x180 + 0xC ] = 
    [](const TPCANRdMsg m) { canopen::schunkDefaultPDO_incoming( 0xC, m ); };
  canopen::sendPos = canopen::schunkDefaultPDOOutgoing;
  canopen::deviceGroups[ "tray" ] = canopen::DeviceGroup({CANid}); */

  // set up services, subscribers, and publishers for each of the chains:
  std::vector<TriggerType> initCallbacks;
  std::vector<ros::ServiceServer> initServices;
  std::vector<SetOperationModeCallbackType> setOperationModeCallbacks;
  std::vector<ros::ServiceServer> setOperationModeServices;

  std::vector<JointVelocitiesType> jointVelocitiesCallbacks;
  std::vector<ros::Subscriber> jointVelocitiesSubscribers;
  std::map<std::string, ros::Publisher> currentOperationModePublishers;
  std::map<std::string, ros::Publisher> statePublishers;
  ros::Publisher jointStatesPublisher = 
    n.advertise<sensor_msgs::JointState>("/joint_states", 100);
  
  for (auto it : canopen::deviceGroups) {
    std::cout << it.first << std::endl;

    initCallbacks.push_back( boost::bind(CANopenInit, _1, _2, it.first) );
    initServices.push_back
      (n.advertiseService("/" + it.first + "/init", initCallbacks.back()) );
    setOperationModeCallbacks.push_back( boost::bind(setOperationModeCallback, _1, _2, it.first) );
    setOperationModeServices.push_back( n.advertiseService("/" + it.first + "/set_operation_mode", setOperationModeCallbacks.back()) );

    jointVelocitiesCallbacks.push_back( boost::bind(setVel, _1, it.first) );
    jointVelocitiesSubscribers.push_back
      (n.subscribe<brics_actuator::JointVelocities>
       ("/" + it.first + "/command_vel", 100, jointVelocitiesCallbacks.back())  );
    // todo: remove global namespace; ticket

    currentOperationModePublishers[it.first] =
      n.advertise<std_msgs::String>
      ("/" + it.first + "/current_operationmode", 1000);
    
    statePublishers[it.first] =
      n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("/" + it.first + "/state", 1000);
  }

  double lr = 1000.0 / std::chrono::duration_cast
    <std::chrono::milliseconds>(canopen::syncInterval).count();
  std::cout << "Loop rate: " << lr << std::endl;
  ros::Rate loop_rate(lr); 

  canopen::initDeviceManagerThread(canopen::deviceManager);

  while (ros::ok()) {
    
    for (auto dg : (canopen::deviceGroups)) { 
      // iterate over all chains, get current pos and vel and publish as topics:

      sensor_msgs::JointState js;  
      // std::vector<std::string> ss = {"tray_1_joint"};
      // std::vector<std::string> ss = {"tray_1_joint", "tray_2_joint", "tray_3_joint"};
      // std::vector<std::string> ss = {"arm_1_joint", "arm_2_joint", "arm_3_joint",
      // "arm_4_joint", "arm_5_joint", "arm_6_joint"};
      /* std::cout << "Names: ";
      for (auto it1 : dg.second.names_)
	std::cout << it1 << "  ";
	std::cout << std::endl; */
      js.name = dg.second.names_;
      js.header.stamp = ros::Time::now(); // todo: possibly better use timestamp of hardware msg?
      js.position = dg.second.getActualPos();
      js.velocity = dg.second.getActualVel(); 
      js.effort = std::vector<double>(dg.second.names_.size(), 0.0); // {0}; // todo {0,0,0,0,0,0};
      /* std::cout << "Efforts: ";
      for (auto it1 : js.effort)
	std::cout << it1 << "  ";
	std::cout << std::endl; */
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

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

