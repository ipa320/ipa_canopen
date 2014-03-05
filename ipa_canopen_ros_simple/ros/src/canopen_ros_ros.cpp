// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <ipa_canopen_ros_simple/canopen_rosConfig.h>

// ROS message includes
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>
#include <brics_actuator/JointVelocities.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>




#include <canopen_ros_common.cpp>


class canopen_ros_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<ipa_canopen_ros_simple::canopen_rosConfig> server;
  		dynamic_reconfigure::Server<ipa_canopen_ros_simple::canopen_rosConfig>::CallbackType f;
		

		ros::Publisher state_;
		ros::Publisher joint_states_;
		ros::Publisher diagnostics_;
		ros::Publisher current_operationmode_;
		

		ros::Subscriber command_vel_;
	ros::ServiceServer init_;
	ros::ServiceServer recover_;
        
 
        canopen_ros_data component_data_;
        canopen_ros_config component_config_;
        canopen_ros_impl component_implementation_;

        canopen_ros_ros()
        {
       	
  			f = boost::bind(&canopen_ros_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
        	
        		std::string init_remap;
        		n_.param("init_remap", init_remap, (std::string)"init");
        		init_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(init_remap, boost::bind(&canopen_ros_impl::callback_init, &component_implementation_,_1,_2,component_config_));
        		std::string recover_remap;
        		n_.param("recover_remap", recover_remap, (std::string)"recover");
        		recover_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(recover_remap, boost::bind(&canopen_ros_impl::callback_recover, &component_implementation_,_1,_2,component_config_));
        
				state_ = 	n_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
				joint_states_ = 	n_.advertise<sensor_msgs::JointState>("joint_states", 1);
				diagnostics_ = 	n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
				current_operationmode_ = 	n_.advertise<std_msgs::String>("current_operationmode", 1);
					command_vel_ =  n_.subscribe("command_vel", 1, &canopen_ros_ros::topicCallback_command_vel, this);
  	

				n_.param("can_device", component_config_.can_device, (std::string)"");
				n_.param("can_baudrate", component_config_.can_baudrate, (std::string)"500K");
				n_.param("modul_ids", component_config_.modul_ids, (std::string)"");
				n_.param("joint_names", component_config_.joint_names, (std::string)"[torso_1_joint, torso_2_joint, torso_3_joint]");
				n_.param("robot_description", component_config_.robot_description, (std::string)"");
            
        }
		
		
        
        void topicCallback_command_vel(const brics_actuator::JointVelocities::ConstPtr& msg)
		{
            component_data_.in_command_vel = *msg;
            
        }
		
		void configure_callback(ipa_canopen_ros_simple::canopen_rosConfig &config, uint32_t level) 
		{
				component_config_.can_device = config.can_device;
				component_config_.can_baudrate = config.can_baudrate;
				component_config_.modul_ids = config.modul_ids;
				component_config_.joint_names = config.joint_names;
				component_config_.robot_description = config.robot_description;
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
				state_.publish(component_data_.out_state);
				joint_states_.publish(component_data_.out_joint_states);
				diagnostics_.publish(component_data_.out_diagnostics);
				current_operationmode_.publish(component_data_.out_current_operationmode);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "canopen_ros");

	canopen_ros_ros node;
    node.configure();

	
 	ros::Rate loop_rate(100.0); // Hz

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
	
    return 0;
}
