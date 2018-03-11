#include "ros/ros.h"
#include <giskard_msgs/ControllerListAction.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <tf/transform_listener.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <kdl_parser/kdl_parser.hpp>
#include <map>

class GiskardVisualizer
{
public:
	GiskardVisualizer(ros::NodeHandle &n_, const std::map<std::string, double> &current_joint_state):
		n(),
		current_joint_state(current_joint_state),
		vis_pub(n.advertise<visualization_msgs::Marker>( "visualization_marker", 0)){
	 		std::string robot_desc_string;
  			n.param("robot_description", robot_desc_string, std::string());
   			if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
     			ROS_ERROR("Failed to construct kdl tree in giskard visualization");
 			}
 			kdlToYamlJointNames["l_upper_arm_joint"] = "l_elbow_flex_joint";
	 };
	~GiskardVisualizer();

	void visualizeController(const giskard_msgs::ControllerListGoalConstPtr	& msg);
	//void updateJointState(std::map<std::string, double> new_joint_state);

private:
	void visualizeTranslation(giskard_msgs::Controller &c);
	void visualizeRotation(giskard_msgs::Controller &c);
	void visualizeJointcommand(giskard_msgs::Controller &c);
	visualization_msgs::Marker getRotationMarker(tf::Vector3 axis, tf::Vector3 goal, int id, std::string frame_id);
	visualization_msgs::Marker getJointMarker(KDL::Frame jointFrame, KDL::Segment segment, double current_joint_state, double new_joint_state, int id, std::string frame_id);



	
   	const std::map<std::string, double> &current_joint_state;
	std::map<std::string, std::string> kdlToYamlJointNames;
   	KDL::Tree my_tree;

	ros::NodeHandle n;
	ros::Publisher vis_pub;
	tf::TransformListener listener;
};