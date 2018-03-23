#include <ros/ros.h>
#include <giskard_msgs/ControllerListAction.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <unordered_map>
#include <utility>

class GiskardVisualizer
{
public:
	GiskardVisualizer(ros::NodeHandle &n_, const std::map<std::string, double> &current_joint_state):
		n(),
		current_joint_state(current_joint_state),
		vis_pub(n.advertise<visualization_msgs::Marker>( "visualization_marker", 0)){
	 		std::string robot_desc_string;
  			n.param("robot_description", robot_desc_string, std::string());
   			if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
     			ROS_ERROR("Failed to construct kdl tree in giskard visualization");
 			}

 			segment_map = kdl_tree.getSegments();

 			mesh_resource_path["l_shoulder_pan_link"] = "package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae";
 			mesh_resource_path["l_shoulder_lift_link"] = "package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae";
 			mesh_resource_path["l_upper_arm_roll_link"] = "package://pr2_description/meshes/shoulder_v0/upper_arm_roll.dae";
 			mesh_resource_path["l_upper_arm_link"] = "package://pr2_description/meshes/upper_arm_v0/upper_arm.dae";
 			mesh_resource_path["l_elbow_flex_link"] = "package://pr2_description/meshes/upper_arm_v0/elbow_flex.dae";
 			//mesh_resource_path["l_forearm_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
 			mesh_resource_path["l_forearm_link"] = "package://pr2_description/meshes/forearm_v0/forearm.dae";
 			mesh_resource_path["l_wrist_flex_link"] = "package://pr2_description/meshes/forearm_v0/wrist_flex.dae";
 			//mesh_resource_path["l_wrist_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
 			//mesh_resource_path["l_force_torque_adapter_link"] = "package://pr2_description/meshes/base_v0/base.dae"
 			//mesh_resource_path["l_force_torque_link"] = "package://pr2_description/meshes/base_v0/base.dae"
 			mesh_resource_path["l_gripper_palm_link"] = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae";


	 };
	~GiskardVisualizer();

	void visualizeController(const giskard_msgs::ControllerListGoalConstPtr	& msg);
	//void updateJointState(std::map<std::string, double> new_joint_state);

private:
	typedef std::map<std::string, KDL::TreeElement> SegmentMap;
	typedef std::vector<std::pair<std::string, std::string>> tipRootPairVector;

	struct markerColor{
		markerColor(double a, double r, double g, double b): a(a), r(r), g(g), b(b){};
		double a;
		double r;
		double g;
		double b;
	};

	void visualizeTranslation(const giskard_msgs::Controller& c);
	void visualizeRotation(const giskard_msgs::Controller& c);
	void visualizeJointcommand(const giskard_msgs::Controller& c);

	void publishRotationMarker(tf::Vector3 axis, tf::Vector3 goal, int id, std::string frame_id);
	void publishJointMarker(KDL::Frame jointFrame, std::string mesh_path, int id, std::string frame_id);
	void publishArrowMarker(markerColor color, int id, std::string frame_id , std::string name_space, tf::Vector3 start, tf::Vector3 end);

	void getTipLinks(SegmentMap::const_iterator it, std::string root, tipRootPairVector& tip_links);
	KDL::JntArray getJointPositionsForChain(KDL::Chain chain, const giskard_msgs::Controller& c);

	

	std::unordered_map<std::string, std::string> mesh_resource_path;
   	const std::map<std::string, double> &current_joint_state;
   	KDL::Tree kdl_tree;

	ros::NodeHandle n;
	ros::Publisher vis_pub;
	tf::TransformListener listener;
	SegmentMap segment_map;
};