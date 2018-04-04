#include <giskard_ros/giskard_visualization.hpp>
#include <iostream>
#include <cmath>



void GiskardVisualizer::init(){

	//left arm
	link_vis_info_map["l_shoulder_pan_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae", 0.2);
	link_vis_info_map["l_shoulder_lift_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae", 0.2);
	link_vis_info_map["l_upper_arm_roll_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/upper_arm_roll.dae", 0.2);
	link_vis_info_map["l_upper_arm_link"] = linkVisInfo("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae", 0.2);
	link_vis_info_map["l_elbow_flex_link"] = linkVisInfo("package://pr2_description/meshes/upper_arm_v0/elbow_flex.dae", 0.15);
	//mesh_resource_path["l_forearm_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	link_vis_info_map["l_forearm_link"] = linkVisInfo("package://pr2_description/meshes/forearm_v0/forearm.dae", 0.15);
	link_vis_info_map["l_wrist_flex_link"] = linkVisInfo("package://pr2_description/meshes/forearm_v0/wrist_flex.dae", 0.07);
	//mesh_resource_path["l_wrist_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	//mesh_resource_path["l_force_torque_adapter_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	//mesh_resource_path["l_force_torque_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	link_vis_info_map["l_gripper_palm_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/gripper_palm.dae", 0.07);
	link_vis_info_map["l_gripper_r_finger_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/l_finger.dae", 0.07);
	link_vis_info_map["l_gripper_l_finger_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/l_finger.dae", 0.07);
	link_vis_info_map["l_gripper_l_finger_tip_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae", 0.07);
	link_vis_info_map["l_gripper_r_finger_tip_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae", 0.07);
	
	//link_vis_info_map["l_gripper_l_finger_tip_frame"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/l_finger_tip.dae", 0.07);


	//right arm
	link_vis_info_map["r_shoulder_pan_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae", 0.2);
	link_vis_info_map["r_shoulder_lift_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae", 0.2);
	link_vis_info_map["r_upper_arm_roll_link"] = linkVisInfo("package://pr2_description/meshes/shoulder_v0/upper_arm_roll.dae", 0.2);
	link_vis_info_map["r_upper_arm_link"] = linkVisInfo("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae", 0.2);
	link_vis_info_map["r_elbow_flex_link"] = linkVisInfo("package://pr2_description/meshes/upper_arm_v0/elbow_flex.dae", 0.15);
	//mesh_resource_path["l_forearm_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	link_vis_info_map["r_forearm_link"] = linkVisInfo("package://pr2_description/meshes/forearm_v0/forearm.dae", 0.15);
	link_vis_info_map["r_wrist_flex_link"] = linkVisInfo("package://pr2_description/meshes/forearm_v0/wrist_flex.dae", 0.07);
	//mesh_resource_path["l_wrist_roll_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	//mesh_resource_path["l_force_torque_adapter_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	//mesh_resource_path["l_force_torque_link"] = "package://pr2_description/meshes/base_v0/base.dae"
	link_vis_info_map["r_gripper_palm_link"] = linkVisInfo("package://pr2_description/meshes/gripper_v0/gripper_palm.dae", 0.07);
}



void GiskardVisualizer::visualizeTranslation(const giskard_msgs::Controller& c, int marker_id){
	tf::StampedTransform transform;
	tf::Vector3 arrow_start;

	try{
		ros::Time now = ros::Time::now();
    	listener.waitForTransform(c.goal_pose.header.frame_id, c.tip_link, now, ros::Duration(3.0));
		listener.lookupTransform(c.goal_pose.header.frame_id, c.tip_link, now, transform);
		arrow_start = transform.getOrigin();
		tf::Vector3 arrow_end;

		arrow_end.setX(c.goal_pose.pose.position.x);
		arrow_end.setY(c.goal_pose.pose.position.y);
		arrow_end.setZ(c.goal_pose.pose.position.z);

		publishArrowMarker(markerColor(0.5, 1, 0.8, 0), marker_id, c.goal_pose.header.frame_id, "translation", arrow_start, arrow_end);

	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

void GiskardVisualizer::publishArrowMarker(markerColor color, int id, std::string frame_id , std::string name_space, tf::Vector3 start, tf::Vector3 end){

	visualization_msgs::Marker arrow_marker;

	arrow_marker.header.frame_id = frame_id;
	arrow_marker.header.stamp = ros::Time();
	arrow_marker.ns = name_space;
	arrow_marker.id = id;
	arrow_marker.type = visualization_msgs::Marker::ARROW;
	arrow_marker.action = visualization_msgs::Marker::ADD;

	arrow_marker.scale.x = 0.01;
	arrow_marker.scale.y = 0.02;
	arrow_marker.scale.z = 0.03;
	arrow_marker.color.a = color.a;
	arrow_marker.color.r = color.r;
	arrow_marker.color.g = color.g;
	arrow_marker.color.b = color.b;

	arrow_marker.points.resize(2);
	arrow_marker.points[0].x = start.getX();
	arrow_marker.points[0].y = start.getY();
	arrow_marker.points[0].z = start.getZ();

	arrow_marker.points[1].x = end.getX();
	arrow_marker.points[1].y = end.getY();
	arrow_marker.points[1].z = end.getZ();

	vis_pub.publish(arrow_marker);
}


void GiskardVisualizer::visualizeRotation(const giskard_msgs::Controller& c, int base_marker_id){
	try{
		tf::Transform transformGoalRotation(tf::Quaternion(c.goal_pose.pose.orientation.x, c.goal_pose.pose.orientation.y, c.goal_pose.pose.orientation.z, c.goal_pose.pose.orientation.w), tf::Vector3(0,0,0));
		tf::StampedTransform transformFrameToTipLink;
		
		ros::Time now = ros::Time::now();
    	listener.waitForTransform(c.tip_link, c.goal_pose.header.frame_id, now, ros::Duration(3.0));
		listener.lookupTransform(c.tip_link, c.goal_pose.header.frame_id, now, transformFrameToTipLink);

		transformFrameToTipLink.setOrigin(tf::Vector3(0,0,0));

		tf::Transform combinedTransform = transformFrameToTipLink * transformGoalRotation;

		float rotation_scale = 0.1;
		auto it = link_vis_info_map.find(c.tip_link);
		if(it != link_vis_info_map.end()){
			rotation_scale = it->second.rotation_marker_scale;
		}


		tf::Vector3 xAxisInGoal = combinedTransform * tf::Vector3(1,0,0);
		tf::Vector3 yAxisInGoal = combinedTransform * tf::Vector3(0,1,0);
		tf::Vector3 zAxisInGoal = combinedTransform * tf::Vector3(0,0,1);

		publishArrowMarker(markerColor(0.3, 1, 0, 0), base_marker_id + 3, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * xAxisInGoal);
		publishArrowMarker(markerColor(0.3, 0, 1, 0), base_marker_id + 4, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * yAxisInGoal);
		publishArrowMarker(markerColor(0.3, 0, 0, 1), base_marker_id + 5, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * zAxisInGoal);


		publishArrowMarker(markerColor(1, 1, 0, 0), base_marker_id + 0, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * tf::Vector3(1,0,0));
		publishArrowMarker(markerColor(1, 0, 1, 0), base_marker_id + 1, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * tf::Vector3(0,1,0));
		publishArrowMarker(markerColor(1, 0, 0, 1), base_marker_id + 2, c.tip_link, "rotation_axis", tf::Vector3(0,0,0), rotation_scale * tf::Vector3(0,0,1));
		
	
		
		publishRotationMarker(rotation_scale * tf::Vector3(1,0,0), xAxisInGoal, base_marker_id + 0, c.tip_link);
		publishRotationMarker(rotation_scale * tf::Vector3(0,1,0), yAxisInGoal, base_marker_id + 1, c.tip_link);
		publishRotationMarker(rotation_scale * tf::Vector3(0,0,1), zAxisInGoal, base_marker_id + 2, c.tip_link);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

void GiskardVisualizer::publishRotationMarker(tf::Vector3 axis, tf::Vector3 goal, int id, std::string frame_id){
	tf::Vector3 crossX = axis.cross(goal);
	crossX.normalize();
	visualization_msgs::Marker marker;
	
	geometry_msgs::Point p;
	p.x = axis.getX();
	p.y = axis.getY();
	p.z = axis.getZ();
	marker.points.push_back(p);

	double angle = std::acos(goal.dot(axis) / (axis.length() * goal.length()));
	double stepSize = angle/20;

	for(int i = 0; i<20; i++){
		axis = axis * std::cos(stepSize) + std::sin(stepSize) * crossX.cross(axis) + crossX * (crossX.dot(axis)) * (1-std::cos(stepSize));
		geometry_msgs::Point p;
		p.x = axis.getX();
		p.y = axis.getY();
		p.z = axis.getZ();
		marker.points.push_back(p);
	}
	marker.scale.x = 0.01;
	marker.scale.z = 0.01;
	marker.scale.y = 0.01;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.8;
	marker.color.b = 0.0;
	marker.header.stamp = ros::Time();
	marker.ns = "rotation";
	marker.id = id;
	marker.header.frame_id = frame_id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	vis_pub.publish(marker);
}


void GiskardVisualizer::getTipLinks(SegmentMap::const_iterator it, std::string root, tipRootPairVector& tip_links){
	//std::vector<SegmentMap::const_iterator> children_vector = it->second.children;

	switch(it->second.children.size()){
		case 0:
			tip_links.push_back(std::pair<std::string, std::string>(root, it->second.segment.getName()));
			return;
		case 1:
			getTipLinks(it->second.children[0], root, tip_links);
			break;
		default:
			getTipLinks(it->second.children[0], root, tip_links);

			for(int i = 1; i < it->second.children.size(); i++){
				getTipLinks(it->second.children[i], it->second.segment.getName(), tip_links);
			}
			break;
	}
}

KDL::JntArray GiskardVisualizer::getJointPositionsForChain(KDL::Chain chain, const giskard_msgs::Controller& c){
	unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions(nj);


	//fill joint positions
    for(int segmentIdx = 0, jointIdx = 0; segmentIdx < chain.segments.size(); segmentIdx++){
    	KDL::Joint j = chain.segments[segmentIdx].getJoint();
    	if(j.getType() == KDL::Joint::None){
    		continue;
    	}

    	std::string name = j.getName();

    	double joint_state;

    	auto goalIt = std::find(c.goal_state.name.begin(), c.goal_state.name.end(), name);
    	if(goalIt != c.goal_state.name.end()){
			joint_state = c.goal_state.position[goalIt - c.goal_state.name.begin()];
    	}
    	else{
    		auto it = current_joint_state.find(name);
    		if(it != current_joint_state.end()){
    			joint_state = it->second;
    		}
    	}
		jointpositions(jointIdx) = joint_state;
		jointIdx++;
    }

    return jointpositions;
}

void GiskardVisualizer::visualizeJointcommand(const giskard_msgs::Controller& c, int base_marker_id){
	tipRootPairVector tip_links;

	SegmentMap::const_iterator root_iterator = segment_map.find(c.root_link);
	
	getTipLinks(root_iterator, c.root_link, tip_links);

	int marker_id = 0;

	for(auto p : tip_links){
		std::cout << "pair: " << p.first << " " << p.second << std::endl;


		bool exit_value;
		
		KDL::Chain root_chain;
		
		exit_value = kdl_tree.getChain(c.root_link, p.first, root_chain);
		KDL::ChainFkSolverPos_recursive fksolver_root_chain(root_chain);
		KDL::JntArray jointpositions_root_chain = getJointPositionsForChain(root_chain, c);

		KDL::Frame base;
		bool kinematics_status;
		kinematics_status = fksolver_root_chain.JntToCart(jointpositions_root_chain, base);
		//std::cout << "	base x " << base.p.x() << " y " << base.p.y() << " z " << base.p.z() << std::endl;

		KDL::Chain tip_chain;
		exit_value = kdl_tree.getChain(p.first, p.second, tip_chain);
		KDL::ChainFkSolverPos_recursive fksolver_tip_chain(tip_chain);
		KDL::JntArray jointpositions_tip_chain = getJointPositionsForChain(tip_chain, c);
		
		for(int segmentIdx = 0; segmentIdx < tip_chain.segments.size(); segmentIdx++){
	    	std::string segment_name = tip_chain.segments[segmentIdx].getName();
	    	std::cout << "		segment name: " << segment_name << std::endl;
	    	auto it = link_vis_info_map.find(segment_name);
	    	if(it == link_vis_info_map.end() || it->second.mesh_resource_path.size() == 0){
	    		continue;
	    	}

		 	KDL::Frame tip_link_frame;
			bool kinematics_status;
			kinematics_status = fksolver_tip_chain.JntToCart(jointpositions_tip_chain, tip_link_frame, segmentIdx+1);
			KDL::Frame final_frame = base * tip_link_frame;
			//std::cout << "		final_frame x " << final_frame.p.x() << " y " << final_frame.p.y() << " z " << final_frame.p.z() << std::endl;
			
			publishJointMarker(final_frame, it->second.mesh_resource_path, marker_id + base_marker_id, c.root_link);
			marker_id++;
 	   }
	}
}

void GiskardVisualizer::publishJointMarker(KDL::Frame jointFrame, std::string mesh_path, int id, std::string frame_id){
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = mesh_path;
	marker.pose.position.x = jointFrame.p.x();
	marker.pose.position.y = jointFrame.p.y();
	marker.pose.position.z = jointFrame.p.z();

	double x;
	double y;
	double z;
	double w;

	jointFrame.M.GetQuaternion(x,y,z,w);

	marker.pose.orientation.x = x;
	marker.pose.orientation.y = y;
	marker.pose.orientation.z = z;
	marker.pose.orientation.w = w;
	marker.scale.x = 1.0;
	marker.scale.z = 1.0;
	marker.scale.y = 1.0;
	marker.color.a = 0.5;
	marker.color.r = 1.0;
	marker.color.g = 0.8;
	marker.color.b = 0.0;
	marker.header.stamp = ros::Time();
	marker.ns = "joint_vis";
	marker.id = id;
	marker.header.frame_id = frame_id;
	marker.action = visualization_msgs::Marker::ADD;
	vis_pub.publish(marker);
}

void GiskardVisualizer::visualizeController(const giskard_msgs::ControllerListGoalConstPtr	& msg){
	int num_translation_controller = 0;
	int num_rotation_controller = 0;
	int num_joint_controller = 0;
	for(giskard_msgs::Controller c : msg->controllers){
		switch (c.type){
			case giskard_msgs::Controller::TRANSLATION_3D:
				visualizeTranslation(c, num_translation_controller);
				num_translation_controller++;
				break;
			case giskard_msgs::Controller::JOINT:
				visualizeJointcommand(c, num_joint_controller * link_vis_info_map.size());
				num_joint_controller++;
				break;
			case giskard_msgs::Controller::ROTATION_3D:
				visualizeRotation(c, num_rotation_controller * 10);
				num_rotation_controller++;
				break;
			default:
				ROS_WARN("the controller was not recognized by the visualization");
				break;
		}
	}
}

/*void GiskardVisualizer::updateJointState(std::map<std::string, double> new_joint_state){
	current_joint_state = new_joint_state;
}*/


GiskardVisualizer::~GiskardVisualizer(){
	
}


void GiskardVisualizer::deleteMarkers(){
	visualization_msgs::Marker marker;
	marker.action = 3;
	vis_pub.publish(marker);
}