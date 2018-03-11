#include <giskard_ros/giskard_visualization.hpp>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <kdl/chainfksolverpos_recursive.hpp>

void GiskardVisualizer::visualizeTranslation(giskard_msgs::Controller &c){
	
	tf::StampedTransform transform;
	tf::Vector3 arrow_start;


	try{
		ros::Time now = ros::Time::now();
    	listener.waitForTransform(c.goal_pose.header.frame_id, c.tip_link, now, ros::Duration(3.0));
		listener.lookupTransform(c.goal_pose.header.frame_id, c.tip_link, now, transform);
		arrow_start = transform.getOrigin();

		//visualization_msgs::Marker goal_pos;
		visualization_msgs::Marker arrow;
		/*
		goal_pos.header.frame_id = "base_link";
		goal_pos.header.stamp = ros::Time();
		goal_pos.ns = "translation";
		goal_pos.id = 0;
		goal_pos.type = visualization_msgs::Marker::SPHERE;
		goal_pos.action = visualization_msgs::Marker::ADD;

		goal_pos.pose.position = c.goal_pose.pose.position;

		goal_pos.scale.x = 0.05;
		goal_pos.scale.y = 0.05;
		goal_pos.scale.z = 0.05;
		goal_pos.color.a = 1.0;
		goal_pos.color.r = 0.7;
		goal_pos.color.g = 1.0;
		goal_pos.color.b = 0.0;*/

		arrow.header.frame_id = c.goal_pose.header.frame_id;
		arrow.header.stamp = ros::Time();
		arrow.ns = "translation";
		arrow.id = 1;
		arrow.type = visualization_msgs::Marker::ARROW;
		arrow.action = visualization_msgs::Marker::ADD;

		arrow.points.resize(2);
		arrow.points[1] = c.goal_pose.pose.position;

		arrow.points[0].x = arrow_start.getX();
		arrow.points[0].y = arrow_start.getY();
		arrow.points[0].z = arrow_start.getZ();

		arrow.scale.x = 0.01;
		arrow.scale.y = 0.02;
		arrow.scale.z = 0.03;
		arrow.color.a = 1.0;
		arrow.color.r = 1.0;
		arrow.color.g = 0.7;
		arrow.color.b = 0.0;

		//vis_pub.publish(goal_pos);
		vis_pub.publish(arrow);

	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

void GiskardVisualizer::visualizeRotation(giskard_msgs::Controller &c){
	std::cout << "rot" << std::endl;

	try{
		tf::Transform transformGoalRotation(tf::Quaternion(c.goal_pose.pose.orientation.x, c.goal_pose.pose.orientation.y, c.goal_pose.pose.orientation.z, c.goal_pose.pose.orientation.w), tf::Vector3(0,0,0));
		tf::StampedTransform transformFrameToTipLink;
		
		ros::Time now = ros::Time::now();
    	listener.waitForTransform(c.tip_link, c.goal_pose.header.frame_id, now, ros::Duration(3.0));
		listener.lookupTransform(c.tip_link, c.goal_pose.header.frame_id, now, transformFrameToTipLink);

		transformFrameToTipLink.setOrigin(tf::Vector3(0,0,0));
		transformFrameToTipLink;

		tf::Transform combinedTransform = transformFrameToTipLink * transformGoalRotation;

		tf::Vector3 xRotated = combinedTransform * tf::Vector3(0.1,0,0);
		tf::Vector3 yRotated = combinedTransform * tf::Vector3(0,0.1,0);
		tf::Vector3 zRotated = combinedTransform * tf::Vector3(0,0,0.1);



		/*
		visualization_msgs::Marker xtest;
		visualization_msgs::Marker ytest;
		visualization_msgs::Marker ztest;
		
		xtest.header.frame_id = c.tip_link;
		xtest.header.stamp = ros::Time();
		xtest.ns = "rotation";
		xtest.id = 10;
		xtest.type = visualization_msgs::Marker::ARROW;
		xtest.action = visualization_msgs::Marker::ADD;

		xtest.points.resize(2);
		xtest.points[0].x = 0;
		xtest.points[0].y = 0;
		xtest.points[0].z = 0;

		xtest.points[1].x = xRotated.getX();
		xtest.points[1].y = xRotated.getY();
		xtest.points[1].z = xRotated.getZ();

		xtest.scale.x = 0.01;
		xtest.scale.y = 0.02;
		xtest.scale.z = 0.03;
		xtest.color.a = 0.3;
		xtest.color.r = 1;
		xtest.color.g = 0.0;
		xtest.color.b = 0.0;

		ytest = xtest;
		ytest.points[1].x = yRotated.getX();
		ytest.points[1].y = yRotated.getY();
		ytest.points[1].z = yRotated.getZ();
		ytest.color.r = 0;
		ytest.color.g = 1;
		ytest.id = 11;

		ztest = xtest;
		ztest.points[1].x = zRotated.getX();
		ztest.points[1].y = zRotated.getY();
		ztest.points[1].z = zRotated.getZ();
		ztest.color.r = 0;
		ztest.color.b = 1;
		ztest.id = 12;

		//vis_pub.publish(goal_pos);
		vis_pub.publish(xtest);
		vis_pub.publish(ytest);
		vis_pub.publish(ztest);*/
		/*
		std::cout << "xRotated: " << xRotated.getX() << " " << xRotated.getZ() << " " << xRotated.getZ() << std::endl;
		std::cout << "yRotated: " << yRotated.getX() << " " << yRotated.getZ() << " " << yRotated.getZ() << std::endl;
		std::cout << "zRotated: " << zRotated.getX() << " " << zRotated.getZ() << " " << zRotated.getZ() << std::endl;


		xRotated = transformFrameToTipLink * xRotated;
		yRotated = transformFrameToTipLink * yRotated;
		zRotated = transformFrameToTipLink * zRotated;

		std::cout << "xRotated: " << xRotated.getX() << " " << xRotated.getZ() << " " << xRotated.getZ() << std::endl;
		std::cout << "yRotated: " << yRotated.getX() << " " << yRotated.getZ() << " " << yRotated.getZ() << std::endl;
		std::cout << "zRotated: " << zRotated.getX() << " " << zRotated.getZ() << " " << zRotated.getZ() << std::endl;
		*/

		
		visualization_msgs::Marker xGoal;
		visualization_msgs::Marker yGoal;
		visualization_msgs::Marker zGoal;

		xGoal.header.frame_id = c.tip_link;
		xGoal.header.stamp = ros::Time();
		xGoal.ns = "rotation";
		xGoal.id = 3;
		xGoal.type = visualization_msgs::Marker::ARROW;
		xGoal.action = visualization_msgs::Marker::ADD;

		xGoal.scale.x = 0.01;
		xGoal.scale.y = 0.02;
		xGoal.scale.z = 0.03;
		xGoal.color.a = 0.3;
		xGoal.color.r = 1.0;
		xGoal.color.g = 0;
		xGoal.color.b = 0;

		xGoal.points.resize(2);
		xGoal.points[0].x = 0;
		xGoal.points[0].y = 0;
		xGoal.points[0].z = 0;

		xGoal.points[1].x = xRotated.getX();
		xGoal.points[1].y = xRotated.getY();
		xGoal.points[1].z = xRotated.getZ();

		yGoal = xGoal;
		yGoal.points[1].x = yRotated.getX();
		yGoal.points[1].y = yRotated.getY();
		yGoal.points[1].z = yRotated.getZ();
		yGoal.color.r = 0;
		yGoal.color.g = 1.0;
		yGoal.id = 4;

		zGoal = xGoal;
		zGoal.points[1].x = zRotated.getX();
		zGoal.points[1].y = zRotated.getY();
		zGoal.points[1].z = zRotated.getZ();
		zGoal.color.r = 0;
		zGoal.color.b = 1.0;
		zGoal.id = 5;


		vis_pub.publish(xGoal);
		vis_pub.publish(yGoal);
		vis_pub.publish(zGoal);
		

		visualization_msgs::Marker xAxis;
		visualization_msgs::Marker yAxis;
		visualization_msgs::Marker zAxis;
		
		xAxis.header.frame_id = c.tip_link;
		xAxis.header.stamp = ros::Time();
		xAxis.ns = "rotation";
		xAxis.id = 0;
		xAxis.type = visualization_msgs::Marker::ARROW;
		xAxis.action = visualization_msgs::Marker::ADD;

		xAxis.points.resize(2);
		xAxis.points[0].x = 0;
		xAxis.points[0].y = 0;
		xAxis.points[0].z = 0;

		xAxis.points[1].x = 0.1;
		xAxis.points[1].y = 0;
		xAxis.points[1].z = 0;

		xAxis.scale.x = 0.01;
		xAxis.scale.y = 0.02;
		xAxis.scale.z = 0.03;
		xAxis.color.a = 1.0;
		xAxis.color.r = 1.0;
		xAxis.color.g = 0.0;
		xAxis.color.b = 0.0;

		yAxis = xAxis;
		yAxis.points[1].x = 0;
		yAxis.points[1].y = 0.1;
		yAxis.color.r = 0;
		yAxis.color.g = 1.0;
		yAxis.id = 1;

		zAxis = xAxis;
		zAxis.points[1].x = 0;
		zAxis.points[1].z = 0.1;
		zAxis.color.r = 0;
		zAxis.color.b = 1.0;
		zAxis.id = 2;

		//vis_pub.publish(goal_pos);
		vis_pub.publish(xAxis);
		vis_pub.publish(yAxis);
		vis_pub.publish(zAxis);
		
		visualization_msgs::Marker rotationMakerX = getRotationMarker(tf::Vector3(0.1,0,0), xRotated, 0, c.tip_link);
		visualization_msgs::Marker rotationMakerY = getRotationMarker(tf::Vector3(0,0.1,0), yRotated, 1, c.tip_link);
		visualization_msgs::Marker rotationMakerZ = getRotationMarker(tf::Vector3(0,0,0.1), zRotated, 2, c.tip_link);
		vis_pub.publish(rotationMakerX);
		vis_pub.publish(rotationMakerY);
		vis_pub.publish(rotationMakerZ);

		visualization_msgs::Marker arrowTipX;
		arrowTipX.header.frame_id = c.tip_link;
		arrowTipX.header.stamp = ros::Time();
		arrowTipX.ns = "rotation";
		arrowTipX.id = 10;
		arrowTipX.type = visualization_msgs::Marker::ARROW;
		arrowTipX.action = visualization_msgs::Marker::ADD;

		arrowTipX.points.resize(2);
		arrowTipX.points[0] = rotationMakerX.points[rotationMakerX.points.size() - 2];//[&rotationMakerX]{rotationMakerX.}

		arrowTipX.points[1] = rotationMakerX.points[rotationMakerX.points.size() - 1];

		arrowTipX.scale.x = 0.0;
		arrowTipX.scale.y = 0.02;
		arrowTipX.scale.z = 0.05;
		arrowTipX.color.a = 1.0;
		arrowTipX.color.r = 1.0;
		arrowTipX.color.g = 0.7;
		arrowTipX.color.b = 0.0;

		vis_pub.publish(arrowTipX);

	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

visualization_msgs::Marker GiskardVisualizer::getRotationMarker(tf::Vector3 axis, tf::Vector3 goal, int id, std::string frame_id){
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
	marker.scale.x = 0.005;
	marker.scale.z = 0.005;
	marker.scale.y = 0.005;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.7;
	marker.color.b = 0.0;
	marker.header.stamp = ros::Time();
	marker.ns = "newrotation";
	marker.id = id;
	marker.header.frame_id = frame_id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	return marker;
}

void GiskardVisualizer::visualizeJointcommand(giskard_msgs::Controller &c){
	std::cout << "joint" << std::endl;


	bool exit_value;
	KDL::Chain chain;
	exit_value = my_tree.getChain(c.root_link, c.tip_link, chain);

	KDL::ChainFkSolverPos_recursive fksolver(chain);

	unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions(nj);

    int jointIdx = 0;

    for(int segmentIdx = 0; segmentIdx < chain.segments.size()-1; segmentIdx++){
    	KDL::Joint j = chain.segments[segmentIdx].getJoint();
    	if(j.getType() == KDL::Joint::None){
    		continue;
    	}

    	std::string name = j.getName();
    	std::cout << "name " << name << std::endl;
    	auto it = current_joint_state.find(name);
    	if(it != current_joint_state.end()){
    		jointpositions(jointIdx) = it->second;
    		 KDL::Frame cartpos;
  			 bool kinematics_status;
   			 kinematics_status = fksolver.JntToCart(jointpositions, cartpos, segmentIdx+1);
   			 std::cout << "cartpos x " << cartpos.p.x() << " y " << cartpos.p.y() << " z " << cartpos.p.z() << std::endl;
   			 int newJointStateIdx = std::find(c.goal_state.name.begin(), c.goal_state.name.end(), name) - c.goal_state.name.begin();
   			 if(newJointStateIdx >= c.goal_state.name.size()){
   			 	std::cout << "not found" << std::endl;
   			 }
   			 double newJointState = c.goal_state.position[newJointStateIdx];

   			 visualization_msgs::Marker marker = getJointMarker(cartpos, chain.segments[segmentIdx], it->second, newJointState, jointIdx, c.root_link);
   			 vis_pub.publish(marker);

    		jointIdx++;
    		std::cout << "state: " << it->second << std::endl;
    	}else{
    		std::cout << "joint not found" << std::endl;
    	}
	}

	/*std::cout << "asdf " << chain.segments.size() << std::endl;
	for(int i=0; i < chain.segments.size(); i++){
		std::cout << i << ": " << chain.segments[i].getJoint().getName() << "  " << jointpositions(i) << std::endl;
		//std::cout << i << ":i " << v[i].getJoint().getName() << std::endl;
	}*/



    /*for(int i=0; i<nj; i++){
    	jointpositions(i) = c.goal_state.position[i+1];
    	std::cout << i << " " << c.goal_state.position[i+1] << "  " << v[i].getJoint().getName() << std::endl;
    }*/
	
}

visualization_msgs::Marker GiskardVisualizer::getJointMarker(KDL::Frame jointFrame, KDL::Segment segment, double current_joint_state, double new_joint_state, int id, std::string frame_id){
	visualization_msgs::Marker marker;

	KDL::Vector rotationAxisKDL = segment.getJoint().JointAxis();

	double jointStateDiff = new_joint_state - current_joint_state;
	double stepSize = jointStateDiff/10;
	double step = 0;

	if(rotationAxisKDL.x() == 1){
		for(int i = 0; i < 10; i++, step+=stepSize){
			geometry_msgs::Point p;
			
			double vx = 0;
			double vy = 0.15 * cos(step);
			double vz = 0.15 * sin(step);

			double x;
			double y;
			double z;
			double w;

			jointFrame.M.GetQuaternion(x,y,z,w);

			tf::Vector3 v(vx,vy,vz);
			tf::Transform t(tf::Quaternion(x,y,z,w), tf::Vector3(jointFrame.p.x(), jointFrame.p.y(), jointFrame.p.z()));
			v = t * v;
			p.x = v.getX();
			p.y = v.getY();
			p.z = v.getZ();
			marker.points.push_back(p);
		}
	}

	if(rotationAxisKDL.y() == 1){
		for(int i = 0; i < 10; i++, step+=stepSize){
			geometry_msgs::Point p;
			
			double vx = 0.2 * cos(step);
			double vy = 0;
			double vz = 0.2 * -sin(step);

			double x;
			double y;
			double z;
			double w;

			jointFrame.M.GetQuaternion(x,y,z,w);

			tf::Vector3 v(vx,vy,vz);
			tf::Transform t(tf::Quaternion(x,y,z,w), tf::Vector3(jointFrame.p.x(), jointFrame.p.y(), jointFrame.p.z()));
			v = t * v;
			p.x = v.getX();
			p.y = v.getY();
			p.z = v.getZ();
			marker.points.push_back(p);
		}
	}


	/*for(int i = 0; i < 36; i++){
		geometry_msgs::Point p;
		
		double vx = 0.2 * sin(0.2*i);
		double vy = 0.2 * cos(0.2*i);
		double vz = 0.2 * sin(0.2*i);

		double x;
		double y;
		double z;
		double w;

		jointFrame.M.GetQuaternion(x,y,z,w);

		tf::Vector3 v(vx,vy,vz);
		tf::Transform t(tf::Quaternion(x,y,z,w), tf::Vector3(jointFrame.p.x(), jointFrame.p.y(), jointFrame.p.z()));
		v = t * v;
		p.x = v.getX();
		p.y = v.getY();
		p.z = v.getZ();
		marker.points.push_back(p);
	}*/
	

	marker.scale.x = 0.02;
	marker.scale.z = 0.02;
	marker.scale.y = 0.02;
	marker.color.a = 0.5;
	marker.color.r = 1.0;
	marker.color.g = 0.8;
	marker.color.b = 0.0;
	marker.header.stamp = ros::Time();
	marker.ns = "joint_vis";
	marker.id = id;
	marker.header.frame_id = frame_id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	return marker;
}

void GiskardVisualizer::visualizeController(const giskard_msgs::ControllerListGoalConstPtr	& msg){
	for(giskard_msgs::Controller c : msg->controllers){
		switch (c.type){
			case giskard_msgs::Controller::TRANSLATION_3D:
				visualizeTranslation(c);
				break;
			case giskard_msgs::Controller::JOINT:
				visualizeJointcommand(c);
				break;
			case giskard_msgs::Controller::ROTATION_3D:
				visualizeRotation(c);
				break;
			default:
				std::cout << "default" << std::endl;
				break;
		}
	}
}

/*void GiskardVisualizer::updateJointState(std::map<std::string, double> new_joint_state){
	current_joint_state = new_joint_state;
}*/


GiskardVisualizer::~GiskardVisualizer(){
	
}