/*
 *  Copyright (C) 2017 Vision-Guided and Intelligent Robotics Lab
 *  Written by Ali Shafiekhani <Ashafiekhani@mail.missouri.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation. Meaning:
 *          keep this copyright notice,
 *          do  not try to make money out of it,
 *          it's distributed WITHOUT ANY WARRANTY,
 *          yada yada yada...
 *
 *  You can get a copy of the GNU General Public License by writing to
 *  the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 *
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv,"head_eye_calibration");
	ros::NodeHandle nh;
	
	// Transformation from box to robot(from launch file)
	// TODO: Add launch file
	double transX, transY, transZ;
	double rollMtoR, pitchMtoR, yawMtoR;
	nh.param<double>("transX", transX, 0.75);	// transX is distance (robot coordinate) from marker to center of robot
	nh.param<double>("transY", transY, 0);	// transY is lateral distance from center of marker to center of robot
	nh.param<double>("transZ", transZ, 0.157);	// transZ is height of marder respect to ground

	nh.param<double>("roll" , rollMtoR, M_PI/2);	// roll is rotation around x
	nh.param<double>("pitch", pitchMtoR, 0.0);	// pitch is rotation around y
	nh.param<double>("yaw"  , yawMtoR, -M_PI/2);		// yaw is rotation around z

	tf::Transform rTb;
	rTb.setOrigin(tf::Vector3( transX, transY, transZ));
	tf::Quaternion q;
	q.setRPY(rollMtoR,pitchMtoR,yawMtoR);
	rTb.setRotation(q);

	tf::TransformListener ar_tool_listener;
	tf::Transform rTc;
	int i = 0;
	while(nh.ok()){
		tf::StampedTransform cTb;
		try{
		//	ar_tool_listener.waitForTransform("camera", "ar_marker",  
        //						ros::Time(0), ros::Duration(5.0));
			ar_tool_listener.lookupTransform("camera", "ar_marker",  
        						ros::Time(0), cTb);
			rTc = rTb * cTb.inverse();
			if(i == 10){
				tf::Matrix3x3 m(rTc.getRotation());
				double rollCtoR, pitchCtoR, yawCtoR;
				m.getRPY(rollCtoR, pitchCtoR, yawCtoR);
				ROS_INFO_STREAM("X: " << rTc.getOrigin().getX() << ", " <<
								"Y: " << rTc.getOrigin().getY() << ", " <<
								"Z: " << rTc.getOrigin().getZ() << ", | " <<
								"roll: "  << rollCtoR << ", " <<
								"pitch: " << pitchCtoR << ", " <<
								"yaw: "   << yawCtoR );
				break;
			}
			i++;
		}
		catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}

	return 0;
}
