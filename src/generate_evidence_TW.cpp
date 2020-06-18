/*
 * generate_evidence.cpp
 *
 *  Created on: July 1, 2020
 *      Author: T.P.C. Wester t.p.c.wester@student.tue.nl
 */
#include <iostream>
#include <ros/ros.h>
#include "Room.h"
#include "Trajectories.h"

int Room_int, Trajectories_int;

int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"generate_evidence_TW");
    ros::NodeHandle n("~");

    // set parameters
    n.getParam("/Room_int", Room_int);
    n.getParam("/Trajectories_int", Trajectories_int);

    Room Room_(Room_int);
    Trajectories Trajectories_(Trajectories_int);

	// Publish with 10 Hz
	ros::Rate r(10.0);

//    time_last_cycle_ = ros::Time::now();
	while (ros::ok()) {
        ros::spinOnce();
		Room_.publish();
        Trajectories_.publish();
		r.sleep();
	}

	return 0;
}