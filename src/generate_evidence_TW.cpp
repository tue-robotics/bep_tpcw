/*
 * generate_evidence.cpp
 *
 *  Created on: July 1, 2020
 *      Author: T.P.C. Wester t.p.c.wester@student.tue.nl
 */
#include <random>
#include <iostream>

#include <ros/ros.h>

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include "problib/conversions.h"

#include <visualization_msgs/Marker.h>

// Define Publisher
ros::Publisher world_evidence_publisher_; // used to send evidence to world model
ros::Publisher vis_pub;                   // Used to generate visual object

// Define initial parameters person
ros::Time time_last_cycle_;
double x_person = 0, y_person = 1.5, vel_person = 0.5; // Start position and velocity person
double noiseSTD = 0.01 ; // STD of the noise on position measurement
int switch_ID = 0      ;
double delay = 0       ;

// Define initial FP parameters
double x_personFP, y_personFP; // Parameters containing position False Positives
double labdaFP = 0.2         ; // Poisson arrival rate false positives per second
ros::Time newArrivalTimeFP(0);

void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const std::string& class_label, const std::string& color) {
	wire_msgs::ObjectEvidence obj_evidence;

	// Set the continuous position property
	wire_msgs::Property posProp;
	posProp.attribute = "position";

	// Set position (x,y,z), set the covariance matrix as 0.005*identity_matrix
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0005, 0.0005, 0.0005)), posProp.pdf);
	obj_evidence.properties.push_back(posProp);

	// Set the discrete class label property
	wire_msgs::Property classProp;
	classProp.attribute = "class_label";
	pbl::PMF classPMF;

	// Probability of the class label is 0.7
	classPMF.setProbability(class_label, 0.7);
	pbl::PDFtoMsg(classPMF, classProp.pdf);
	obj_evidence.properties.push_back(classProp);

	// Set the discrete color property
	wire_msgs::Property colorProp;
	colorProp.attribute = "color";
	pbl::PMF colorPMF;

	// The probability of the detected color is 0.9
	colorPMF.setProbability(color, 0.9);
	pbl::PDFtoMsg(colorPMF, colorProp.pdf);
	obj_evidence.properties.push_back(colorProp);

//    // Set the discrete name property
//    wire_msgs::Property nameProp;
//    nameProp.attribute = "name";
//    pbl::PMF namePMF;
//
//    // The probability of the detected name is 0.9
//    namePMF.setProbability(name, 0.9);
//    pbl::PDFtoMsg(namePMF, nameProp.pdf);
//    obj_evidence.properties.push_back(nameProp);

	// Add all properties to the array
	world_evidence.object_evidence.push_back(obj_evidence);
}


void generateEvidence() {

    // Define path person
    ros::Time time_now = ros::Time::now();
    double dt = (time_now - time_last_cycle_).toSec();

    // Random sets
    std::random_device det_rd{}                        ; //
    std::mt19937 det_gen{det_rd()}                     ; //
    std::normal_distribution<> detNoise{0,noiseSTD}    ; // Gaussian noise mean = 0, std = noiseSTD
    std::exponential_distribution<double> exp(labdaFP) ; // Poisson arrival time for false positives
    std::uniform_real_distribution<> dis(0, 2.0)       ; // Uniform distribution over the room

    switch(switch_ID){
        case 0: // Wait for rviz to start up
            x_person += 0 + detNoise(det_gen);
            y_person += 0 + detNoise(det_gen);
            delay += dt;
            if(delay > 4){
                delay = 0;
                switch_ID = 1;
            }
            break;
        case 1: // Move in positive x-direction
            x_person += dt * vel_person + detNoise(det_gen);
            y_person += 0               + detNoise(det_gen);
            if(x_person > 1.5){
                switch_ID = 2;
            }
            break;
        case 2: // Turn
            x_person += std::cos(delay * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person += std::sin(delay * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay += dt;
            if(delay > 1){
                delay = 0;
                switch_ID = 3;
            }
            break;
        case 3: // Move in negative y-direction
            x_person += 0;
            y_person += -dt * vel_person;
            if(y_person < -1.2){
                switch_ID = 4;
            }
            break;
        case 4: // Turn
            x_person += -std::sin(delay * 0.5 * M_PI) *  dt * vel_person + detNoise(det_gen);
            y_person +=  std::cos(delay * 0.5 * M_PI) * -dt * vel_person + detNoise(det_gen);
            delay += dt;
            if(delay > 1){
                delay = 0;
                switch_ID = 5;
            }
            break;
        case 5: // Move in negative x-direction
            x_person += -dt * vel_person + detNoise(det_gen);
            y_person += 0                + detNoise(det_gen);
            if(x_person < 0){
                switch_ID = 6;
            }
            break;
        case 6: // End position
            x_person += 0 + detNoise(det_gen);
            y_person += 0 + detNoise(det_gen);
            break;
    }

	// Create world evidence message
	wire_msgs::WorldEvidence world_evidence;

	// Set header
	world_evidence.header.stamp = ros::Time::now();
	world_evidence.header.frame_id = "/map";

	// Add evidence person
	if(y_person < -0.55 || y_person > 0.55) {
        addEvidence(world_evidence, x_person, y_person, 1, "person", "red");
	}

    // Generate false positives
    if (time_now > newArrivalTimeFP){
        x_personFP = dis(det_gen);
        y_personFP = dis(det_gen);
        if((y_personFP < -0.55 || y_personFP > 0.55) && x_personFP < 1.3) {
            addEvidence(world_evidence, x_personFP, y_personFP, 1, "person", "");
            newArrivalTimeFP = time_now + ros::Duration(exp(det_gen));
        }
    }

	// Publish results
	world_evidence_publisher_.publish(world_evidence);
    time_last_cycle_ = time_now;

	ROS_INFO("Published world evidence with size %d", world_evidence.object_evidence.size());

}

// Generate visual map with closet
void visualMap() {
    visualization_msgs::Marker closet;
    closet.header.frame_id = "/map";
    closet.header.stamp = ros::Time::now();
    closet.ns = "Closet";
    closet.id = 0;
    closet.type = visualization_msgs::Marker::CUBE;
    closet.action = visualization_msgs::Marker::ADD;
    closet.pose.position.x = 1.5;
    closet.pose.position.y = 0;
    closet.pose.position.z = 1;
    closet.pose.orientation.x = 0.0;
    closet.pose.orientation.y = 0.0;
    closet.pose.orientation.z = 0.0;
    closet.pose.orientation.w = 1.0;
    closet.scale.x = 0.2;
    closet.scale.y = 1.0;
    closet.scale.z = 2.0;
    closet.color.a = 1.0;
    closet.color.r = 0.5;
    closet.color.g = 0.3;
    closet.color.b = 0.0;
    vis_pub.publish( closet );
}

/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"generate_evidence");
	ros::NodeHandle nh;

	// Publishers
	world_evidence_publisher_ = nh.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "/visual_map", 0 );

	// Publish with 3 Hz
	ros::Rate r(10.0);

    time_last_cycle_ = ros::Time::now();
	while (ros::ok()) {
		generateEvidence();
		visualMap();
		r.sleep();
	}

	return 0;
}