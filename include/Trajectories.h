//
// Created by tpcw on 27-05-20.
//

#ifndef WIRE_GENERATE_EVIDENCE_H
#define WIRE_GENERATE_EVIDENCE_H

#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"
#include "problib/conversions.h"
#include <random>

class Trajectories {
public:
    // Constructor
    Trajectories(int n);

    void publish();

protected:
    // Define which situation is being published
    int Trajectories_numb;

    // Define initial parameters person
    double x_person1, y_person1;
    double x_person2, y_person2;
    double x_person3, y_person3;
    double noiseSTD;  // STD of the noise on position measurement
    double delay1, delay2, delay3;     // Delay variable used to define path
    int switch_ID1, switch_ID2, switch_ID3;    // Integer for switch case

    // Define initial FP parameters
    double x_personFP, y_personFP; // Parameters containing position False Positives
    double labdaFP               ; // Poisson arrival rate false positives per second
    ros::Time newArrivalTimeFP   ; // Time between two Poisson arrivals

    // Ros time parameters
    ros::Time time_now;
    ros::Time time_last_cycle_;
    double dt;

    // Ros publisher
    ros::Publisher world_evidence_publisher_;
    ros::Publisher OOS_publisher_; // Publishes region that is out of sight
    std_msgs::Float32MultiArray OOS_range_;

    // Random sets
    std::random_device det_rd{}   ; // Random device
    std::mt19937 det_gen{det_rd()}; // Random motor

    void initialize();
    void Situation1(wire_msgs::WorldEvidence& world_evidence);
    void Situation2(wire_msgs::WorldEvidence& world_evidence);
    void Situation3(wire_msgs::WorldEvidence& world_evidence);
    void Situation4(wire_msgs::WorldEvidence& world_evidence);
    void Situation5(wire_msgs::WorldEvidence& world_evidence);
    void Situation6(wire_msgs::WorldEvidence& world_evidence);
    void Situation7(wire_msgs::WorldEvidence& world_evidence);
    void FalsePositives(wire_msgs::WorldEvidence& world_evidence);
    void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const std::string& class_label, const std::string& color);
};


#endif //WIRE_GENERATE_EVIDENCE_H
