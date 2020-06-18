//
// Created by tpcw on 26-05-20.
//

#ifndef WIRE_ROOM1_H
#define WIRE_ROOM1_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>

class Room {
public:
    // Constructor
    Room(int n);

    virtual ~Room();

    void publish();

protected:
    // Define which room is being published
    int Room_numb;

    // Create marker id to prevent objects from having the same id
    int marker_id;

    // Ros publisher
    ros::Publisher vis_pub;

    // Ros subscriber
    ros::Subscriber OOS_subscriber_;

    visualization_msgs::MarkerArray Room_MarkerArray;

    visualization_msgs::Marker Closet(double pos_x, double pos_y, double scale_x, double scale_y);
    visualization_msgs::Marker Floor(double pos_x, double pos_y, double scale_x, double scale_y);
    visualization_msgs::Marker Wall(double pos_x, double pos_y, double scale_x, double scale_y);
    visualization_msgs::Marker Sensor(double pos_x, double pos_y);
    visualization_msgs::Marker AreaOOS(double pos_x, double pos_y, double scale_x, double scale_y);

    void initialize();

    void OOSRangeCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    double OOS_mean_x_, OOS_mean_y_, OOS_scale_x_, OOS_scale_y_;

    void Room1();

    void Room2();

};


#endif //WIRE_ROOM1_H
