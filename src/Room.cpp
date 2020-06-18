//
// Created by tpcw on 26-05-20.
//

#include "Room.h"

Room::Room(int n) : Room_numb(n), marker_id(1) {
    initialize();

    // Define room
    if(Room_numb == 1) {
        Room1();
    } else if(Room_numb == 2) {
        Room2();
    } else {
        ROS_ERROR("The room does not exist");
    }
    return;
}

Room::~Room() {
    OOS_subscriber_.shutdown();
}

void Room::initialize() {
    // Set node handle
    ros::NodeHandle n;

    // Publishers
    vis_pub = n.advertise<visualization_msgs::MarkerArray>( "/visual_map", 100 );

    // Subscribers
    OOS_subscriber_ = n.subscribe("/OOS_range", 100, &Room::OOSRangeCallback, this);

    return;
}

void Room::publish() {
    vis_pub.publish( Room_MarkerArray );
    return;
}

void Room::OOSRangeCallback(const std_msgs::Float32MultiArray::ConstPtr& array) {
    OOS_mean_x_ = (array->data.at(0) + array->data.at(1)) / 2;
    OOS_mean_y_ = (array->data.at(2) + array->data.at(3)) / 2;
    OOS_scale_x_ = array->data.at(1) - array->data.at(0);
    OOS_scale_y_ = array->data.at(3) - array->data.at(2);

    Room_MarkerArray.markers.push_back(AreaOOS(OOS_mean_x_, OOS_mean_y_, OOS_scale_x_, OOS_scale_y_));
    return;
}

// Generate visual map with single closet
void Room::Room1() {
    Room_MarkerArray.markers.push_back(Closet(1.5, 0, 0.2, 1.0));
    Room_MarkerArray.markers.push_back(Floor(1.0, 0.0, 4.0, 4.0));
    Room_MarkerArray.markers.push_back(Wall(2.95, 0.0, 0.1, 4.0));
    Room_MarkerArray.markers.push_back(Wall(1.0, 1.95, 4.0, 0.1));
    Room_MarkerArray.markers.push_back(Wall(1.0, -1.95, 4.0, 0.1));
    Room_MarkerArray.markers.push_back(Sensor(-1.0, 0.0));
    return;
}

void Room::Room2() {
    Room_MarkerArray.markers.push_back(Floor(1.0, 0.0, 4.0, 8.0));
    Room_MarkerArray.markers.push_back(Wall(2.95, 0.0, 0.1, 8.0));
    Room_MarkerArray.markers.push_back(Wall(1.0, 3.95, 4.0, 0.1));
    Room_MarkerArray.markers.push_back(Wall(1.0, -3.95, 4.0, 0.1));
    Room_MarkerArray.markers.push_back(Sensor(-1.0, 0.0));
    return;
}

visualization_msgs::Marker Room::Closet(double pos_x, double pos_y, double scale_x, double scale_y) {
    visualization_msgs::Marker Closet_marker;
    Closet_marker.header.frame_id = "/map";
    Closet_marker.header.stamp = ros::Time::now();
    Closet_marker.ns = "Closet";
    Closet_marker.id = marker_id;
    Closet_marker.type = visualization_msgs::Marker::CUBE;
    Closet_marker.action = visualization_msgs::Marker::ADD;
    Closet_marker.pose.position.x = pos_x;
    Closet_marker.pose.position.y = pos_y;
    Closet_marker.pose.position.z = 1;
    Closet_marker.pose.orientation.x = 0.0;
    Closet_marker.pose.orientation.y = 0.0;
    Closet_marker.pose.orientation.z = 0.0;
    Closet_marker.pose.orientation.w = 1.0;
    Closet_marker.scale.x = scale_x;
    Closet_marker.scale.y = scale_y;
    Closet_marker.scale.z = 2.0;
    Closet_marker.color.a = 1.0;
    Closet_marker.color.r = 0.5;
    Closet_marker.color.g = 0.3;
    Closet_marker.color.b = 0.0;
    marker_id += 1;
    return Closet_marker;
}

visualization_msgs::Marker Room::Floor(double pos_x, double pos_y, double scale_x, double scale_y) {
    visualization_msgs::Marker Floor_marker;
    Floor_marker.header.frame_id = "/map";
    Floor_marker.header.stamp = ros::Time::now();
    Floor_marker.ns = "Floor";
    Floor_marker.id = marker_id;
    Floor_marker.type = visualization_msgs::Marker::CUBE;
    Floor_marker.action = visualization_msgs::Marker::ADD;
    Floor_marker.pose.position.x = pos_x;
    Floor_marker.pose.position.y = pos_y;
    Floor_marker.pose.position.z = -0.05;
    Floor_marker.pose.orientation.x = 0.0;
    Floor_marker.pose.orientation.y = 0.0;
    Floor_marker.pose.orientation.z = 0.0;
    Floor_marker.pose.orientation.w = 1.0;
    Floor_marker.scale.x = scale_x;
    Floor_marker.scale.y = scale_y;
    Floor_marker.scale.z = 0.1;
    Floor_marker.color.a = 1.0;
    Floor_marker.color.r = 0.5;
    Floor_marker.color.g = 0.5;
    Floor_marker.color.b = 0.5;
    marker_id += 1;
    return Floor_marker;
}

visualization_msgs::Marker Room::Wall(double pos_x, double pos_y, double scale_x, double scale_y) {
    visualization_msgs::Marker Wall_marker;
    Wall_marker.header.frame_id = "/map";
    Wall_marker.header.stamp = ros::Time::now();
    Wall_marker.ns = "Wall";
    Wall_marker.id = marker_id;
    Wall_marker.type = visualization_msgs::Marker::CUBE;
    Wall_marker.action = visualization_msgs::Marker::ADD;
    Wall_marker.pose.position.x = pos_x;
    Wall_marker.pose.position.y = pos_y;
    Wall_marker.pose.position.z = 1;
    Wall_marker.pose.orientation.x = 0.0;
    Wall_marker.pose.orientation.y = 0.0;
    Wall_marker.pose.orientation.z = 0.0;
    Wall_marker.pose.orientation.w = 1.0;
    Wall_marker.scale.x = scale_x;
    Wall_marker.scale.y = scale_y;
    Wall_marker.scale.z = 2.0;
    Wall_marker.color.a = 1.0;
    Wall_marker.color.r = 1.0;
    Wall_marker.color.g = 1.0;
    Wall_marker.color.b = 1.0;
    marker_id += 1;
    return Wall_marker;
}

visualization_msgs::Marker Room::Sensor(double pos_x, double pos_y) {
    visualization_msgs::Marker Sensor_marker;
    Sensor_marker.header.frame_id = "/map";
    Sensor_marker.header.stamp = ros::Time::now();
    Sensor_marker.ns = "Sensor";
    Sensor_marker.id = marker_id;
    Sensor_marker.type = visualization_msgs::Marker::SPHERE;
    Sensor_marker.action = visualization_msgs::Marker::ADD;
    Sensor_marker.pose.position.x = pos_x;
    Sensor_marker.pose.position.y = pos_y;
    Sensor_marker.pose.position.z = 1;
    Sensor_marker.pose.orientation.x = 0.0;
    Sensor_marker.pose.orientation.y = 0.0;
    Sensor_marker.pose.orientation.z = 0.0;
    Sensor_marker.pose.orientation.w = 1.0;
    Sensor_marker.scale.x = 0.1;
    Sensor_marker.scale.y = 0.3;
    Sensor_marker.scale.z = 0.1;
    Sensor_marker.color.a = 1.0;
    Sensor_marker.color.r = 0.0;
    Sensor_marker.color.g = 0.0;
    Sensor_marker.color.b = 1.0;
    marker_id += 1;
    return Sensor_marker;
}

visualization_msgs::Marker Room::AreaOOS(double pos_x, double pos_y, double scale_x, double scale_y) {
    visualization_msgs::Marker AreaOOS_marker;
    AreaOOS_marker.header.frame_id = "/map";
    AreaOOS_marker.header.stamp = ros::Time::now();
    AreaOOS_marker.ns = "AreaOOS";
    AreaOOS_marker.id = marker_id;
    AreaOOS_marker.type = visualization_msgs::Marker::CUBE;
    AreaOOS_marker.action = visualization_msgs::Marker::ADD;
    AreaOOS_marker.pose.position.x = pos_x;
    AreaOOS_marker.pose.position.y = pos_y;
    AreaOOS_marker.pose.position.z = 1.0;
    AreaOOS_marker.pose.orientation.x = 0.0;
    AreaOOS_marker.pose.orientation.y = 0.0;
    AreaOOS_marker.pose.orientation.z = 0.0;
    AreaOOS_marker.pose.orientation.w = 1.0;
    AreaOOS_marker.scale.x = scale_x;
    AreaOOS_marker.scale.y = scale_y;
    AreaOOS_marker.scale.z = 0.02;
    AreaOOS_marker.color.a = 1.0;
    AreaOOS_marker.color.r = 1.0;
    AreaOOS_marker.color.g = 0.6;
    AreaOOS_marker.color.b = 0.2;
    return AreaOOS_marker;
}