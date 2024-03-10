#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// Create a grid map.
grid_map::GridMap map;
bool map_received = false;
ros::Publisher map_pub;

void mapCallback(const grid_map_msgs::GridMap& msg) {
    // Process the received map data here if needed.
    ROS_INFO("Map obtained 1");
    grid_map::GridMapRosConverter::fromMessage(msg, map);
    
    // Convert grid map to occupancy grid message.
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "elevation", -10.0, 10.0, message);

    // Publish the occupancy grid message.
    map_pub.publish(message);
    
    ROS_INFO("Map obtained 2");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_to_occ");
    ros::NodeHandle nh;

    // Read subscriber topic name from ROS parameter.
    std::string map_topic;
    ros::param::param<std::string>("~map_topic", map_topic, "grid_map_loader_demo/grid_map");
    ROS_INFO("Node started with map topic: %s",map_topic.c_str());

    // Create a subscriber for the occupancy grid map.
    ros::Subscriber map_sub = nh.subscribe(map_topic, 10, mapCallback);

    // Create a publisher for the occupancy grid map.
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occ_map", 1);
    ros::spin();
    ros::Rate rate(1);  // Adjust the rate if needed.
    //ROS_INFO("...");
    //int i=0;
    while (ros::ok()) {
        if(map_received) {
        }
        //ROS_INFO("%d",i);
        //i++;
        //ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
