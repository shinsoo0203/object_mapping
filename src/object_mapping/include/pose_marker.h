#ifndef __POSE_MARKER__
#define __POSE_MARKER__

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker pose_marker(geometry_msgs::Pose target){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns="";
    marker.id=0;
    marker.type=visualization_msgs::Marker::SPHERE;
    marker.action=visualization_msgs::Marker::ADD;

    marker.pose.position = target.position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.a = 1.0; //visible
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;

    marker.lifetime = ros::Duration();
    return marker;
}

#endif
