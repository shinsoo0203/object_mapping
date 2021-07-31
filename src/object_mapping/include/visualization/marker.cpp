#include "marker.h"

visualization_msgs::Marker Marker::point_marker(geometry_msgs::Point target_point){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns="";
    marker.id=0;
    marker.type=visualization_msgs::Marker::SPHERE;
    marker.action=visualization_msgs::Marker::ADD;

    marker.pose.position = target_point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.a = 1.0; //visible
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;

    marker.lifetime = ros::Duration();
    return marker;
}

visualization_msgs::Marker Marker::pose_marker(geometry_msgs::Pose target_pose){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns="";
    marker.id=0;
    marker.type=visualization_msgs::Marker::SPHERE;
    marker.action=visualization_msgs::Marker::ADD;

    marker.pose = target_pose;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 5;
    marker.scale.y = 5;
    marker.scale.z = 5;

    marker.color.a = 1.0; //visible
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;

    marker.lifetime = ros::Duration();
    return marker;
}
