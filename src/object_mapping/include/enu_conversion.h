#ifndef __ENU_CONVERSION__
#define __ENU_CONVERSION__

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>

//konkuk reference
const double origin_lat_deg = 37.5413302340118;
const double origin_lon_deg = 127.0761387792444;

const double m_a = 6378137.0;       // semi-major axis [m]
const double m_b = 6356752.314245;  // semi-minor axis [m]

double d2r = M_PI/180; //degree to radian
double origin_lat_rad;
double origin_lon_rad;

double MeridionalRadius(double a, double b, double lat){
      return pow(a*b, 2) / sqrt(pow((pow(a*cos(lat), 2) + pow(b*sin(lat), 2)), 3));
}

double NormalRadius(double a, double b, double lat){
      return (a*a) / sqrt(pow(a*cos(lat), 2) + pow(b*sin(lat),2));
}

geometry_msgs::Pose enuConversion(const geometry_msgs::Pose global_pose){
      geometry_msgs::Pose local_pose;
      double lon_rad, lat_rad;
      lon_rad = global_pose.position.x * d2r;
      lat_rad = global_pose.position.y * d2r;

      local_pose.position.x = (lon_rad - origin_lon_rad)*(NormalRadius(m_a, m_b, lat_rad)*cos(lat_rad));
      local_pose.position.y = (lat_rad - origin_lat_rad)*MeridionalRadius(m_a, m_b, lat_rad);
      //local_pose.position.z = global_pose.position.z;
      local_pose.orientation = global_pose.orientation;

      return local_pose;
}

#endif
