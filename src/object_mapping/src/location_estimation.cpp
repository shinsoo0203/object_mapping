/**
 * @file location_estimation.cpp
 * @author IDPLab sooyeon Shin
 *
 * Function to convert 2D pixel point to 3D point by extracting point
   from PointCloud2 corresponding to input pixel coordinate. This function
   can be used to get the X,Y,Z coordinates of a feature using an
   stereo camera, e.g., zed2.
   Also, generate data to send finally with the big data platform
 *
 * input: 2d bounding boxes via YOLO algorithm
 * output: Descriptors, including 3d bounding boxes
 *
 * Written with reference to IntelligentRoboticsLabs/gb_visual_detection_3d
 * Author: Francisco Martín fmrico@gmail.com
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <visualization_msgs/Marker.h>

#include "enu_conversion.h"
#include "pose_marker.h"

class LocationEstimation{

private:
  ros::NodeHandle nh;
  ros::Subscriber object_bboxes_sub;
  ros::Subscriber vehicle_gps_sub;
  ros::Publisher gps_marker_pub;

  darknet_ros_msgs::BoundingBoxes object_bboxes;

public:
  LocationEstimation(){
      object_bboxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>\
            ("/darknet_ros/bounding_boxes", 10, &LocationEstimation::ObjectBBoxCb, this);
      vehicle_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>\
              ("/ublox_gps/fix",10, &LocationEstimation::VehicleGPSCb,this);
      gps_marker_pub = nh.advertise<visualization_msgs::Marker>("/vehicle_gps/marker", 10);
  }
  ~LocationEstimation(){}

  void ObjectBBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg){
      object_bboxes=*msg;
  }

  void VehicleGPSCb(const sensor_msgs::NavSatFixConstPtr& msg){
      geometry_msgs::Pose geo_vehicle; //geodetic
      geo_vehicle.position.x = msg->longitude;
      geo_vehicle.position.y = msg->latitude;
      geo_vehicle.position.z = 0; //msg->altitude;

      geometry_msgs::Pose local_vehicle; //local ENU
      local_vehicle = enuConversion(geo_vehicle);

      gps_marker_pub.publish(pose_marker(local_vehicle));
  }

  void main(){
    ros::Rate rate(10.0);

    while(ros::ok()){
      ros::spinOnce();
      rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "location_estimation");
    LocationEstimation le;
    le.main();
    return 0;
}
