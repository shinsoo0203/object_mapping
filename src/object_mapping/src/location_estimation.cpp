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

#include "enu_conversion/enu_conversion.cpp"
#include "pose_marker.h"

// konkuk reference
const double origin_lat_deg = 37.5413302340118;
const double origin_lon_deg = 127.0761387792444;

class LocationEstimation{

private:
  ros::NodeHandle nh;

  ros::Subscriber object_bboxes_sub;
  ros::Subscriber vehicle_gps_sub;

  ros::Publisher vehicle_marker_pub;
  ros::Publisher ref_marker_pub;

  ENU_Conversion enu_conversion;

public:
  LocationEstimation()
  {
    object_bboxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>\
        ("/darknet_ros/bounding_boxes", 10, &LocationEstimation::ObjectBBoxCb, this);
    vehicle_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>\
        ("/ublox_gps/fix",10, &LocationEstimation::VehicleGPSCb,this);

    vehicle_marker_pub = nh.advertise<visualization_msgs::Marker>("/vehicle_marker", 10);

    enu_conversion.setOrigin(origin_lat_deg, origin_lon_deg);
  }
  ~LocationEstimation(){}

  void ObjectBBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
  {
    darknet_ros_msgs::BoundingBoxes object_bboxes;
    object_bboxes=*msg;
  }

  void VehicleGPSCb(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    geometry_msgs::Pose geo_vehicle; //geodetic, global
    geo_vehicle.position.x = msg->longitude;
    geo_vehicle.position.y = msg->latitude;
    //geo_vehicle.position.z = 0;

    geometry_msgs::Pose local_vehicle; //local ENU
    local_vehicle = enu_conversion.enuConversion(geo_vehicle);
    vehicle_marker_pub.publish(pose_marker(local_vehicle));
  }

  void main()
  {
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
