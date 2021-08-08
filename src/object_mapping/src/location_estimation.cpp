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
#include <ublox_msgs/NavPVT.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "enu_conversion/enu_conversion.cpp"
#include "visualization/marker.cpp"

// konkuk reference
const double origin_lat_deg = 37.5413302340118;
const double origin_lon_deg = 127.0761387792444;

class LocationEstimation{

private:
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;
  tf::TransformListener ls;

  ros::Subscriber object_bboxes_sub;
  ros::Subscriber vehicle_gps_sub;
  ros::Subscriber vehicle_head_sub;

  ros::Publisher vehicle_local_pub;
  ros::Publisher vehicle_marker_pub;
  //ros::Publisher vehicle_markerArray_pub;

  geometry_msgs::Pose geo_vehicle; //geodetic, global
  geometry_msgs::Pose local_vehicle; //local ENU

  ENU_Conversion enu_conversion;
  Marker marker;
  int mark_num = 0;

public:
  LocationEstimation()
  {
    object_bboxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>\
        ("/darknet_ros/bounding_boxes", 10, &LocationEstimation::ObjectBBoxCb, this);
    vehicle_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>\
        ("/ublox_gps/fix",10, &LocationEstimation::VehicleGPSCb,this);
    vehicle_head_sub = nh.subscribe<ublox_msgs::NavPVT>\
            ("/ublox_gps/navpvt", 10, &LocationEstimation::VehicleHeadCb, this);

    vehicle_local_pub = nh.advertise<geometry_msgs::Pose>("/vehicle_local", 10);
    vehicle_marker_pub = nh.advertise<visualization_msgs::Marker>("/vehicle_marker", 10);
    //vehicle_markerArray_pub = nh.advertise<visualization_msgs::MarkerArray>("/vehicle_markerArray", 10);

    ls.waitForTransform("map", "vehicle", ros::Time::now(), ros::Duration(4,0));
    ls.waitForTransform("vehicle", "stereo", ros::Time::now(), ros::Duration(4,0));
    //ls.waitForTransform("map", "stereo", ros::Time::now(), ros::Duration(4,0));

    ROS_INFO("[Location Estimation]: started");
    enu_conversion.setOrigin(origin_lat_deg, origin_lon_deg);
  }
  ~LocationEstimation(){}

  //Callback
  void ObjectBBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
  {
    darknet_ros_msgs::BoundingBoxes object_bboxes;
    object_bboxes=*msg;
  }

  void VehicleGPSCb(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    geo_vehicle.position.x = msg->longitude;
    geo_vehicle.position.y = msg->latitude;
    geo_vehicle.position.z = 1.2; //msg->altitude;
  }

  void VehicleHeadCb(const ublox_msgs::NavPVTConstPtr& msg)
  {
      double heading = (msg->heading * pow(0.1, 5) - 90) * -1;

//      geo_vehicle.position.x = msg->lon * pow(0.1, 7);
//      geo_vehicle.position.y = msg->lat * pow(0.1, 7);
//      geo_vehicle.position.z = 0.5; //gps hegith

      tf::Quaternion vehicle_q;
      vehicle_q.setRPY(0, 0, heading* M_PI/180); //d2r
      geo_vehicle.orientation.x = vehicle_q[0];
      geo_vehicle.orientation.y = vehicle_q[1];
      geo_vehicle.orientation.z = vehicle_q[2];
      geo_vehicle.orientation.w = vehicle_q[3];

      local_vehicle = enu_conversion.enuConversion(geo_vehicle);
      vehicle_local_pub.publish(local_vehicle);

      visualization_msgs::Marker vehicle_mark = marker.pose_marker(local_vehicle, mark_num);
      vehicle_marker_pub.publish(vehicle_mark);
      mark_num ++;

      //Check multiple ROSBAG Paths
      //vehicle_markArray.markers.push_back(vehicle_mark);
      //vehicle_markerArray_pub.publish(vehicle_markArray);

      tf::Transform tf_vehicle;
      tf::poseMsgToTF(local_vehicle, tf_vehicle);
      br.sendTransform(tf::StampedTransform(tf_vehicle, ros::Time::now(), "map", "vehicle"));

      tf::Transform tf_cam;
      tf_cam.setOrigin(tf::Vector3(0.6, 0, 0));
      tf_cam.setRotation(tf::Quaternion(0,0,0,1));
      br.sendTransform(tf::StampedTransform(tf_cam, ros::Time(0), "vehicle", "stereo"));
  }

  void main()
  {
    ros::Rate rate(10.0);

    while(ros::ok())
    {
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
