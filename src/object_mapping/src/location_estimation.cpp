/**
 * @file location_estimation.cpp
 * @author IDPLab sooyeon Shin
 * Object location estimation with stereo image and darknet_ros
 *
 * Function to convert 2D pixel point to 3D point by extracting point
   from PointCloud2 corresponding to input pixel coordinate. This function
   can be used to get the X,Y,Z coordinates of a feature using an
   RGBD camera, e.g., Kinect.
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

class LocationEstimation{

private:
  ros::NodeHandle nh;
  ros::Subscriber object_bboxes_sub;

  darknet_ros_msgs::BoundingBoxes object_bboxes;

public:
  LocationEstimation(){
      object_bboxes_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>\
            ("/darknet_ros/bounding_boxes", 10, &LocationEstimation::ObjectBBoxCb, this);
  }
  ~LocationEstimation(){}

  void ObjectBBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg){
      object_bboxes=*msg;
  }

  void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p){
      // get width and height of 2D point cloud data
      int width = pCloud.width;
      int height = pCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

     // put data into the point p
      p.x = X;
      p.y = Y;
      p.z = Z;

  }

  void main(){
    ros::Rate rate(10.0);

    while(ros::ok()){
      for(int i=0; i<object_bboxes.bounding_boxes.size(); i++){
        std::cout<<object_bboxes;
      }
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
