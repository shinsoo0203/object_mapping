/**
 * @file location_estimation.cpp
 * @author IDPLab sooyeon Shin
 * Object location estimation with stereo image and darknet_ros
 */

#include <ros/ros.h>
#include <iostream>

class LocationEstimation{

private:
  ros::NodeHandle nh;

public:
  LocationEstimation(){
  }
  ~LocationEstimation(){}

  void main(){

  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "location_estimation");
    LocationEstimation le;
    le.main();
    return 0;
}
