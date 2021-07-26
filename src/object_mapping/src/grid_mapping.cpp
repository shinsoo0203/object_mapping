/**
 * @file grid_mapping.cpp
 * @author IDPLab sooyeon Shin
 * Occupancy Grid Mapping with 3d object
 *
 * Function to convert 2D pixel point to 3D point by extracting point
   from PointCloud2 corresponding to input pixel coordinate. This function
   can be used to get the X,Y,Z coordinates of a feature using an
   RGBD camera, e.g., Kinect.
 */

#include <ros/ros.h>
#include <iostream>

class GridMapping{

private:

public:
  GridMapping(){
  }
  ~GridMapping(){}

  void main(){
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_mapping");
    GridMapping gm;
    gm.main();
    return 0;
}
