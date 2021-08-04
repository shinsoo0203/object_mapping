/**
 * @file grid_mapping.cpp
 * @author IDPLab sooyeon Shin
 * Occupancy Grid Mapping with 3d object information
 *
 * Function to creates a grid map in a particular region
   and finding the grid in which the collected dynamic objects are located.
   Also, the module runs on bigdata platform and handles all object information that is collect.
 *
 * input: Descriptors, including 3d bounding boxes
 * output: Send to Spark via Kafka
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
