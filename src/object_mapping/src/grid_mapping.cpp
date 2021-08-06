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
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

class GridMapping{

private:
  ros::NodeHandle nh;
  ros::Publisher grid_mapper;

public:
  GridMapping(){
    grid_mapper = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  }
  ~GridMapping(){}

  void main()
  {
    // Create grid map
    GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
    map.setFrameId("map");
    map.setGeometry(Length(500, 500), 3.0, Position(0.0, 0.0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
      map.getLength().x(), map.getLength().y(),
      map.getSize()(0), map.getSize()(1),
      map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

    ros::Rate rate(30.0);
    while(nh.ok()){
      ros::Time time = ros::Time::now();

      // iterating through grid map and adding data
      for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("elevation", *it) = 1;
        Eigen::Vector3d normal(1, 1, 1);

        normal.normalize();
        map.at("normal_x", *it) = normal.x();
        map.at("normal_y", *it) = normal.y();
        map.at("normal_z", *it) = normal.z();
      }

      // Publish grid map.
      map.setTimestamp(time.toNSec());
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(map, message);
      grid_mapper.publish(message);
      ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

      rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_mapping");

    GridMapping gm;
    gm.main();

    return 0;
}
