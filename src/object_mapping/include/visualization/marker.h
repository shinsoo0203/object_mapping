#ifndef __MARKER__
#define __MARKER__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class Marker{

private:

public:
  visualization_msgs::Marker point_marker(geometry_msgs::Point target_point);
  visualization_msgs::Marker pose_marker(geometry_msgs::Pose target_pose);
  visualization_msgs::Marker pose_marker(geometry_msgs::Pose target_pose, int id);
  visualization_msgs::Marker pose_marker(geometry_msgs::Pose target_pose, int id, string color);
};

#endif
