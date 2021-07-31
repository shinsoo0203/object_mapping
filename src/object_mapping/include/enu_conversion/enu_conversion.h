#ifndef __ENU_CONVERSION__
#define __ENU_CONVERSION__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

class ENU_Conversion{

private:
  double m_origin_lat_deg;
  double m_origin_lon_deg;

  double m_origin_lat_rad;
  double m_origin_lon_rad;

  const double m_a = 6378137.0;       // semi-major axis [m]
  const double m_b = 6356752.314245;  // semi-minor axis [m]

public:
  void setOrigin(double origin_lat, double origin_lon);
  double MeridionalRadius(double a, double b, double lat);
  double NormalRadius(double a, double b, double lat);

  geometry_msgs::Point enuConversion(const geometry_msgs::Point global_point);
  geometry_msgs::Pose enuConversion(const geometry_msgs::Pose global_pose);
  geometry_msgs::PoseArray enuConversion(const geometry_msgs::PoseArray globalArr);

};

#endif
