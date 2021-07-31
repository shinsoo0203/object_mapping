# include "enu_conversion.h"

void ENU_Conversion::setOrigin(double origin_lat, double origin_lon)
{
  m_origin_lat_deg = origin_lat;
  m_origin_lat_rad = m_origin_lat_deg * M_PI/180.0;

  m_origin_lon_deg = origin_lon;
  m_origin_lon_rad = m_origin_lon_deg * M_PI/180.0;
}

geometry_msgs::Point ENU_Conversion::enuConversion(const geometry_msgs::Point global_point)
{
  geometry_msgs::Point local_point;

  double temp_lon;
  double temp_lat;

  temp_lon = global_point.x * M_PI / 180.0;
  temp_lat = global_point.y * M_PI / 180.0;

  local_point.x = (temp_lon - m_origin_lon_rad)*(NormalRadius(m_a, m_b, temp_lat)*cos(temp_lat));
  local_point.y = (temp_lat - m_origin_lat_rad)*MeridionalRadius(m_a, m_b, temp_lat);
  //local_point.z = global_point.z;

  ROS_INFO("[ENU Conversion] east : %f, north : %f", local_point.x, local_point.y);

  return local_point;
}

geometry_msgs::Pose ENU_Conversion::enuConversion(const geometry_msgs::Pose global_pose)
{
  geometry_msgs::Pose local_pose;

  double temp_lon;
  double temp_lat;

  temp_lon = global_pose.position.x * M_PI / 180.0;
  temp_lat = global_pose.position.y * M_PI / 180.0;

  local_pose.position.x = (temp_lon - m_origin_lon_rad)*(NormalRadius(m_a, m_b, temp_lat)*cos(temp_lat));
  local_pose.position.y = (temp_lat - m_origin_lat_rad)*MeridionalRadius(m_a, m_b, temp_lat);
  //local_pose.position.z = global_pose.position.z;
  local_pose.orientation.w = 1.0;

  ROS_INFO("[ENU Conversion] east : %f, north : %f", local_pose.position.x, local_pose.position.y);

  return local_pose;
}

geometry_msgs::PoseArray ENU_Conversion::enuConversion(const geometry_msgs::PoseArray globalArr)
{
  geometry_msgs::PoseArray localArr;

  for (int i_point = 0; i_point < globalArr.poses.size(); i_point++)
  {
    geometry_msgs::Pose local_pose;
    local_pose = enuConversion(globalArr.poses[i_point]);
    localArr.poses.push_back(local_pose);

    ROS_INFO("east : %f, north : %f", local_pose.position.x, local_pose.position.y);
  }
  return localArr;
}

double ENU_Conversion::MeridionalRadius(double a, double b, double lat){
  return pow(a*b, 2) / sqrt( pow((pow( a*cos(lat), 2) + pow( b*sin(lat), 2 )), 3));
}

double ENU_Conversion::NormalRadius(double a, double b, double lat){
  return (a*a) / sqrt(pow( a*cos(lat), 2 ) + pow( b*sin(lat), 2));
}
