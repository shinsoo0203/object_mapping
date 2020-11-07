/**
 * @file object_mapping.cpp
 * @author IDPLab sooyeon Shin
 */

#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/NavSatFix.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectPoint.h>
#include <darknet_ros_msgs/ObjectArray.h>

#include <visualization_msgs/Marker.h>

//OCAM-U
const double fx = 627.407948;
const double fy = 629.163305;
const double cx = 280.506703;
const double cy = 229.733459;

const double m_a = 6378137.0;       // semi-major axis [m]
const double m_b = 6356752.314245;  // semi-minor axis [m]
const double origin_lat_deg = 37.5413302340118;
const double origin_lon_deg = 127.0761387792444;

class DObjectMapping{

private:
  ros::NodeHandle nh;
  ros::Subscriber vehicle_pose_sub;
  ros::Subscriber obj_counted_sub;
  ros::Subscriber obj_detected_sub;
  ros::Publisher map_obj_pose_pub;
  ros::Publisher obj_marker_pub;

  geometry_msgs::Point obj_pixel;
  geometry_msgs::Point obj_ground;

  tf::TransformBroadcaster br;
  tf::TransformListener ls;
  tf::StampedTransform tf_fcu;
  tf::Transform tf_cam;

  tf::Quaternion q_fcu;
  tf::Quaternion q_cam;
  tf::Vector3 t;
  cv::Mat R, T;

  bool vehicle_pose_exist = false;
  double origin_lat_rad;
  double origin_lon_rad;
  double d2r = M_PI/180; //degree to radian
  double x_gap = 1;
  double y_gap;
  double z_gap;

  int obj_count;
  darknet_ros_msgs::BoundingBoxes _obj_boxes;
  darknet_ros_msgs::BoundingBoxes obj_boxes; //**
  geometry_msgs::Pose _vehicle_pose;    //global
  geometry_msgs::Pose vehicle_pose;     //local

public:
  DObjectMapping(){
      vehicle_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>\
              ("/ublox_gps/fix", 10, &DObjectMapping::VehiclePoseCb, this);
      obj_counted_sub = nh.subscribe<darknet_ros_msgs::ObjectCount>\
              ("/darknet_ros/yolo_object_count", 10,&DObjectMapping::ObjectCountedCb, this);
      obj_detected_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>\
              ("/darknet_ros/yolo_object_bboxes", 10, &DObjectMapping::ObjectDetectedCb, this);

      map_obj_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/detection/map_obj_pose",10);
      obj_marker_pub = nh.advertise<visualization_msgs::Marker>("/detection/obj_marker", 10);

      init();
  }
  ~DObjectMapping(){}

  void init(){
      ros::Rate rate(20.0);

      origin_lat_rad = origin_lat_deg * d2r;
      origin_lon_rad = origin_lon_deg * d2r;

      ROS_INFO("[Mapping] Waiting for local pose...");
      while(ros::ok()){
          if(vehicle_pose_exist){
              ROS_INFO("[Mapping] Received local pose. Start object mapping.");
              break;
          }
          ros::spinOnce();
          rate.sleep();
      }

      ls.waitForTransform("map","vehicle",ros::Time::now(),ros::Duration(0.1));
      ls.waitForTransform("map","camera",ros::Time::now(),ros::Duration(0.1));
  }

  // Callback
  void VehiclePoseCb(const sensor_msgs::NavSatFixConstPtr& msg){
      if(!vehicle_pose_exist) vehicle_pose_exist = true;
      _vehicle_pose.position.x = msg->longitude;
      _vehicle_pose.position.y = msg->latitude;
      _vehicle_pose.position.z = msg->altitude;

      vehicle_pose = enuConversion(_vehicle_pose);
      tf::Transform transform;
      tf::poseMsgToTF(vehicle_pose, transform);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "vehicle"));
  }
  void ObjectCountedCb(const darknet_ros_msgs::ObjectCountConstPtr& msg){
      obj_count = msg->count;
      if(obj_count>0) core();
  }
  void ObjectDetectedCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg){
      _obj_boxes = *msg;
  }

  // Definition
  geometry_msgs::Pose enuConversion(const geometry_msgs::Pose global_pose){
      geometry_msgs::Pose local_pose;
      double lon_rad, lat_rad;
      lon_rad = global_pose.position.x * d2r;
      lat_rad = global_pose.position.y * d2r;

      local_pose.position.x = (lon_rad - origin_lon_rad)*(NormalRadius(m_a, m_b, lat_rad)*cos(lat_rad));
      local_pose.position.y = (lat_rad - origin_lat_rad)*MeridionalRadius(m_a, m_b, lat_rad);
      local_pose.orientation.w = 1.0;

      return local_pose;
  }
  double MeridionalRadius(double a, double b, double lat){
      return pow(a*b, 2) / sqrt(pow((pow(a*cos(lat), 2) + pow(b*sin(lat), 2)), 3));
  }
  double NormalRadius(double a, double b, double lat){
      return (a*a) / sqrt(pow(a*cos(lat), 2) + pow(b*sin(lat),2));
  }

  // Coordinate Transform
  void getTFvalue(){
      //camera location from gps
      t[0] = tf_fcu.getOrigin().x() + x_gap;
      t[1] = tf_fcu.getOrigin().y() + y_gap;
      t[2] = tf_fcu.getOrigin().z() + z_gap;

      q_fcu[0] = tf_fcu.getRotation().x();
      q_fcu[1] = tf_fcu.getRotation().y();
      q_fcu[2] = tf_fcu.getRotation().z();
      q_fcu[3] = tf_fcu.getRotation().w();
  }
  tf::Vector3 getRPY(tf::Quaternion q){
      tf::Vector3 rpy;

      double sinr_cosp = 2*(q[3]*q[0]+q[1]*q[2]);
      double cosr_cosp = 1- 2*(q[0]*q[0] + q[1]*q[1]);
      rpy[0] = atan2(sinr_cosp, cosr_cosp);

      double sinp = 2*(q[3]*q[1]-q[2]*q[0]);
      if(std::abs(sinp)>=1) rpy[1] = std::copysign(M_PI/2,sinp);
      else rpy[1] = asin(sinp);

      double siny_cosp = 2*(q[3]*q[2]+q[0]*q[1]);
      double cosy_cosp = 1- 2*(q[1]*q[1]+q[2]*q[2]);
      rpy[2] = atan2(siny_cosp, cosy_cosp);

      return rpy;
  }
  void Quaternion2RotMat(){

      q_cam.setRPY(0,60*d2r,0); //RPY

      tf::Quaternion q;
      q = q_fcu*q_cam; //q.normalize();
      tf::Vector3 yaw_r = getRPY(q); //relate yaw
      q.setRPY(0, 60*d2r, yaw_r[2]);

      tf_cam.setOrigin(t);
      tf_cam.setRotation(q);

      //homogeneous
      R = (cv::Mat_<double>(3,3)
           <<pow(q[3],2)+pow(q[0],2)-pow(q[1],2)-pow(q[2],2), 2*(q[0]*q[1]-q[3]*q[2]), 2*(q[3]*q[1]+q[0]*q[2]),
             2*(q[3]*q[2]+q[0]*q[1]), pow(q[3],2)-pow(q[0],2)+pow(q[1],2)-pow(q[2],2), 2*(q[1]*q[2]-q[3]*q[0]),
             2*(q[0]*q[2]-q[3]*q[1]), 2*(q[3]*q[0]+q[1]*q[2]), pow(q[3],2)-pow(q[0],2)-pow(q[1],2)+pow(q[2],2));
      T = (cv::Mat_<double>(3,1) << t[0], t[1], t[2]);

      br.sendTransform(tf::StampedTransform(tf_cam, ros::Time::now(), "map", "camera"));
  }
  cv::Mat Pixel2Normal(geometry_msgs::Point pixel){//Normal
      geometry_msgs::Point normal;
      cv::Mat _normal;

      normal.x = (pixel.x-cx)/fx;
      normal.y = (pixel.y-cy)/fy;
      normal.z = 1;     //camera criteria
      //camera.z = 0;   //Pixel2Camera

      //normal coordinate to ned
      _normal = (cv::Mat_<double>(3,1) << normal.z, -normal.x, -normal.y);
      return _normal;
  }
  geometry_msgs::Point Dimension_transform(cv::Mat _normal){
      cv::Mat _ground;
      cv::Mat camera_cam = (cv::Mat_<double>(3, 1) << 0, 0, 0); //Cc
      geometry_msgs::Point ground;

      getTFvalue();
      Quaternion2RotMat();

      cv::Mat world_point = R*_normal+T;                        //Pw
      cv::Mat world_cam = T;                                    //Cw

      double z_wp = world_point.at<double>(2,0);
      double z_wc = world_cam.at<double>(2,0);
      double k = (-z_wc) / (z_wp-z_wc);

      _ground = world_cam + k*(world_point-world_cam);

      ground.x = _ground.at<double>(0,0);
      ground.y = _ground.at<double>(1,0);
      ground.z = _ground.at<double>(2,0);
      return ground;
  }
  geometry_msgs::Point getTransformed(geometry_msgs::Point pixel){
      cv::Mat _normal = Pixel2Normal(pixel);
      geometry_msgs::Point ground = Dimension_transform(_normal);

      //ground_point_pub.publish(ground);
      return ground;
  }

  // Object Data Pre-processing
  void core(){
      darknet_ros_msgs::BoundingBox obj_box;

      for(int i=0; i<obj_count; i++){ //obj_boxes.bounding_boxes.size()
          obj_box = obj_boxes.bounding_boxes[i];
          if(obj_box.probability>0.5){

          }
      }
/*
      if(_yolo_object_count>0){
        for(int i=0; i<(int)_yolo_bboxes.bounding_boxes.size(); i++){
          darknet_ros_msgs::BoundingBox _yolo_bbox = _yolo_bboxes.bounding_boxes[i];
          if(_yolo_bbox.Class=="person"||_yolo_bbox.Class=="bird"){ //_yolo_bbox.probability
            geometry_msgs::Point person_pnt;
            person_pnt.x = (_yolo_bbox.xmax+_yolo_bbox.xmin)/2;
            person_pnt.y = (_yolo_bbox.ymax+_yolo_bbox.ymin)/2;
            double yolo_d = sqrt(pow(pixel_point.x-person_pnt.x,2)+pow(pixel_point.y-person_pnt.y,2));
            if(yolo_d<gap_from_object) {
              gap_from_object = yolo_d;
              _person_box = _yolo_bbox;
      } } } }

      if(_person_box.probability>0.1){
        pixel_point.x= (_person_box.xmax+_person_box.xmin)/2;
        pixel_point.y= (_person_box.ymax+_person_box.ymin)/2;
        pointPub(); //pixel publish for openCV
        detection_state = OPENCV;
        ROS_INFO("[Detection] OPENCV!"); ROS_WARN("Detected!");
      }
*/
      obj_ground = getTransformed(obj_pixel);
      objMarker(obj_ground);

      msgPub(obj_ground);
  }

  void msgPub(geometry_msgs::Point human_ground){
      geometry_msgs::PoseStamped human_pose;
      human_pose.pose.position= human_ground;
      human_pose.pose.orientation.x=0;
      human_pose.pose.orientation.y=0;
      human_pose.pose.orientation.z=0;
      human_pose.pose.orientation.w=1;

      map_obj_pose_pub.publish(human_pose);
  }

  double getDistance(geometry_msgs::Point prev, geometry_msgs::Point cur){
    double d = sqrt(pow(cur.x-prev.x,2)+pow(cur.y-prev.y,2));
    return d;
  }

  void main(){
    ros::Rate rate(10.0);

    while(ros::ok()){

        try{
            ls.lookupTransform("map","vehicle",ros::Time(0),tf_fcu);
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("[Transform] %s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO_ONCE("[Transform] tf received");

        ros::spinOnce();
        rate.sleep();
    }
  }

  void objMarker(geometry_msgs::Point ground){
      visualization_msgs::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position = ground;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;

      marker.color.a = 1.0; //visible
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;

      marker.lifetime = ros::Duration();

      obj_marker_pub.publish(marker);
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_transform");
    DObjectMapping dom;
    dom.main();
    return 0;
}
