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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "visualization/marker.cpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>

using namespace grid_map;

class GridMapping{

private:
  ros::NodeHandle nh;

  ros::Publisher grid_mapper;
  ros::Publisher obj_local_pub;
  ros::Publisher obj_marker_pub;
  ros::Subscriber obj_3Dbboxes_sub;

  tf::TransformListener ls;
  tf::StampedTransform tf_zed; //transform zed2_left_camera_frame from map
  gb_visual_detection_3d_msgs::BoundingBoxes3d bboxes_3d;

  tf::Quaternion q;
  tf::Vector3 t;
  cv::Mat R, T;

  Marker marker;
  int mark_num = 0;

public:
  GridMapping(){
    grid_mapper = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    obj_3Dbboxes_sub = nh.subscribe<gb_visual_detection_3d_msgs::BoundingBoxes3d>\
        ("/darknet_ros_3d/bounding_boxes", 10, &GridMapping::Object3DBBoxesCb, this);

    obj_local_pub = nh.advertise<geometry_msgs::Pose>("/obj_local", 10);
    obj_marker_pub = nh.advertise<visualization_msgs::Marker>("/obj_marker", 10);

    ls.waitForTransform("zed2_left_camera_frame","map",ros::Time::now(),ros::Duration(3.0));
    ROS_INFO("[Grid Mapping]: started.");
  }
  ~GridMapping(){}

  void Object3DBBoxesCb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr& msg){
    bboxes_3d = *msg;
  }

  void getTransform(){

    ls.lookupTransform("zed2_left_camera_frame","map",ros::Time(0),tf_zed);
    geometry_msgs::Transform transform; //translation, rotation

    t[0] = tf_zed.getOrigin().x();
    t[1] = tf_zed.getOrigin().y();
    t[2] = 0; //tf_zed.getOrigin().z();

    q[0] = tf_zed.getRotation().x();
    q[1] = tf_zed.getRotation().y();
    q[2] = tf_zed.getRotation().z();
    q[3] = tf_zed.getRotation().w();

    R = (cv::Mat_<double>(3,3)
         <<pow(q[3],2)+pow(q[0],2)-pow(q[1],2)-pow(q[2],2), 2*(q[0]*q[1]-q[3]*q[2]), 2*(q[3]*q[1]+q[0]*q[2]),
           2*(q[3]*q[2]+q[0]*q[1]), pow(q[3],2)-pow(q[0],2)+pow(q[1],2)-pow(q[2],2), 2*(q[1]*q[2]-q[3]*q[0]),
           2*(q[0]*q[2]-q[3]*q[1]), 2*(q[3]*q[0]+q[1]*q[2]), pow(q[3],2)-pow(q[0],2)-pow(q[1],2)+pow(q[2],2));
    T = (cv::Mat_<double>(3,1) << t[0], t[1], t[2]);
  }

  void objFiltering(){

    for(int i=0; i<bboxes_3d.bounding_boxes.size(); i++){
      gb_visual_detection_3d_msgs::BoundingBox3d bbox = bboxes_3d.bounding_boxes[i];

      if(bbox.xmax!=INFINITY && bbox.ymax!=INFINITY && bbox.probability>=0.95){

        getTransform();
        cv::Mat obj_zed2_M = (cv::Mat_<double>(3, 1) << (bbox.xmin+bbox.xmax)/2, bbox.ymin, 0);
        cv::Mat obj_map_M = (R*obj_zed2_M)+T;

        geometry_msgs::Pose obj_map;
        obj_map.position.x = obj_map_M.at<double>(0,0);
        obj_map.position.y = obj_map_M.at<double>(1,0);
        obj_map.position.z = obj_map_M.at<double>(2,0);

        std::cout<<obj_map<<std::endl;

        obj_local_pub.publish(obj_map);

        visualization_msgs::Marker obj_mark = marker.pose_marker(obj_map, mark_num, "red");
        obj_marker_pub.publish(obj_mark);
        mark_num ++;
      }
    }
  }

  void main()
  {
    // Create grid map
    GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
    map.setFrameId("map");
    map.setGeometry(Length(500, 500), 4.0, Position(0.0, 0.0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
      map.getLength().x(), map.getLength().y(),
      map.getSize()(0), map.getSize()(1),
      map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

    // main loop
    ros::Rate rate(30.0);
    while(nh.ok()){
      ros::Time time = ros::Time::now();

      objFiltering();
      //objMapping();

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

      ros::spinOnce();
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
