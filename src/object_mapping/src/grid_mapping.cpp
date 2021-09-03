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

#include "object_mapping/ObjectInfo.h"
#include "object_mapping/ObjectArray.h"

using namespace grid_map;

class GridMapping{

private:
  ros::NodeHandle nh;

  ros::Publisher grid_mapper;
  ros::Publisher obj_local_pub;
  ros::Publisher obj_marker_pub;
  ros::Publisher obj_grid_pub;
  ros::Publisher obj_pub;
  ros::Subscriber obj_3Dbboxes_sub;

  tf::TransformListener ls;
  tf::StampedTransform tf_zed; //transform zed2_left_camera_frame from map
  gb_visual_detection_3d_msgs::BoundingBoxes3d bboxes_3d;
  object_mapping::ObjectArray objectArray_map;
  object_mapping::ObjectArray objectArray_grid;

  tf::Quaternion q;
  tf::Vector3 t;
  cv::Mat R, T;

  Marker marker;
  int mark_num = 0;

  double cell_size = 4.0;

public:
  GridMapping(){
    grid_mapper = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    obj_3Dbboxes_sub = nh.subscribe<gb_visual_detection_3d_msgs::BoundingBoxes3d>\
        ("/darknet_ros_3d/bounding_boxes", 10, &GridMapping::Object3DBBoxesCb, this);

    obj_local_pub = nh.advertise<geometry_msgs::Pose>("/local/obj", 10);
    obj_marker_pub = nh.advertise<visualization_msgs::Marker>("/marker/obj_map", 10);
    obj_grid_pub = nh.advertise<object_mapping::ObjectArray>("/grid/objArray", 10);
    obj_pub = nh.advertise<object_mapping::ObjectInfo>("/grid/obj",10);

    ls.waitForTransform("zed2_left_camera_frame","map",ros::Time::now(),ros::Duration(3.0));
    ROS_INFO("[Grid Mapping]: started.");
  }
  ~GridMapping(){}

  void Object3DBBoxesCb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr& msg){
    bboxes_3d = *msg;

    // Initialize bboxes msg
    objectArray_map.header = msg->header;
    objectArray_map.header.frame_id = "map";
    objectArray_map.object_info.clear();

    objectArray_grid.header = msg->header;
    objectArray_grid.header.frame_id = "map";
    objectArray_grid.object_info.clear();
  }

  void getTransform(){

    //ls.lookupTransform("zed2_left_camera_frame","map",ros::Time(0),tf_zed);
    ls.lookupTransform("map","zed2_left_camera_frame",ros::Time(0),tf_zed);
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
      object_mapping::ObjectInfo object_map;

      if(bbox.xmax!=INFINITY && bbox.ymax!=INFINITY && bbox.probability>=0.95){

        getTransform();
        cv::Mat obj_zed2_M = (cv::Mat_<double>(3, 1) << (bbox.xmin+bbox.xmax)/2, bbox.ymin, 0);
        cv::Mat obj_map_M = (R*obj_zed2_M)+T;

        geometry_msgs::Pose obj_map;
        obj_map.position.x = obj_map_M.at<double>(0,0);
        obj_map.position.y = obj_map_M.at<double>(1,0);
        obj_map.position.z = obj_map_M.at<double>(2,0);

        //std::cout<<obj_map<<std::endl;
        obj_local_pub.publish(obj_map);

        visualization_msgs::Marker obj_mark = marker.pose_marker(obj_map, mark_num, "red");
        obj_marker_pub.publish(obj_mark);
        mark_num ++;

        //obj_map
        object_map.Class = bbox.Class;
        object_map.probability = bbox.probability;
        object_map.position.x = int(obj_map.position.x);
        object_map.position.y = int(obj_map.position.y);
        objectArray_map.object_info.push_back(object_map);
      }
    }
  }

  void main()
  {
    // Create grid map
    GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
    map.setFrameId("map");
    map.setGeometry(Length(800, 800), cell_size, Position(0.0, 0.0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
      map.getLength().x(), map.getLength().y(),
      map.getSize()(0), map.getSize()(1),
      map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

    // main loop
    ros::Rate rate(1);

    ROS_INFO("[Grid Mapping] Loop Start.");
    while(nh.ok()){
      ros::Time time = ros::Time::now();

      objFiltering();

      // iterating through grid map and adding data
      for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
        Position position;
        map.getPosition(*it, position);
        map.at("elevation", *it) = 0;

        // object mapping
        for(int i=0; i<objectArray_map.object_info.size(); i++){
          object_mapping::ObjectInfo obj = objectArray_map.object_info[i];
          object_mapping::ObjectInfo obj_grid;

          geometry_msgs::Point idx;
          idx.x = position[0];
          idx.y = position[1];

          if(int(obj.position.x/cell_size)==int(idx.x/cell_size) && int(obj.position.y/cell_size)==int(idx.y/cell_size)){
            ROS_INFO("[Grid Mapping] Found %s", obj.Class.c_str());
            obj_grid.Class = obj.Class;
            obj_grid.probability = obj.probability;
            obj_grid.position.x = idx.x;
            obj_grid.position.y = idx.y;
            objectArray_grid.object_info.push_back(obj_grid);


            obj_pub.publish(obj_grid);
          }
        }

        Eigen::Vector3d normal(1, 1, 1);
        normal.normalize();
        map.at("normal_x", *it) = normal.x();
        map.at("normal_y", *it) = normal.y();
        map.at("normal_z", *it) = normal.z();
      }

      // Publish obj_grid info
      if(objectArray_grid.object_info.size()>0){ obj_grid_pub.publish(objectArray_grid); }

      // Publish grid map.
      map.setTimestamp(time.toNSec());
      grid_map_msgs::GridMap message;
      GridMapRosConverter::toMessage(map, message);
      grid_mapper.publish(message);
      //ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

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
