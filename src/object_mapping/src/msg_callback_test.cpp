#include <ros/ros.h>
#include <iostream>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "visualization/marker.cpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>

class msg_callback_test{
  private:
    ros::NodeHandle nh;
    ros::Subscriber obj_3Dbboxes_sub;
  public:
    msg_callback_test(){
      obj_3Dbboxes_sub = nh.subscribe<gb_visual_detection_3d_msgs::BoundingBoxes3d>\
          ("/darknet_ros_3d/bounding_boxes", 10, &msg_callback_test::Object3DBBoxesCb, this);
    }
    ~msg_callback_test(){
    }
    void Object3DBBoxesCb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr& msg){
      gb_visual_detection_3d_msgs::BoundingBoxes3d bbox = *msg;//gb_visual_detection_3d_msgs::BoundingBoxes3d::iterator it = msg->bounding_boxes.begin();
      std::cout<<"Class:"<<bbox<<std::endl;
    }
    void spin(){
      ros::Rate r(5);
      while(ros::ok()){
        r.sleep();
        ros::spinOnce();
      }
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_mapping");
    msg_callback_test *_msg_callback_test = new msg_callback_test();
    _msg_callback_test->spin();
    return 0;
}
