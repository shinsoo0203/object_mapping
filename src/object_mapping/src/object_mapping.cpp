/**
 * @file object_mapping.cpp
 * @author IDPLab sooyeon Shin
 */

#include <ros/ros.h>
#include <iostream>

class DObjectMapping{

private:
  ros::NodeHandle nh;

public:
  DObjectMapping(){
  }
  ~DObjectMapping(){}

  void main(){

  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_mapping");
    DObjectMapping dom;
    dom.main();
    return 0;
}
