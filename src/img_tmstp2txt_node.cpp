#include <ros/ros.h>

#include <bag_extractor/ImgTmstp2Txt.hpp>

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "img_tmstp2txt");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::ImgTmstp2Txt imgTmstp2Txt(nodeHandle);
  
  // Spin
  ros::spin();
}
