#include <ros/ros.h>

#include <bag_extractor/PCTmstp2Txt.hpp>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_tmstp");
  ros::NodeHandle nodeHandle("~");

  bag_extractor::PCTmstp2Txt pCTmstp2Txt(nodeHandle);

  // Spin
  ros::spin ();

}
