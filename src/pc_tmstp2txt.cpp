#include <ros/ros.h>

//Libraries to manage files
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <time.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
//Library to manage bags
#include <rosbag/bag.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
//Function to convert from int to string using sstream
template< typename Type> std::string to_str (const Type & t){
  std::ostringstream os;
  os << t;
  return os.str();
}

//Global counter for the number of files
int counter;
std::string folder;
std::string topic;
std::ofstream out;
//Function to extract the timestamp from the recieved msg

void getTimestamp(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std_msgs::Header h = msg->header;
  auto rawtime = h.stamp.toBoost();
  std::stringstream ss;
  auto facet = new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << rawtime << std::endl;
  ROS_INFO("Saved time: %s", ss.str().c_str());
  out.write(&ss.str().c_str()[0], ss.str().size());
}

int
main (int argc, char** argv)
{
  ROS_INFO("Point cloud timestamp extractor started");
  // Initialize ROS
  ros::init (argc, argv, "pc_tmstp");
  ros::NodeHandle nh("~");

  nh.getParam("topic",topic);
  nh.getParam("folder",folder);

  ROS_INFO("Subscribed to %s",topic.c_str());
  // Create a ROS subscriber
  std::string filename = "timestamps.txt";
  filename = "./" + folder + filename;
  out.open(filename.c_str(), std::ios::out);
  ROS_INFO("Opened file: %s",filename.c_str());
  ros::Subscriber sub = nh.subscribe (topic.c_str(), 1, getTimestamp);

  while(ros::ok()){

    // Spin
    ros::spin ();
  }

  out.close();
}
