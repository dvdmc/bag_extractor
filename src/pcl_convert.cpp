#include <ros/ros.h>

//Libraries to manage files
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//Library to manage bags
#include <rosbag/bag.h>

//Function to convert from int to string using sstream
template< typename Type> std::string to_str (const Type & t){
  std::ostringstream os;
  os << t;
  return os.str();
}

//Global counter for the number of files
int counter;
std::string bagname;
//Function to create a file storing all the point cloud as a 4xn matrix in a binary file
//with the format: xyzintensity

void
velodyne2bin(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //Get the number of the file and creates it according to the specifications
  std::string number = to_str(counter);
  std::string filename = ".bin";
  filename = number + filename;

  std::ostringstream os;

  os << std::setw(14) << std::setfill('0') << filename;
  filename = os.str();

  filename = "./" + bagname +  "velodyne_points/" + filename;

  ROS_INFO("Saving PointCloud into: %s",filename.c_str());

  std::ofstream out;
  out.open(filename.c_str(), std::ios::binary);

  // Create iterator for the data
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x");it != it.end(); ++it){
    out.write(reinterpret_cast<const char*>( &it[0] ), sizeof(float)*4);
  }
  out.close();
  counter++;
}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "pcl_convert");
  ros::NodeHandle nh("~");

  std::string velodyne_topic;

  nh.getParam("bagname",bagname);
  nh.getParam("lidar_topic",velodyne_topic);
  ROS_INFO("Subscribed to %s",velodyne_topic.c_str());
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (velodyne_topic.c_str(), 1, velodyne2bin);

  while(ros::ok()){

    // Spin
    ros::spin ();
  }
}
