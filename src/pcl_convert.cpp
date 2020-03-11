#include <ros/ros.h>

//Libraries to manage files
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//Library to manage bags
#include <rosbag/bag.h>

//Function to convert from t to string using sstream
template< typename Type> std::string to_str (const Type & t){
  std::ostringstream os;
  os << t;
  return os.str();
}

//Global counter for the number of files
int counter;

//Function to create a file storing all the point cloud as a 4xn matrix in a binary file
//with the format: xyzintensity

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //Get the number of the file and creates it according to the specifications
  std::string number = to_str(counter);
  std::string filename = ".txt";
  filename = number + filename;

  while(filename.length() <= 13){
  filename = '0' + filename;
  }

  filename="./velodyne_points/" + filename;

  ROS_INFO("saving into: %s",filename.c_str());
  std::ofstream out;
  out.open(filename.c_str(), std::ios::out);

  // Create iterator for the data
  for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x");it != it.end(); ++it){
    out << it[0] <<  it[1] <<  it[2] << it[3] << std::endl;
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

  std::string bag_name, velodyne_topic;

  nh.getParam("bagname",bag_name);
  nh.getParam("lidar_topic",velodyne_topic);

  ROS_INFO("Trying to open %s and extract from topic %s",bag_name.c_str(),velodyne_topic.c_str());

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe (velodyne_topic.c_str(), 1, cloud_cb);
  
  while(ros::ok()){

    // Spin
    ros::spin ();
  }
}
