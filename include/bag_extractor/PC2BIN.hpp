#pragma once

#include "bag_extractor/Utils.hpp"

// ROS
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Library to manage bags TODO: Change approach to bag API
//#include <rosbag/bag.h>

// STD
#include <string>

namespace bag_extractor
{

/*!
*  Main class for the node to handle the ROS interfacing.
*/
class PC2BIN
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    PC2BIN(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    virtual ~PC2BIN();

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if succesful.
     */
    bool readParameters_();

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void pcCallback_(const sensor_msgs::PointCloud2ConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriberPC_;

    //! Counter for the number of files
    int counter_;

    //! Folder name for the saved point clouds
    std::string folder_;

    //! Velodyne topic name
    std::string subscriberTopic_;

    //! ROS subscriber queue size.
    int queueSize_;
};

} // namespace bag_extractor