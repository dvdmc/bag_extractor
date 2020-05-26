#pragma once

#include "bag_extractor/Utils.hpp"

// ROS
#include <ros/ros.h>

// Quaternion conversion
#include <tf/tf.h>

// Bag reading and processing
#include <rosbag/bag.h>
#include <rosbag/view.h>

// IMU specific includes
#include <sensor_msgs/CompressedImage.h>

// OpenCV

#include <opencv2/imgcodecs.hpp>

// CV bridge
#include <cv_bridge/cv_bridge.h>

// Image transport
#include <image_transport/image_transport.h>

//Boost
//#include <boost/lexical_cast.hpp>
// Can be used as an alternative to std::to_str but it seems that is slower for "float"

// STD
#include <string>

namespace bag_extractor
{

/*!
*  Main class for the image bag extractor.
*/
class IMG2JPG
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    IMG2JPG(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    virtual ~IMG2JPG();

    /*!
     * Extract all data.
     * TODO: It takes the start and end point params if specified.
     */
    void extract();

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if succesful.
     */
    bool readParameters_();

    /*!
     * Set the time queries if the time limits have been set.
     * @param start the starting ros time for the query.
     * @param end the end ros time for the query.
     */
    void setTimeFilters_(ros::Time &start, ros::Time &end);

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void imgMsgProcess_(const sensor_msgs::CompressedImageConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! Counter for the number of files
    int counter_;

    //! Folder name for the saved point clouds
    std::string folder_;

    //! File to save the Imu data
    std::ofstream out_;

    //! Velodyne topic name
    std::string topic_;

    //! Bag name
    std::string bagName_;

    //! Rosbag
    rosbag::Bag bag_;

    double start_time_filter_;
    double end_time_filter_;
    
    //! Encoding of the compressed images
    // std::string encoding_ = "bgr8";

    //! Extension of the file to save the data.
    const std::string EXTENSION_ = ".jpg";
};

} // namespace bag_extractor