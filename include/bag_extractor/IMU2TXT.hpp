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
#include <sensor_msgs/Imu.h>

//Boost
//#include <boost/lexical_cast.hpp>
// Can be used as an alternative to std::to_str but it seems that is slower for "float"

// STD
#include <string>

namespace bag_extractor
{

/*!
*  Main class for the node to handle the ROS interfacing.
*/
class IMU2TXT
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    IMU2TXT(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    virtual ~IMU2TXT();

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
     * Method to process the message obtained from the message instance.
     * @param msg the message to process.
     */
    void imuMsgProcess_(const sensor_msgs::ImuConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! Counter for the number of files
    int counter_;

    //! Folder name for the saved point clouds
    std::string folder_;

    //! File to save the Imu data
    std::ofstream out_;

    //! Imu topic name
    std::string topic_;

    //! Bag name
    std::string bagName_;

    //! Rosbag
    rosbag::Bag bag_;

    double start_time_filter_;
    double end_time_filter_;

    //! Extension of the file to save the data.
    const std::string EXTENSION_ = ".txt";
};

} // namespace bag_extractor