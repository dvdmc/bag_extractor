#pragma once

#include <bag_extractor/Utils.hpp>

// ROS
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/CompressedImage.h>

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

// STD
#include <fstream>
#include <string>

// Library to manage bags
//#include <rosbag/bag.h>

namespace bag_extractor
{

/*!
*  Main class for the node to handle the ROS interfacing.
*/
class ImgTmstp2Txt
{

public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    ImgTmstp2Txt(ros::NodeHandle &nodeHandle);

    /*!
     * Destructor;
     */
    ~ImgTmstp2Txt();

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
    void writeTimestampCallback_(const sensor_msgs::CompressedImageConstPtr &msg);

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriberImage_;

    //! Folder name for the saved timestamps
    std::string folder_;

    //! Image topic name
    std::string subscriberTopic_;

    //! Output file holder
    std::ofstream outFile_;

    //! ROS subscriber queue size.
    int queueSize_;
};

} // namespace bag_extractor