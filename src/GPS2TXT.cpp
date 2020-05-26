#include "bag_extractor/GPS2TXT.hpp"

namespace bag_extractor
{

    GPS2TXT::GPS2TXT(ros::NodeHandle &nodeHandle)
        : nodeHandle_(nodeHandle)
    {

        if (!readParameters_())
        {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }

        ROS_INFO("Successfully launched node.");
    }

    GPS2TXT::~GPS2TXT()
    {
    }

    void GPS2TXT::extract()
    {
        // Create the file in the specified path.
        std::string filename = "./" + folder_ + "data.txt";

        // Open the file to store the data.
        out_.open(filename.c_str(), std::ios::out);

        // Check if the file was opened correctly.
        if (!out_)
        {
            ROS_ERROR("Problem opening the file: %s!!!", filename.c_str());
            ros::requestShutdown();
        }

        // Init a counter (Not used right now)
        //counter_ = 0;

        ROS_INFO("Saving GPS data into: %s", filename.c_str());

        // Open the bag using the bagname path.
        bag_.open(bagName_);

        // Declare the default time query to use if the time was not specified.
        /* TODO A work around to check if the time is valid for the 
        specified bag would be to create a previous ros::View without 
        the time param to extract the max and min times of the bag 
        and check that the selected ones are in between. */

        ros::Time ros_start_filter = ros::TIME_MIN;
        ros::Time ros_end_filter = ros::TIME_MAX;

        // If the time was specified, modify the time query.
        setTimeFilters_(ros_start_filter, ros_end_filter);

        // This case covers several possible topics.
        // Therefore a coma separated list is created from them.
        std::vector<std::string> topics = utils::split_strings(topic_, ',');

        ROS_INFO("Topics: ");
        for(std::string const topic : topics)
        {
            ROS_INFO("%s",topic.c_str());
        }

        // Create the topic queryto extract information about.
        rosbag::View view(bag_, rosbag::TopicQuery(topics), ros_start_filter, ros_end_filter);

        // Initialize time data for progress indicator.
        uint64_t begin_time = view.getBeginTime().toNSec();
        uint64_t end_time = view.getEndTime().toNSec();
        uint64_t duration = end_time - begin_time;

        for (rosbag::MessageInstance const m : view)
        {
            // Try to get a GPGGA message instance from the iterator.
            novatel_gps_msgs::GpggaConstPtr gpggaMsg = m.instantiate<novatel_gps_msgs::Gpgga>();

            if (gpggaMsg != NULL)
            {
                // Compute and show the progress.
                uint64_t m_time = m.getTime().toNSec();
                float progress = (float)(m_time - begin_time) / (float)duration * 100;
                ROS_INFO("Processing GPGGA message ( %.2f%% )", progress);

                // Process the gpgga msg and write to file.
                gpggaMsgProcess_(gpggaMsg);
            }

            // Try to get a GPRMC message instance from the iterator.
            novatel_gps_msgs::GprmcConstPtr gprmcMsg = m.instantiate<novatel_gps_msgs::Gprmc>();

            if (gprmcMsg != NULL)
            {
                // Compute and show the progress.
                uint64_t m_time = m.getTime().toNSec();
                float progress = (float)(m_time - begin_time) / (float)duration * 100;
                ROS_INFO("Processing GPRMC message ( %.2f%% )", progress);

                // Process the gprmc msg and write to file.
                gprmcMsgProcess_(gprmcMsg);
            }

            // Break the loop in case of shutdown.
            if (!ros::ok())
            {
                ROS_INFO("ROS shutdown!");
                bag_.close();
                return;
            }
        }

        //bag_.close();
        ROS_INFO("Finished!");
    }

    void GPS2TXT::setTimeFilters_(ros::Time &start, ros::Time &end)
    {

        // Set the time filters if they were set.
        if (start_time_filter_ >= 0)
        {
            start = ros::Time(start_time_filter_);
        }

        if (end_time_filter_ >= 0)
        {
            end = ros::Time(end_time_filter_);
        }

        ROS_INFO("Start filtering at: %f", start.toSec());

        ROS_INFO("End filtering at: %f", end.toSec());
    }

    void GPS2TXT::gpggaMsgProcess_(const novatel_gps_msgs::GpggaConstPtr &msg)
    {
        std::string r = "";

        r += std::to_string(msg->header.stamp.toSec()) + " $GPGGA,";

        r += std::to_string(msg->utc_seconds) + ",";
        r += std::to_string(msg->lat) + ",";
        r += msg->lat_dir + ",";
        r += std::to_string(msg->lon) + ",";
        r += msg->lon_dir + ",";
        r += std::to_string(msg->gps_qual) + ",";
        r += std::to_string(msg->num_sats) + ",";
        r += std::to_string(msg->hdop) + ",";
        r += std::to_string(msg->alt) + ",";
        r += msg->altitude_units + ",";
        r += std::to_string(msg->undulation) + ",";
        r += msg->undulation_units + ",";
        r += std::to_string(msg->diff_age) + ",";
        r += msg->station_id + "\n";

        ROS_INFO("Printing GPGGA sentence: \n%s",r.c_str());
        out_.write(r.c_str(), r.length());

        //counter_++;
    }

    void GPS2TXT::gprmcMsgProcess_(const novatel_gps_msgs::GprmcConstPtr &msg)
    {
        std::string r = "";

        r += std::to_string(msg->header.stamp.toSec()) + " $GPRMC,";

        r += std::to_string(msg->utc_seconds) + ",";
        r += msg->position_status + ",";
        r += std::to_string(msg->lat) + ",";
        r += msg->lat_dir + ",";
        r += std::to_string(msg->lon) + ",";
        r += msg->lon_dir + ",";
        r += std::to_string(msg->speed) + ",";
        r += std::to_string(msg->track) + ",";
        r += msg->date + ",";
        r += std::to_string(msg->mag_var) + ",";
        r += msg->mag_var_direction + ",";
        r += msg->mode_indicator + "\n";

        ROS_INFO("Printing GPRMC sentence: \n%s",r.c_str());
        out_.write(r.c_str(), r.length());

        //counter_++;
    }

    bool GPS2TXT::readParameters_()
    {
        if (!nodeHandle_.getParam("folder", folder_))
            return false;
        if (!nodeHandle_.getParam("topic", topic_))
            return false;
        if (!nodeHandle_.getParam("bag", bagName_))
            return false;

        // Init the time filters if specified or -1 otherwise.
        if (!nodeHandle_.getParam("start_time", start_time_filter_))
            start_time_filter_ = -1;
        if (!nodeHandle_.getParam("end_time", end_time_filter_))
            end_time_filter_ = -1;
        return true;
    }

} // namespace bag_extractor