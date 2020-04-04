#include "bag_extractor/PC2BIN.hpp"

namespace bag_extractor
{

PC2BIN::PC2BIN(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters_())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    counter_ = 0;

    subscriberPC_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                          &PC2BIN::pcCallback_, this);

    ROS_INFO("Successfully launched node.");
}

PC2BIN::~PC2BIN()
{
}

void PC2BIN::pcCallback_(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::string filename = utils::get_pc_name(counter_,folder_);

    ROS_INFO("Saving PointCloud into: %s", filename.c_str());

    std::ofstream out;

    out.open(filename.c_str(), std::ios::binary);

    // Create iterator for the data
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)
    {
        out.write(reinterpret_cast<const char *>(&it[0]), sizeof(float) * 4);
    }
    out.close();

    counter_++;
}

bool PC2BIN::readParameters_()
{
    if (!nodeHandle_.getParam("folder", folder_))
        return false;
    if (!nodeHandle_.getParam("topic", subscriberTopic_))
        return false;
    if (!nodeHandle_.getParam("queue_size", queueSize_))
        return false;
    return true;
}

} // namespace bag_extractor