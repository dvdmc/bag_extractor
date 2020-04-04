#include "bag_extractor/ImgTmstp2Txt.hpp"

namespace bag_extractor
{

ImgTmstp2Txt::ImgTmstp2Txt(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters_())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    utils::openTimestampsFile_(folder_, outFile_);

    ROS_INFO("Opened timestamp.txt file in: %s", folder_.c_str());

    subscriberImage_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                          &ImgTmstp2Txt::writeTimestampCallback_, this);

    ROS_INFO("Image timestamp extractor started");
}

ImgTmstp2Txt::~ImgTmstp2Txt()
{
}

void ImgTmstp2Txt::writeTimestampCallback_(const sensor_msgs::CompressedImageConstPtr &msg)
{
    std_msgs::Header h = msg->header;
    boost::posix_time::ptime rawtime = h.stamp.toBoost();

    std::stringstream ss;

    boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y-%m-%d %H:%M:%s");

    ss.imbue(std::locale(std::locale::classic(), facet));
    ss << rawtime << std::endl;

    ROS_INFO("Saved img time: %s", ss.str().c_str());
    outFile_.write(&ss.str().c_str()[0], ss.str().size());
}

bool ImgTmstp2Txt::readParameters_()
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