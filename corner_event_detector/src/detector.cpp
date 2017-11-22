#include "corner_event_detector/detector.h"

namespace corner_event_detector
{

Detector::Detector(bool detect)
: total_time_(0.), total_events_(0), total_corners_(0)
{
  // interface
  if (detect)
  {
    feature_pub_ = nh_.advertise<dvs_msgs::EventArray>("/feature_events", 1);
    event_sub_ = nh_.subscribe("/dvs/events", 0, &Detector::eventCallback, this);
  }
}

Detector::~Detector()
{
  // print overall statistics
  std::cout << "Statistics for " <<  detector_name_  << std::endl
  << " Total time [ns]: " << total_time_ << std::endl
  << " Total number of events: " << total_events_ << std::endl
  << " Total number of corners: " << total_corners_ << std::endl
  << " Time/event [ns]: " << total_time_/(double) total_events_ << std::endl
  << " Events/s: " << total_events_/total_time_*1e9  << std::endl
  << " Reduction (%): " << (1.-total_corners_/(double)total_events_)*100
  << std::endl;
}

void Detector::eventCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
  dvs_msgs::EventArray feature_msg;
  feature_msg.header = msg->header;
  feature_msg.width = msg->width;
  feature_msg.height = msg->height;

  utils::time::Timer<std::chrono::nanoseconds> timer;
  for (const auto e : msg->events)
  {
    if (isFeature(e))
    {
      feature_msg.events.push_back(e);
    }
  }
  const auto elapsed_time_nsecs = timer.toc();

  // global stats
  total_time_ += elapsed_time_nsecs;
  total_events_ += msg->events.size();
  total_corners_ += feature_msg.events.size();

  // publish feature events
  feature_pub_.publish(feature_msg);

  // stats
  const int num_events = msg->events.size();
  if (num_events > 0)
  {
    const int num_features = feature_msg.events.size();
    const float reduction_rate = 100.*(1.-num_features/(float) num_events);
    const float reduction_factor = num_events/(float) num_features;
    const float events_per_second = float(num_events)/(elapsed_time_nsecs/1e9);
    const float ns_per_event = elapsed_time_nsecs/float(num_events);
    ROS_INFO("%s reduction rate: %.3f%% (%.0fx). Speed: %.0f e/s / %.0f ns/e.",
      detector_name_.c_str(), reduction_rate, reduction_factor,
      events_per_second, ns_per_event);
  }
  else
  {
    ROS_INFO("%s reduction rate: No events.", detector_name_.c_str());
  }
}

} // namespace
