#pragma once

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include "corner_event_detector/timer.h"

namespace corner_event_detector
{

class Detector
{
public:
  Detector(bool connect = true);
  virtual ~Detector();

  // check if event
  virtual bool isFeature(const dvs_msgs::Event &e) = 0;

protected:
  std::string detector_name_;

private:
  // interface
  ros::NodeHandle nh_;
  ros::Publisher feature_pub_;
  ros::Subscriber event_sub_;
  void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg);

  // statistics
  double total_time_;
  int total_events_, total_corners_;
};


} // namespace
