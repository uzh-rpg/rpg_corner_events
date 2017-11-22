#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include "corner_event_detector/detector.h"

#include "corner_event_detector/local_event_queues.h"
#include "corner_event_detector/distinct_queue.h"

namespace corner_event_detector
{

class HarrisDetector : public Detector
{
public:
  HarrisDetector(bool connect = true);
  virtual ~HarrisDetector();

  bool isFeature(const dvs_msgs::Event &e);
  double getLastScore() const {
    return last_score_;
  }

private:
  // methods
  void updateQueue(const int x, const int y, const dvs_msgs::Event &e);
  double getHarrisScore(int x, int y, bool polarity);

  // queues
  LocalEventQueues* queues_;

  // parameters
  int queue_size_;
  int window_size_;
  int kernel_size_;
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
  double harris_threshold_;

  double last_score_;

  // kernels
  Eigen::MatrixXd Gx_, h_;
  int factorial(int n) const;
  int pasc(int k, int n) const;
};


} // namespace
