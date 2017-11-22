#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <Eigen/Dense>

#include "corner_event_detector/detector.h"

namespace corner_event_detector
{

class FastDetector : public Detector
{
public:
  FastDetector(bool connect = true);
  virtual ~FastDetector();

  bool isFeature(const dvs_msgs::Event &e);

private:
  // SAE
  Eigen::MatrixXd sae_[2];

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};


} // namespace
