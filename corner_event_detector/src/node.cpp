#include <ros/ros.h>

#include "corner_event_detector/detector.h"
#include "corner_event_detector/harris_detector.h"
#include "corner_event_detector/fast_detector.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "corner_event_detector");

  // load parameter
  std::string feature_type;
  ros::param::param<std::string>("~feature_type", feature_type, "harris");

  // create feature detecotr
  corner_event_detector::Detector* detector;
  if (feature_type == "harris")
  {
    ROS_INFO("Using Harris detector.");
    detector = new corner_event_detector::HarrisDetector;
  }
  else if (feature_type == "fast")
  {
    ROS_INFO("Using fast detector.");
    detector = new corner_event_detector::FastDetector;
  }
  else
  {
    ROS_ERROR("Feature type '%s' is unknown.", feature_type.c_str());
    return 1;
  }

  // run
  ros::spin();

  delete detector;

  return 0;
}
