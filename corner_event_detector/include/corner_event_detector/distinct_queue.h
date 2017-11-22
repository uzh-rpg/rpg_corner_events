#pragma once

#include <deque>
#include <Eigen/Dense>

#include "corner_event_detector/local_event_queues.h"
#include "corner_event_detector/fixed_distinct_queue.h"

namespace corner_event_detector
{

class DistinctQueue : public LocalEventQueues
{
public:
  DistinctQueue(int window_size, int queue_size, bool use_polarity);
  virtual ~DistinctQueue();

  void newEvent(int x, int y, bool pol=false);
  bool isFull(int x, int y, bool pol=false) const;
  Eigen::MatrixXi getPatch(int x, int y, bool pol=false);

private:
  // data structure
  std::vector<FixedDistinctQueue> queues_;

  // helper function
  int getIndex(int x, int y, bool polarity) const;

  // constants
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};

} // namespace
