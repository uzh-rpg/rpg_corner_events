#pragma once

#include <Eigen/Dense>

namespace corner_event_detector
{

class LocalEventQueues
{
public:
  LocalEventQueues(int window_size, int queue_size)
  : window_size_(window_size), queue_size_(queue_size) {}

  virtual void newEvent(int x, int y, bool pol) = 0;
  virtual bool isFull(int x, int y, bool pol) const = 0;
  virtual Eigen::MatrixXi getPatch(int x, int y, bool pol) = 0;

protected:
  int window_size_;
  int queue_size_;
};

} // namespace
