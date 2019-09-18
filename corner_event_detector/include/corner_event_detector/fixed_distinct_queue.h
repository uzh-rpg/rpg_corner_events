#pragma once

#include <deque>
#include <vector>
#include <Eigen/Dense>

namespace corner_event_detector
{

class FixedDistinctQueue
{
public:
  FixedDistinctQueue(int window, int queue);

  bool isFull() const;

  void addNew(int x, int y);
  Eigen::MatrixXi getWindow() const;

private:
  // contains index of queue element if occupied, negative value otherwise
  Eigen::MatrixXi window_;
  // contains one event
  struct QueueEvent
  {
    int prev, next;
    int x, y;
  };
  std::vector<QueueEvent> queue_;
  int first_, last_;
  int queue_max_;
};


} // namespace
