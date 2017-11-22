#include "corner_event_detector/fixed_distinct_queue.h"

namespace corner_event_detector
{

FixedDistinctQueue::FixedDistinctQueue(int window, int queue) :
  first_(-1), last_(-1), queue_max_(queue)
{
  window_ = Eigen::MatrixXi::Constant(window, window, -1);
  queue_.reserve(queue_max_);
}

bool FixedDistinctQueue::isFull() const
{
  return (queue_.size() >= queue_max_);
}

void FixedDistinctQueue::addNew(int x, int y)
{
  // queue full?
  if (queue_.size() < queue_max_)
  {
    if (window_(x, y) < 0)
    {
      // first element?
      if (queue_.empty())
      {
        first_ = 0;
        last_ = 0;

        QueueEvent qe;
        qe.prev = -1;
        qe.next = -1;
        qe.x = x;
        qe.y = y;
        queue_.push_back(qe);

        window_(x, y) = 0;
      }
      else
      {
        // add new element
        QueueEvent qe;
        qe.prev = -1;
        qe.next = first_;
        qe.x = x;
        qe.y = y;
        queue_.push_back(qe);

        const int place = queue_.size() - 1;
        queue_[first_].prev = place;
        first_ = place;

        window_(x, y) = place;
      }
    }
    else
    {
      // link neighbors of old event in queue
      const int place = window_(x, y);

      if (queue_[place].next >= 0 && queue_[place].prev >= 0)
      {
        queue_[queue_[place].prev].next = queue_[place].next;
        queue_[queue_[place].next].prev = queue_[place].prev;
      }

      // relink first and last
      if (place == last_)
      {
        if (queue_[place].prev >= 0)
        {
          last_ = queue_[place].prev;
          queue_[queue_[place].prev].next = -1;
        }
      }
      queue_[first_].prev = place;

      queue_[place].prev = -1;
      if (first_ != place)
      {
        queue_[place].next = first_;
      }

      first_ = place;
    }
  }
  else
  {
    // is window empty at location
    if (window_(x, y) < 0)
    {
      // update window
      window_(queue_[last_].x, queue_[last_].y) = -1;
      window_(x, y) = last_;

      // update queue
      queue_[queue_[last_].prev].next = -1;
      queue_[last_].x = x;
      queue_[last_].y = y;
      queue_[last_].next = first_;
      const int second_last = queue_[last_].prev;
      queue_[last_].prev = -1;
      queue_[first_].prev = last_;
      first_ = last_;
      last_ = second_last;
    }
    else
    {
      const int place = window_(x, y);
      if (place != first_)
      {
        // update window
        window_(x, y) = place;

        // update queue
        if (queue_[place].prev != -1)
        {
          queue_[queue_[place].prev].next = queue_[place].next;
        }
        if (queue_[place].next != -1)
        {
          queue_[queue_[place].next].prev = queue_[place].prev;
        }

        if (place == last_)
        {
          last_ = queue_[last_].prev;
        }

        queue_[place].prev = -1;
        queue_[place].next = first_;
        queue_[first_].prev = place;

        first_ = place;
      }
    }
  }
}

Eigen::MatrixXi FixedDistinctQueue::getWindow() const
{
  Eigen::MatrixXi patch = window_;
  for (int x = 0; x<window_.rows(); x++)
  {
    for (int y = 0; y<window_.cols(); y++)
    {
      patch(x, y) = (window_(x, y) < 0) ? 0 : 1;
    }
  }
  return patch;
}

} // namespace
