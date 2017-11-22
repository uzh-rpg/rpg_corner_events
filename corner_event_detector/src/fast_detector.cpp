#include "corner_event_detector/fast_detector.h"

namespace corner_event_detector
{

FastDetector::FastDetector(bool connect)
: Detector(connect),
  circle3_ {{0, 3}, {1, 3}, {2, 2}, {3, 1},
            {3, 0}, {3, -1}, {2, -2}, {1, -3},
            {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
            {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
  circle4_ {{0, 4}, {1, 4}, {2, 3}, {3, 2},
            {4, 1}, {4, 0}, {4, -1}, {3, -2},
            {2, -3}, {1, -4}, {0, -4}, {-1, -4},
            {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
            {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}
{
  detector_name_ = "FAST";

  // allocate SAE matrices
  sae_[0] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
  sae_[1] = Eigen::MatrixXd::Zero(sensor_height_, sensor_width_);
}

FastDetector::~FastDetector()
{
}

bool FastDetector::isFeature(const dvs_msgs::Event &e)
{
  // update SAE
  const int pol = e.polarity ? 1 : 0;
  sae_[pol](e.x, e.y) = e.ts.toSec();

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*4;
  if (e.x < cs || e.x >= sensor_width_-cs ||
      e.y < cs || e.y >= sensor_height_-cs)
  {
    return false;
  }

  bool found_streak = false;

  for (int i=0; i<16; i++)
  {
    for (int streak_size = 3; streak_size<=6; streak_size++)
    {
      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]) <  sae_[pol](e.x+circle3_[(i-1+16)%16][0], e.y+circle3_[(i-1+16)%16][1]))
        continue;

      // check that streak event is larger than neighbor
      if (sae_[pol](e.x+circle3_[(i+streak_size-1)%16][0], e.y+circle3_[(i+streak_size-1)%16][1]) <          sae_[pol](e.x+circle3_[(i+streak_size)%16][0], e.y+circle3_[(i+streak_size)%16][1]))
        continue;

      double min_t = sae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]);
      for (int j=1; j<streak_size; j++)
      {
        const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);
        if (tj < min_t)
          min_t = tj;
      }

      bool did_break = false;
      for (int j=streak_size; j<16; j++)
      {
        const double tj = sae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);

        if (tj >= min_t)
        {
          did_break = true;
          break;
        }
      }

      if (!did_break)
      {
        found_streak = true;
        break;
      }

    }
    if (found_streak)
    {
      break;
    }
  }

  if (found_streak)
  {
    found_streak = false;
    for (int i=0; i<20; i++)
    {
      for (int streak_size = 4; streak_size<=8; streak_size++)
      {
        // check that first event is larger than neighbor
        if (sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]) <  sae_[pol](e.x+circle4_[(i-1+20)%20][0], e.y+circle4_[(i-1+20)%20][1]))
          continue;

        // check that streak event is larger than neighbor
        if (sae_[pol](e.x+circle4_[(i+streak_size-1)%20][0], e.y+circle4_[(i+streak_size-1)%20][1]) <          sae_[pol](e.x+circle4_[(i+streak_size)%20][0], e.y+circle4_[(i+streak_size)%20][1]))
          continue;

        double min_t = sae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]);
        for (int j=1; j<streak_size; j++)
        {
          const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<20; j++)
        {
          const double tj = sae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj >= min_t)
          {
            did_break = true;
            break;
          }
        }

        if (!did_break)
        {
          found_streak = true;
          break;
        }
      }
      if (found_streak)
      {
        break;
      }
    }
  }

  return found_streak;
}

} // namespace
