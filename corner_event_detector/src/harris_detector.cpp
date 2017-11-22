#include "corner_event_detector/harris_detector.h"

namespace corner_event_detector
{

HarrisDetector::HarrisDetector(bool connect)
: Detector(connect)
{
  detector_name_ = "Harris";

  // parameters
  queue_size_ = 25;
  window_size_ = 4;
  kernel_size_ = 5;
  harris_threshold_ = 8.0;

  queues_ = new DistinctQueue(window_size_, queue_size_, true);

  Eigen::VectorXd Dx = Eigen::VectorXd(kernel_size_);
  Eigen::VectorXd Sx = Eigen::VectorXd(kernel_size_);
  for (int i=0; i<kernel_size_; i++)
  {
    Sx[i] = factorial(kernel_size_ - 1)/
            (factorial(kernel_size_ - 1 - i) * factorial(i));
    Dx[i] = pasc(i, kernel_size_-2) - pasc(i-1, kernel_size_-2);
  }
  Gx_ = Sx * Dx.transpose();
  Gx_ = Gx_ / Gx_.maxCoeff();

  const double sigma = 1.;
  const double A = 1./(2.*M_PI*sigma*sigma);
  const int l2 = (2*window_size_+2-kernel_size_)/2;
  h_ = Eigen::MatrixXd(2*l2+1, 2*l2+1);
  for (int x=-l2; x<=l2; x++)
  {
    for (int y=-l2; y<=l2; y++)
    {
      const double h_xy = A * exp(-(x*x+y*y)/(2*sigma*sigma));
      h_(l2+x, l2+y) = h_xy;
    }
  }
  h_ /= h_.sum();
}

HarrisDetector::~HarrisDetector()
{
}

bool HarrisDetector::isFeature(const dvs_msgs::Event &e)
{
  // update queues
  queues_->newEvent(e.x, e.y, e.polarity);

  // check if queue is full
  double score = harris_threshold_ - 10.;
  if (queues_->isFull(e.x, e.y, e.polarity))
  {
    // check if current event is a feature
    score = getHarrisScore(e.x, e.y, e.polarity);

    last_score_ = score;
  }

  return (score > harris_threshold_);
}

double HarrisDetector::getHarrisScore(int img_x, int img_y, bool polarity)
{
  // do not consider border
  if (img_x<window_size_ or img_x>sensor_width_-window_size_ or
      img_y<window_size_ or img_y>sensor_height_-window_size_)
  {
    // something below the threshold
    return harris_threshold_ - 10.;
  }

  const Eigen::MatrixXi local_frame = queues_->getPatch(img_x, img_y, polarity);

  const int l = 2*window_size_+2-kernel_size_;
  Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(l, l);
  Eigen::MatrixXd dy = Eigen::MatrixXd::Zero(l, l);
//  Eigen::MatrixXd dxy = Eigen::MatrixXd::Zero(l, l);
  for (int x=0; x<l; x++)
  {
    for (int y=0; y<l; y++)
    {
      for (int kx=0; kx<kernel_size_; kx++)
      {
        for (int ky=0; ky<kernel_size_; ky++)
        {
          dx(x, y) += local_frame(x+kx, y+ky)*Gx_(kx, ky);
          dy(x, y) += local_frame(x+kx, y+ky)*Gx_(ky, kx);
        }
      }
    }
  }

  double a=0., b=0., d=0.;
  for (int x=0; x<l; x++)
  {
    for (int y=0; y<l; y++)
    {
      a += h_(x, y) * dx(x, y) * dx(x, y);
      b += h_(x, y) * dx(x, y) * dy(x, y);
      d += h_(x, y) * dy(x, y) * dy(x, y);
    }
  }

  const double score = a*d-b*b - 0.04*(a+d)*(a+d);

  return score;
}


int HarrisDetector::factorial(int n) const
{
  if (n > 1)
  {
    return n * factorial(n - 1);
  }
  else
  {
    return 1;
  }
}

int HarrisDetector::pasc(int k, int n) const
{
  if (k>=0 && k<=n)
  {
    return factorial(n)/(factorial(n-k)*factorial(k));
  }
  else
  {
    return 0;
  }
}

} // namespace
