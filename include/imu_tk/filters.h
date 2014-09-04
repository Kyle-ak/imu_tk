#pragma once

#include <vector>
#include "imu_tk/base.h"

namespace imu_tk
{

/**
  * @brief Classify between static and motion intervals checking if for each sample 
  *        of the input signal \f$s\f$ (samples) the local variance magnitude 
  *        is lower or greater then a threshold.
  * 
  * @param samples Input 3D signal (e.g, the acceleremeter readings)
  * @param threshold Threshold used in the classification
  * @param intervals  Ouput detected static intervals
  * @param win_size Size of the sliding window used to compute the local variance magnitude
  * 
  * 
  * 
  * The variance magnitude is a scalar computed in a temporal sliding window of size 
  * \f$w_s\f$ (i.e., win_size) as:
  * \f[ \varsigma(t) = 
  *     \sqrt{[var_{w_s}(s^t_x)]^2 + [var_{w_s}(s^t_y)]^2 + [var_{w_s}(s^t_z)]^2} \f] 
  * 
  * Where \f$var_{w_s}(s^t)\f$ is an operator that compute the variance of
  * a general 1D signal in a interval of length \f$w_s\f$ samples
  * centered in \f$t\f$.
  */
template <typename _T> 
  void staticIntervalsDetector ( const std::vector< TriadData<_T> > &samples,
                                 _T threshold, std::vector< DataInterval<_T> > &intervals,
                                 int win_size = 101 );
}
