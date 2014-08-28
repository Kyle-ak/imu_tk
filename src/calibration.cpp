#include "imu_tk/calibration.h"

using namespace Eigen;

void imu_tk::staticIntervalsDetector ( const std::vector< imu_tk::TriadData > &samples, double threshold,
                                       std::vector< imu_tk::DataInterval > &intervals, int win_size )
{
  if ( win_size < 11 ) win_size = 3;
  if( !(win_size % 2) ) win_size++;
  
  int h = win_size / 2;
  
  if( win_size >=  samples.size() )
    return;
 
  intervals.clear();
  
  bool look_for_start = true;
  DataInterval current_interval;
  
  for( int i = h; i < samples.size() - h; i++ )
  {
    Array3d variance = dataVariance( samples, DataInterval( i - h, i + h) );
    double magnitude = (variance*variance).sum();
    
    if( look_for_start )
    {
      if (magnitude < threshold)
      {
        current_interval.start_idx = i;
        look_for_start = false;
      }
    }
    else
    {
      if (magnitude >= threshold)
      {
        current_interval.end_idx = i;
        look_for_start = true;
        intervals.push_back(current_interval);
      }
    }
  }
  
  // If the last interval has not been included in the intervals vector
  if( !look_for_start )
  {
    current_interval.end_idx = samples.size() - h - 1;
    intervals.push_back(current_interval);
  }
}