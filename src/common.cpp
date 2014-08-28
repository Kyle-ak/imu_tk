#include "imu_tk/common.h"

using namespace Eigen;

static imu_tk::DataInterval checkInterval( const std::vector< imu_tk::TriadData > &samples, imu_tk::DataInterval interval )
{
  int start_idx = interval.start_idx, end_idx = interval.end_idx;
  if( start_idx < 0) start_idx = 0;
  if( end_idx < start_idx ) end_idx = samples.size() - 1;
  
  return imu_tk::DataInterval( start_idx, end_idx );
}

Array3d imu_tk::dataMean( const std::vector< TriadData > &samples, DataInterval interval )
{
  interval =  checkInterval( samples, interval );
  int n_samp = interval.end_idx - interval.start_idx + 1;
  Array3d mean(0, 0, 0);
  
  for( int i = interval.start_idx; i <= interval.end_idx; i++)
    mean += samples[i].data();
  
  mean /= double(n_samp);
  
  return mean;
}

Array3d imu_tk::dataVariance( const std::vector< TriadData > &samples, DataInterval interval )
{
  interval =  checkInterval( samples, interval );
  int n_samp = interval.end_idx -interval.start_idx + 1;
  Array3d mean = dataMean( samples, interval );
  
  Array3d variance(0, 0, 0);
  for( int i = interval.start_idx; i <= interval.end_idx; i++)
  {
    Array3d diff = samples[i].data() - mean ;
    variance += (diff * diff);
  }
  variance /= double(n_samp - 1);
  
  return variance;
}