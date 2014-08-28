#pragma once
#include <vector>
#include <Eigen/Core>

namespace imu_tk
{
  
struct DataInterval
{
public:
  DataInterval(){};
  DataInterval( int idx0, int idx1 ) : start_idx(idx0), end_idx(idx1) {};
  int start_idx, end_idx;
};

class TriadData
{
public:
  TriadData() {};
  TriadData ( double timestamp, double x, double y, double z ) :
    timestamp_ ( timestamp ),
    data_ ( x, y, z ) {};

  TriadData ( double timestamp, const Eigen::Array3d &data ) :
    timestamp_ ( timestamp ),
    data_ ( data ) {};

  TriadData ( double timestamp, const double *data ) :
    timestamp_ ( timestamp ),
    data_ ( data[0], data[1], data[2] ) {};

  ~TriadData() {};

  inline const double& timestamp() const { return timestamp_; };
  inline const Eigen::Array3d& data() const { return data_; };
  inline const double& operator() ( int index ) const { return data_[index]; };
  inline const double& x() const { return data_[0]; };
  inline const double& y() const { return data_[1]; };
  inline const double& z() const { return data_[2]; };
  
private:
  Eigen::Array3d data_;
  double timestamp_;
};

Eigen::Array3d dataMean( const std::vector< TriadData > &samples, DataInterval interval = DataInterval( -1, -1 ) );
Eigen::Array3d dataVariance( const std::vector< TriadData > &samples, DataInterval interval = DataInterval( -1, -1 ) );
  
}
