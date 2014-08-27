#pragma once

#include <stdint.h>
#include <Eigen/Core>

namespace imu_tk
{
 class TriadData
 {
 public:
   TriadData(){};
   TriadData( uint64_t timestamp, double x, double y, double z ) :
    timestamp_(timestamp),
    data_(x, y, z) {};
   
   TriadData( uint64_t timestamp, const Eigen::Vector3d &data ) :
    timestamp_(timestamp),
    data_(data) {};
    
   TriadData( uint64_t timestamp, const double *data ) :
    timestamp_(timestamp),
    data_(data[0], data[1], data[2]) {};
   
   ~TriadData(){};
   
   inline const uint64_t& timestamp() const { return timestamp_; };
   inline const Eigen::Vector3d& data() const { return data_; };
   
   inline const double& operator()( int index ) const { return data_[index]; };
   inline const double& x() const { return data_[0]; };
   inline const double& y() const { return data_[1]; };
   inline const double& z() const { return data_[2]; };
   
 private:
   Eigen::Vector3d data_;
   uint64_t timestamp_;
 };
}