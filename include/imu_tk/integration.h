#pragma once

#include <Eigen/Core>
#include <ceres/rotation.h>

#include "imu_tk/base.h"

#include <iostream>

namespace imu_tk
{
 
template <typename _T> inline void normalizeQuaternion( Eigen::Matrix< _T, 4, 1> &quat );
template <typename _T> inline void normalizeQuaternion( _T quat[4] );

template <typename _T> inline void quatIntegrationStepRK4( const Eigen::Matrix< _T, 4, 1> &quat, 
                                                           const Eigen::Matrix< _T, 3, 1> &omega0, 
                                                           const Eigen::Matrix< _T, 3, 1> &omega1, 
                                                           const _T &dt, Eigen::Matrix< _T, 4, 1> &quat_res );

template <typename _T> inline void quatIntegrationStepRK4( const _T quat[4], 
                                                           const _T omega0[3], 
                                                           const _T omega1[3], 
                                                           const _T &dt, _T quat_res[4] );

template <typename _T> void integrateGyroInterval( const std::vector< TriadData_<_T> > &gyro_samples, 
                                                   Eigen::Matrix< _T, 4, 1> &quat_res, _T data_dt = _T(-1),
                                                   const DataInterval_<_T> &interval = DataInterval_<_T> (-1, -1) );

template <typename _T> void integrateGyroInterval( const std::vector< TriadData_<_T> > &gyro_samples, 
                                                   Eigen::Matrix< _T, 3, 3> &rot_res, _T data_dt = _T(-1),
                                                   const DataInterval_<_T> &interval = DataInterval_<_T> (-1, -1) );

}

/* Implementation */

template <typename _T> inline void imu_tk::normalizeQuaternion ( Eigen::Matrix< _T, 4 , 1  >& quat )
{
  _T quat_norm = quat.squaredNorm();
  quat /= quat_norm;
}

template <typename _T> inline void imu_tk::normalizeQuaternion ( _T quat[4] )
{
  Eigen::Matrix< _T, 4 , 1  > tmp_q = Eigen::Map< Eigen::Matrix< _T, 4 , 1  > >(quat);
  imu_tk::normalizeQuaternion ( tmp_q );
}

template <typename _T> 
  static inline void computeOmegaSkew( const Eigen::Matrix< _T, 3, 1> &omega, 
                                       Eigen::Matrix< _T, 4, 4> &skew )
{
  skew <<   _T(0),     -omega(0),  -omega(1),  -omega(2),
            omega(0),   _T(0),      omega(2),  -omega(1),
            omega(1),  -omega(2),   _T(0),      omega(0),
            omega(2),   omega(1),  -omega(0),   _T(0);
}

template <typename _T> 
  inline void imu_tk::quatIntegrationStepRK4( const Eigen::Matrix< _T, 4, 1> &quat, 
                                              const Eigen::Matrix< _T, 3, 1> &omega0, 
                                              const Eigen::Matrix< _T, 3, 1> &omega1, 
                                              const _T &dt, Eigen::Matrix< _T, 4, 1> &quat_res )
{
  Eigen::Matrix< _T, 3, 1> omega01 = _T(0.5)*( omega0 + omega1 );
  Eigen::Matrix< _T, 4, 1> k1, k2, k3, k4, tmp_q;
  Eigen::Matrix< _T, 4, 4> omega_skew;
  
  // First Runge-Kutta coefficient
  computeOmegaSkew( omega0, omega_skew );
  k1 = _T(0.5)*omega_skew*quat;
  // Second Runge-Kutta coefficient
  tmp_q = quat + _T(0.5)*dt*k1;
  computeOmegaSkew( omega01, omega_skew );
  k2 = _T(0.5)*omega_skew*tmp_q;
  // Third Runge-Kutta coefficient (same omega skew as second coeff.)
  tmp_q = quat + _T(0.5)*dt*k2;
  k3 = _T(0.5)*omega_skew*tmp_q;
  // Forth Runge-Kutta coefficient
  tmp_q = quat + dt*k3;
  computeOmegaSkew( omega1, omega_skew );
  k4 = _T(0.5)*omega_skew*tmp_q;
  _T mult1 = _T(1.0)/_T(6.0), mult2 = _T(1.0)/_T(3.0);
  quat_res = quat + dt*(mult1*k1 + mult2*k2 + mult2*k3 + mult1*k4);
  quat_res /= quat_res.norm();
}

template <typename _T> 
  inline void imu_tk::quatIntegrationStepRK4( const _T quat[4], const _T omega0[3], const _T omega1[3], 
                                              const _T &dt, _T quat_res[4] )
{
  const Eigen::Matrix< _T, 4, 1> m_quat = Eigen::Map< const Eigen::Matrix< _T, 4, 1> >(quat);
  const Eigen::Matrix< _T, 3, 1> m_omega0 = Eigen::Map< const Eigen::Matrix< _T, 3, 1> >(omega0),
                                 m_omega1 = Eigen::Map< const Eigen::Matrix< _T, 3, 1> >(omega1); 
  Eigen::Matrix< _T, 4, 1> m_quat_res;
  
  quatIntegrationStepRK4( m_quat, m_omega0, m_omega1, dt, m_quat_res );
  
  quat_res[0] = m_quat_res(0);
  quat_res[1] = m_quat_res(1);
  quat_res[2] = m_quat_res(2);
  quat_res[3] = m_quat_res(3);
}

template <typename _T> void imu_tk::integrateGyroInterval( const std::vector< TriadData_<_T> > &gyro_samples, 
                                                           Eigen::Matrix< _T, 4, 1> &quat_res,
                                                           _T data_dt, const DataInterval_<_T> &interval )
{
  DataInterval_<_T> rev_interval =  checkInterval( gyro_samples, interval );

  quat_res = Eigen::Matrix< _T, 4, 1>(_T(1.0), _T(0), _T(0), _T(0)); // Identity quaternion
  
  for( int i = rev_interval.start_idx; i < rev_interval.end_idx; i++)
  {
    _T dt = ( data_dt > _T(0))?data_dt:gyro_samples[i+1].timestamp() - gyro_samples[i].timestamp();
    
    quatIntegrationStepRK4( quat_res,
                            gyro_samples[i].data(), 
                            gyro_samples[i + 1].data(), 
                            dt,
                            quat_res );
  }
}

template <typename _T> void imu_tk::integrateGyroInterval( const std::vector< TriadData_<_T> >& gyro_samples, 
                                                           Eigen::Matrix< _T, 3 , 3  >& rot_res, 
                                                           _T data_dt, const DataInterval_<_T>& interval )
{
  Eigen::Matrix< _T, 4, 1> quat_res;
  integrateGyroInterval( gyro_samples, quat_res, data_dt, interval );
  ceres::MatrixAdapter<_T, 1, 3> rot_mat = ceres::ColumnMajorAdapter3x3(rot_res.data());
  ceres::QuaternionToRotation( quat_res.data(), rot_mat );
}