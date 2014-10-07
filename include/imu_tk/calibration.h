#pragma once

#include <vector>
#include <iostream>
#include <fstream>

#include "imu_tk/base.h"

namespace imu_tk
{
/*
 * Misalignment matrix:
 * 
 * general case:
 * 
 *     [    1     -mis_yz   mis_zy  ]
 * T = [  mis_xz     1     -mis_zx  ]
 *     [ -mis_xy   mis_yx     1     ]
 * 
 * "body" frame spacial case:
 * 
 *     [  1     -mis_yz   mis_zy  ]
 * T = [  0        1     -mis_zx  ]
 *     [  0        0        1     ]
 * 
 * Scale matrix:
 * 
 *     [  s_x      0        0  ]
 * K = [   0      s_y       0  ]
 *     [   0       0       s_z ]
 * 
 * Bias vector:
 * 
 *     [ b_x ]
 * B = [ b_y ]
 *     [ b_z ]
 * 
 * Given a raw sensor reading X (e.g., the acceleration ), the calibrated "unbiased" reading X' is obtained
 * 
 * X' = T*K*(X - B)
 * 
 * with B the bias (variable) + offset (constant, possbibly 0), or, equivalently:
 * 
 * X' = T*K*X - B'
 * 
 * with B' = T*K*B
 * 
 * Without knowing the value of the bias (and with offset == 0), the calibrated reading X'' is simply:
 * 
 * X'' = T*K*X
 */
template < typename _T > class CalibratedTriad_
{
public:
  CalibratedTriad_( const _T &mis_yz = _T(0), const _T &mis_zy = _T(0), const _T &mis_zx = _T(0), 
                   const _T &mis_xz = _T(0), const _T &mis_xy = _T(0), const _T &mis_yx = _T(0), 
                   const _T &s_x = _T(1),    const _T &s_y = _T(1),    const _T &s_z = _T(1), 
                   const _T &b_x = _T(0),    const _T &b_y = _T(0),    const _T &b_z  = _T(0) );
 
  ~CalibratedTriad_(){};
               
  inline _T misYZ() const { return -mis_mat_(0,1); };
  inline _T misZY() const { return mis_mat_(0,2); };
  inline _T misZX() const { return -mis_mat_(1,2); };
  inline _T misXZ() const { return mis_mat_(1,0); };
  inline _T misXY() const { return -mis_mat_(2,0); };
  inline _T misYX() const { return mis_mat_(2,1); };

  inline _T scaleX() const { return scale_mat_(0,0); };
  inline _T scaleY() const { return scale_mat_(1,1); };
  inline _T scaleZ() const { return scale_mat_(2,2); };
      
  inline _T biasX() const { return bias_vec_(0); };
  inline _T biasY() const { return bias_vec_(1); };
  inline _T biasZ() const { return bias_vec_(2); };
  
  inline const Eigen::Matrix< _T, 3 , 3>& getMisalignmentMatrix() const { return mis_mat_; };
  inline const Eigen::Matrix< _T, 3 , 3>& getScaleMatrix() const { return scale_mat_; };
  inline const Eigen::Matrix< _T, 3 , 1>& getBiasVector() const { return bias_vec_; };
    
  inline void setScale( const Eigen::Matrix< _T, 3 , 1> &s_vec ) 
  { 
    scale_mat_(0,0) = s_vec(0); scale_mat_(1,1) = s_vec(1);  scale_mat_(2,2) = s_vec(2); 
    update();
  };
  
  inline void setBias( const Eigen::Matrix< _T, 3 , 1> &b_vec ) 
  { 
    bias_vec_ = b_vec;
    update();
  };
  
  bool load( std::string filename );
  bool save( std::string filename ) const;

  inline Eigen::Matrix< _T, 3 , 1> normalize( const Eigen::Matrix< _T, 3 , 1> &raw_data ) const
  {
    return ms_mat_*raw_data;
  };
  
  inline TriadData_<_T> normalize( const TriadData_<_T> &raw_data ) const
  {
    return TriadData_<_T>( raw_data.timestamp(), normalize( raw_data.data()) );
  };
  
  inline Eigen::Matrix< _T, 3 , 1> unbiasNormalize( const Eigen::Matrix< _T, 3 , 1> &raw_data ) const
  {
    return ms_mat_*(raw_data - bias_vec_); 
  };
  
  inline TriadData_<_T> unbiasNormalize( const TriadData_<_T> &raw_data ) const
  {
    return TriadData_<_T>( raw_data.timestamp(), unbiasNormalize( raw_data.data()) );
  };
  
  inline Eigen::Matrix< _T, 3 , 1> unbias( const Eigen::Matrix< _T, 3 , 1> &raw_data ) const
  {
    return raw_data - bias_vec_; 
  };
  
  inline TriadData_<_T> unbias( const TriadData_<_T> &raw_data ) const
  {
    return TriadData_<_T>( raw_data.timestamp(), unbias( raw_data.data()) );
  };
  
private:

  void update();
  /** @brief Misalignment matrix */
  Eigen::Matrix< _T, 3 , 3> mis_mat_;
  /** @brief Scale matrix */
  Eigen::Matrix< _T, 3 , 3> scale_mat_;
  /** @brief Bias vector */
  Eigen::Matrix< _T, 3 , 1> bias_vec_;
  /** @brief Misalignment * scale matrix */
  Eigen::Matrix< _T, 3 , 3> ms_mat_;
};

typedef CalibratedTriad_<double> CalibratedTriad;

template <typename _T> std::ostream& operator<<(std::ostream& os, 
                                                const imu_tk::CalibratedTriad_<_T>& calib_triad);

template <typename _T> class MultiPosCalibration_
{
public:
  
  MultiPosCalibration_();
  ~MultiPosCalibration_(){};
  
  _T gravityMagnitede() const { return g_mag_; };
  int numInitSamples() const { return n_init_samples_; };
  int intarvalsNumSamples() const { return interval_n_samples_; };
  const CalibratedTriad_<_T>& initAccCalibration(){ return init_acc_calib_; };
  const CalibratedTriad_<_T>& initGyroCalibration(){ return init_gyro_calib_; };
  bool accUseMeans() const { return acc_use_means_; };
  _T gyroDataPeriod() const{ return gyro_dt_; };
  bool optimizeGyroBias() const { return optimize_gyro_bias_; };
  bool verboseOutput() const { return verbose_output_; };
  
  void setGravityMagnitude( _T g ){ g_mag_ = g; };
  void setNumInitSamples( int num ) { n_init_samples_ = num; };
  int setIntarvalsNumSamples( int num ) { interval_n_samples_ = num; };
  void setInitAccCalibration( CalibratedTriad_<_T> &init_calib ){ init_acc_calib_ = init_calib; };
  void setInitGyroCalibration( CalibratedTriad_<_T> &init_calib ){ init_gyro_calib_ = init_calib; };
  void enableAccUseMeans ( bool enabled ){ acc_use_means_ = enabled; };
  void setGyroDataPeriod( _T dt ){ gyro_dt_ = dt; };
  bool enableGyroBiasOptimization( bool enabled  ) { optimize_gyro_bias_ = enabled; };
  void enableVerboseOutput( bool enabled ){ verbose_output_ = enabled; };
  
  bool calibrateAcc( const std::vector< TriadData_<_T> > &acc_samples );
  bool calibrateAccGyro( const std::vector< TriadData_<_T> > &acc_samples, 
                         const std::vector< TriadData_<_T> > &gyro_samples );

  const CalibratedTriad_<_T>& getAccCalib() const  { return acc_calib_; };
  const CalibratedTriad_<_T>& getGyroCalib() const  { return gyro_calib_; };
  const std::vector< TriadData_<_T> >& getCalibAccSamples() const { return calib_acc_samples_; };
  const std::vector< TriadData_<_T> >& getCalibGyroSamples() const { return calib_gyro_samples_; };
  
private:
  
  _T g_mag_;
  const int min_num_intervals_;
  int n_init_samples_;
  int interval_n_samples_;
  bool acc_use_means_;
  _T gyro_dt_;
  bool optimize_gyro_bias_;
  std::vector< DataInterval_<_T> > min_cost_static_intervals_;
  CalibratedTriad_<_T> init_acc_calib_, init_gyro_calib_;
  CalibratedTriad_<_T> acc_calib_, gyro_calib_;
  std::vector< TriadData_<_T> > calib_acc_samples_, calib_gyro_samples_;
  
  bool verbose_output_;
};

typedef MultiPosCalibration_<double> MultiPosCalibration;

}

/* Implementations */

template <typename _T> 
  imu_tk::CalibratedTriad_<_T>::CalibratedTriad_( const _T &mis_yz, const _T &mis_zy, const _T &mis_zx, 
                                                const _T &mis_xz, const _T &mis_xy, const _T &mis_yx, 
                                                const _T &s_x, const _T &s_y, const _T &s_z, 
                                                const _T &b_x, const _T &b_y, const _T &b_z )
{
  mis_mat_ <<  _T(1)   , -mis_yz  ,  mis_zy  ,
                mis_xz ,  _T(1)   , -mis_zx  ,  
               -mis_xy ,  mis_yx  ,  _T(1)   ;
              
  scale_mat_ <<   s_x  ,   _T(0)  ,  _T(0) ,
                 _T(0) ,    s_y   ,  _T(0) ,  
                 _T(0) ,   _T(0)  ,   s_z  ;
                    
  bias_vec_ <<  b_x , b_y , b_z ; 
  
  update();
}

template <typename _T> 
  bool imu_tk::CalibratedTriad_<_T>::load( std::string filename )
{
  std::ifstream file( filename.data() );
  if (file.is_open())
  {
    _T mat[9] = {0};
    
    for( int i=0; i<9; i++)
      file >> mat[i];

    mis_mat_ = Eigen::Map< const Eigen::Matrix< _T, 3, 3, Eigen::RowMajor> >(mat);
      
    for( int i=0; i<9; i++)
      file >> mat[i];
    
    scale_mat_ = Eigen::Map< const Eigen::Matrix< _T, 3, 3, Eigen::RowMajor> >(mat);
        
    for( int i=0; i<3; i++)
      file >> mat[i];
    
    bias_vec_ = Eigen::Map< const Eigen::Matrix< _T, 3, 1> >(mat);    
    
    update();
    
    return true;
  }
  return false;  
}

template <typename _T> 
  bool imu_tk::CalibratedTriad_<_T>::save( std::string filename ) const
{
  std::ofstream file( filename.data() );
  if (file.is_open())
  {
    file<<mis_mat_<<std::endl<<std::endl
        <<scale_mat_<<std::endl<<std::endl
        <<bias_vec_<<std::endl<<std::endl;
    
    return true;
  }
  return false;  
}

template <typename _T> void imu_tk::CalibratedTriad_<_T>::update()
{
  ms_mat_ = mis_mat_*scale_mat_;
}

template <typename _T> std::ostream& imu_tk::operator<<(std::ostream& os, 
                                                        const imu_tk::CalibratedTriad_<_T>& calib_triad)
{
  os<<"Misalignment Matrix"<<std::endl;
  os<<calib_triad.getMisalignmentMatrix()<<std::endl;
  os<<"Scale Matrix"<<std::endl;
  os<<calib_triad.getScaleMatrix()<<std::endl;
  os<<"Bias Vector"<<std::endl;
  os<<calib_triad.getBiasVector()<<std::endl;
  return os;
}