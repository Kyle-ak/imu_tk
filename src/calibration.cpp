#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

#include <limits>
#include <iostream>
#include "ceres/ceres.h"

using namespace imu_tk;
using namespace Eigen;
using namespace std;

template <typename _T1> struct MultiPosAccResidual
{
  MultiPosAccResidual( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample ) :
  g_mag_(g_mag),
  sample_(sample){}
  
  template <typename _T2>
    bool operator() ( const _T2* const params, _T2* residuals ) const
  {
    Eigen::Matrix< _T2, 3 , 1> raw_samp( _T2(sample_(0)), _T2(sample_(1)), _T2(sample_(2)) );
    /* Assume body frame same as accelerometer frame, 
     * so bottom left params in the misalignment matris are set to zero */
    CalibratedTriad<_T2> calib_triad( params[0], params[1], params[2], 
                                     _T2(0), _T2(0), _T2(0),
                                     params[3], params[4], params[5], 
                                     params[6], params[7], params[8] );
    
    Eigen::Matrix< _T2, 3 , 1> calib_samp = calib_triad.normalizeUnbias( raw_samp );
    residuals[0] = _T2 ( g_mag_ ) - calib_samp.norm();
    return true;
  }
  
  static ceres::CostFunction* Create ( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample )
  {
    return ( new ceres::AutoDiffCostFunction< MultiPosAccResidual, 1, 9 > (
               new MultiPosAccResidual<_T1>( g_mag, sample ) ) );
  }
  
  const _T1 g_mag_;
  const Eigen::Matrix< _T1, 3 , 1> sample_;
};

// struct MultiPosGyroResidual
// {
//   MultiPosGyroResidual( const Eigen::Vector3d &g_versor_pos0, const Eigen::Vector3d &g_versor_pos1,
//                         const std::vector< TriadData > &gyro_samples, 
//                         const DataInterval &gyro_interval_pos01, double dt ) :
// 
//   g_versor_pos0_(g_versor_pos0), 
//   g_versor_pos1_(g_versor_pos1),
//   gyro_samples_(gyro_samples),
//   interval_pos01_(gyro_interval_pos01),
//   dt_(dt)
//   {
//     calib_gyro_samples_.resize( interval_pos01_.end_idx - interval_pos01_.start_idx + 1 );
//   }
//   
//   template <typename _T>
//     bool operator() ( const _T* const params, _T* residuals ) const
//   {
//     // Bias previously removed 
//     CalibratedTriad<_T> calib_triad( params[0], params[1], params[2], 
//                                      params[3], params[4], params[5], 
//                                      params[6], params[7], params[8],
//                                      _T(0), _T(0), _T(0) );
//    
//     for( int i = 0, j = interval_pos01_.start_idx; 
//          j <= interval_pos01_.end_idx; i++, j++ )
//       calib_gyro_samples_[i] = calib_triad.normalize( gyro_samples_[j] );
//     
//     Eigen::Matrix< _T, 3 , 3> rot_mat;
//     integrateGyroInterval( calib_gyro_samples_, rot_mat, _T(dt_) );
//     
//     Eigen::Matrix< _T, 3 , 1> diff = rot_mat.inverse()*g_versor_pos0_ - g_versor_pos1_;
//     
//     residuals[0] = diff(0);
//     residuals[1] = diff(1);
//     residuals[2] = diff(2);
//     
//     return true;
//   }
//   
//   static ceres::CostFunction* Create ( const Eigen::Vector3d &g_versor_pos0, const Eigen::Vector3d &g_versor_pos1,
//                                        const std::vector< TriadData > &gyro_samples, 
//                                        const DataInterval &gyro_interval_pos01, double dt )
//   {
//     return ( new ceres::AutoDiffCostFunction< MultiPosGyroResidual, 3, 9 > (
//                new MultiPosGyroResidual( g_versor_pos0, g_versor_pos1, gyro_samples, 
//                                          gyro_interval_pos01, dt ) ) );
//   }
//   
//   const Eigen::Vector3d g_versor_pos0_, g_versor_pos1_;
//   const std::vector< TriadData > gyro_samples_;
//   const DataInterval interval_pos01_;
//   const double dt_;
//   std::vector< TriadData > calib_gyro_samples_;
// };

template <typename _T>
  MultiPosCalibration<_T>::MultiPosCalibration() :
  g_mag_(9.81),
  min_num_intervals_(12),
  acc_init_bias_(0, 0, 0),
  n_init_samples_(3000),
  interval_n_samples_(100),
  acc_use_means_(false),
  gyro_dt_(-1.0),
  verbose_output_(false){}

template <typename _T>
  bool MultiPosCalibration<_T>::calibrateAcc ( const std::vector< TriadData<_T> >& acc_samples )
{
  min_cost_static_intervals_.clear();
  calib_acc_samples_.clear();
  calib_gyro_samples_.clear();
  
  int n_samps = acc_samples.size();
  
  Eigen::Matrix<_T, 3, 1> acc_variance = dataVariance( acc_samples, DataInterval( 0, n_init_samples_) );
  _T norm_th = acc_variance.norm();

  _T min_cost = std::numeric_limits< _T >::max();
  int min_cost_th = -1;
  std::vector< double > min_cost_calib_params;
  
  for (int th_mult = 2; th_mult <= 10; th_mult++)
  {
    std::vector< imu_tk::DataInterval > static_intervals;
    std::vector< imu_tk::TriadData<_T> > static_samples;
    std::vector< double > acc_calib_params(9, 0.0 );
    acc_calib_params[3] = acc_calib_params[4] = acc_calib_params[5] = 1.0;
    acc_calib_params[6] = acc_init_bias_(0);
    acc_calib_params[7] = acc_init_bias_(1);
    acc_calib_params[8] = acc_init_bias_(2);
    
    
    staticIntervalsDetector ( acc_samples, th_mult*norm_th, static_intervals );
    int extracted_intervals = extractIntervalsSamples ( acc_samples, static_intervals, 
                                                        static_samples, interval_n_samples_, 
                                                        acc_use_means_ );
    
    if(verbose_output_)
      cout<<"Accelerometers calibration: extracted "<<extracted_intervals
          <<" intervals using threshold multiplier "<<th_mult<<" -> ";
      
    if( extracted_intervals < min_num_intervals_)
    {
      if( verbose_output_) cout<<"Not enough intervals, calibration is not possible"<<endl;
      continue;
    }
    
    if( verbose_output_) cout<<"Trying calibrate... "<<endl;

//     PlotPtr plot = createPlot();
//     plotIntervals( plot, acc_samples, static_intervals );
//     plotSamples( plot, static_samples );
//     waitForKey();
    
    ceres::Problem problem;
    for( int i = 0; i < static_samples.size(); i++)
    {
      ceres::CostFunction* cost_function =
        MultiPosAccResidual<_T>::Create ( g_mag_, static_samples[i].data() );

      problem.AddResidualBlock ( cost_function, NULL /* squared loss */, acc_calib_params.data() ); 
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = verbose_output_;

    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );
    if( summary.final_cost < min_cost)
    {
      min_cost = summary.final_cost;
      min_cost_th = th_mult;
      min_cost_static_intervals_ = static_intervals;
      min_cost_calib_params = acc_calib_params;
    }
    if( verbose_output_) cout<<"residual "<<summary.final_cost<<endl;
  }
  
  if( min_cost_th < 0 )
  {
    if(verbose_output_) 
      cout<<"Can't obtain any calibratin with the current dataset"<<endl;
    return false;
  }

  acc_calib_ = CalibratedTriad<_T>(min_cost_calib_params[0],
                                   min_cost_calib_params[1],
                                   min_cost_calib_params[2],
                                   0,0,0,
                                   min_cost_calib_params[3],
                                   min_cost_calib_params[4],
                                   min_cost_calib_params[5],
                                   min_cost_calib_params[6],
                                   min_cost_calib_params[7],
                                   min_cost_calib_params[8]);
  
  calib_acc_samples_.reserve(n_samps);
  
  // Calibrate the input accelerometer data with the obtained calibration
  for( int i = 0; i < n_samps; i++)
    calib_acc_samples_.push_back( acc_calib_.normalizeUnbias( acc_samples[i]) );
  
  if(verbose_output_) 
  {
    cout<<"Better calibration obtained using threshold multiplier "<<min_cost_th
    <<" with residual "<<min_cost<<endl;
    
    cout<<acc_calib_<<endl;
  }
  
  return true;
    
}

template <typename _T> 
  bool MultiPosCalibration<_T>::calibrateAccGyro ( const vector< TriadData<_T> >& acc_samples, 
                                                   const vector< TriadData<_T> >& gyro_samples )
{
  if( !calibrateAcc( acc_samples ) )
    return false;
  
  std::vector< TriadData<_T> > static_acc_means;
  
  extractIntervalsSamples ( calib_acc_samples_, min_cost_static_intervals_, 
                            static_acc_means, interval_n_samples_, true );
  
  int n_static_pos = static_acc_means.size(), n_samps = gyro_samples.size();
  
  cout<<"static_acc_means.size() : "<<n_static_pos<<endl;
  
  for( int i = 0; i < n_static_pos; i++ )
    cout<<static_acc_means[i]<<endl;
  
  // Compute the gyroscope biases in the (static) initialization interval
  Eigen::Matrix<_T, 3, 1> gyro_bias = dataMean( gyro_samples, DataInterval( 0, n_init_samples_) );
  
  gyro_calib_ = CalibratedTriad<_T>(0, 0, 0, 0, 0, 0, 
                                    1.0, 1.0, 1.0, 
                                    gyro_bias(0), gyro_bias(1), gyro_bias(2) );
  

  // calib_gyro_samples_ alreadt cleared in calibrateAcc()
  calib_gyro_samples_.reserve(n_samps);
  // Remove the bias
  for( int i = 0; i < n_samps; i++ )
    calib_gyro_samples_.push_back(gyro_calib_.unbias(gyro_samples[i]));
  
  
  // TODO
  
  return true;
}

template class MultiPosCalibration<double>;
template class MultiPosCalibration<float>;