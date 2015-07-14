#include <pcl/visualization/cloud_viewer.h>
#include <ceres/rotation.h>

#include "imu_tk/visualization.h"
#include <cstdio>
#include <sstream>

using namespace imu_tk;

class Plot::PlotImpl
{
public:
  
  PlotImpl()
  { 
    gnuplot_pipe_ = popen("gnuplot", "w");
    if( gnuplot_pipe_ == NULL )
      std::cerr<<"WARNING : missing gnuplot application!"<<std::endl;
  };
  
  ~PlotImpl()
  { 
    if( gnuplot_pipe_ != NULL )
      pclose( gnuplot_pipe_ );
  };
  
  bool ready() const { return gnuplot_pipe_ != NULL; };
  void write( const std::string &str )
  {
    if( gnuplot_pipe_ != NULL )
    {
      fprintf( gnuplot_pipe_, "%s\n", str.c_str() );
      fflush( gnuplot_pipe_ );
    }
  };  
  
private:
  
  FILE *gnuplot_pipe_;
};

Plot::Plot()
{
  plot_impl_ptr_ = boost::shared_ptr< PlotImpl > ( new PlotImpl() );
}

template <typename _T> 
  void Plot::plotSamples ( const std::vector< TriadData_<_T> >& samples, 
                                   DataInterval range )
{
  if( !plot_impl_ptr_->ready() )
    return;
  
  range = checkInterval( samples, range );
  
  std::stringstream strs;
  strs<<"plot '-' title 'x' with lines, "
      <<"'-' title 'y' with lines, "
      <<"'-' title 'z' with lines"<<std::endl;
   
  _T base_time = samples[0].timestamp();
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].x())<<std::endl;
  strs<<"EOF"<<std::endl;
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].y())<<std::endl;
  strs<<"EOF"<<std::endl;
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].z())<<std::endl;
  strs<<"EOF"<<std::endl;
  plot_impl_ptr_->write( strs.str() );
}

template <typename _T> 
  void Plot::plotIntervals ( const std::vector< TriadData_< _T > >& samples,
                                     const std::vector< DataInterval >& intervals, 
                                     DataInterval range )
{
   if( !plot_impl_ptr_->ready() )
    return;
   
  range = checkInterval( samples, range );
  int n_pts = range.end_idx - range.start_idx + 1, 
              n_intervals = intervals.size();
  
  double max = 0, mean = 0;
  for( int i = range.start_idx; i <= range.end_idx; i++)
  {
    if( double(samples[i].x()) > max ) max = double(samples[i].x());
    if( double(samples[i].y()) > max ) max = double(samples[i].y());
    if( double(samples[i].z()) > max ) max = double(samples[i].z());
    
    mean += (double(samples[i].x()) + double(samples[i].y()) + double(samples[i].z()))/3;
  }
  
  mean /= n_pts;
  max -= mean;
  double step_h = mean + max/2, val = 0;
  int interval_idx = 0;
  for( ; interval_idx < n_intervals; interval_idx++ )
  {
    if (intervals[interval_idx].start_idx >= range.start_idx )
      break;
  }
  
  std::stringstream strs;
  strs<<"plot '-' title 'x' with lines, "
      <<"'-' title 'y' with lines, "
      <<"'-' title 'z' with lines, "
      <<"'-' title 'intervals' with lines"<<std::endl;
   
  _T base_time = samples[0].timestamp();
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].x())<<std::endl;
  strs<<"EOF"<<std::endl;
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].y())<<std::endl;
  strs<<"EOF"<<std::endl;
  for( int i = range.start_idx; i <= range.end_idx; i++)
    strs<<double(samples[i].timestamp() - base_time)<<" "<<double(samples[i].z())<<std::endl;
  strs<<"EOF"<<std::endl;
  
  for( int i = range.start_idx; i <= range.end_idx; i++)
  {
    if( interval_idx < n_intervals)
    {
      if( i == intervals[interval_idx].start_idx )
        val = step_h;
      else if( i == intervals[interval_idx].end_idx)
      {
        val = 0;
        interval_idx++;
      }
    }
    strs<<double(samples[i].timestamp() - base_time)<<" "<<val<<std::endl;
  }
  strs<<"EOF"<<std::endl;
  
  plot_impl_ptr_->write( strs.str() );
}

void imu_tk::waitForKey()
{
  do
  {
    std::cout<<std::endl<<"Press Enter to continue"<<std::endl;
  }
  while ( getchar() != '\n' );  
}


class imu_tk::Visualizer
{
public:
  Visualizer( const std::string win_name )
  {
    viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>(
                 new pcl::visualization::PCLVisualizer ( win_name ));
    viewer_->setBackgroundColor (0,0,0);
    viewer_->initCameraParameters ();    
    viewer_->spinOnce();
  };
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer>& getHandle() 
  {
    return viewer_; 
  };
  
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
};

VisualizerPtr imu_tk::createVisualizer(const std::string win_name)
{
  return VisualizerPtr( new Visualizer( win_name ) );
}


template <typename _T> 
  void imu_tk::showFrame( VisualizerPtr vis, const _T quat[4], const _T t[4], std::string name )
{
  std::string x_axes_name(name), y_axes_name(name), z_axes_name(name);
  x_axes_name += "_x";
  y_axes_name += "_y";
  z_axes_name += "_z";
  
  _T x_axes_init[3] = {1,0,0}, 
     y_axes_init[3] = {0,1,0}, 
     z_axes_init[3] = {0,0,1}, 
     x_axes[3], y_axes[3], z_axes[3];
  
  ceres::QuaternionRotatePoint(quat, x_axes_init, x_axes);
  ceres::QuaternionRotatePoint(quat, y_axes_init, y_axes);
  ceres::QuaternionRotatePoint(quat, z_axes_init, z_axes);
  
  vis->getHandle()->removeShape(x_axes_name);
  vis->getHandle()->removeShape(y_axes_name);
  vis->getHandle()->removeShape(z_axes_name);
  vis->getHandle()->removeText3D(name);
  
  vis->getHandle()->addLine(pcl::PointXYZ(t[0],t[1],t[2]), 
                            pcl::PointXYZ(t[0] + x_axes[0], t[1] + x_axes[1], t[2] + x_axes[2]), 1, 0, 0, x_axes_name);
  vis->getHandle()->addLine(pcl::PointXYZ(t[0],t[1],t[2]), 
                            pcl::PointXYZ(t[0] + y_axes[0], t[1] + y_axes[1], t[2] + y_axes[2]), 0, 1, 0, y_axes_name);
  vis->getHandle()->addLine(pcl::PointXYZ(t[0],t[1],t[2]), 
                            pcl::PointXYZ(t[0] + z_axes[0], t[1] + z_axes[1], t[2] + z_axes[2]), 0, 0, 1, z_axes_name);
  
  vis->getHandle()->addText3D (name, pcl::PointXYZ(t[0],t[1],t[2]), 0.2 );
}

template <typename _T> 
  void imu_tk::showLine( VisualizerPtr vis, const _T p0[4], const _T p1[4], double r, double g, double b, std::string name )
{
  vis->getHandle()->removeShape(name);
  vis->getHandle()->addLine(pcl::PointXYZ(p0[0],p0[1],p0[2]), pcl::PointXYZ(p1[0],p1[1],p1[2]), r, g, b, name );
}
  
void imu_tk::blockVisualizer( VisualizerPtr vis, int time )
{
  if (time <= 0)
    vis->getHandle()->spin();
  else
    vis->getHandle()->spinOnce( time );
}

template void Plot::plotSamples<double> ( const std::vector< TriadData_<double> >& samples, 
                                          DataInterval range );
template void Plot::plotSamples<float> ( const std::vector< TriadData_<float> >& samples, 
                                         DataInterval range );
template void Plot::plotIntervals<double> ( const std::vector< TriadData_<double> >& samples, 
                                            const std::vector< DataInterval >& intervals,
                                            DataInterval range );
template void Plot::plotIntervals<float> ( const std::vector< TriadData_<float> >& samples, 
                                           const std::vector< DataInterval >& intervals,
                                           DataInterval range );


template void imu_tk::showFrame<double>( VisualizerPtr vis, const double quat[4], const double t[4], 
                                         std::string name );
template void imu_tk::showFrame<float>( VisualizerPtr vis, const float quat[4], const float t[4], 
                                        std::string name );

template void imu_tk::showLine<double>( VisualizerPtr vis, const double p0[4], const double p1[4], 
                                        double r, double g, double b, std::string name );
template void imu_tk::showLine<float>( VisualizerPtr vis, const float p0[4], const float p1[4], 
                                       double r, double g, double b, std::string name );