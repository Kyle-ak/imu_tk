#include <pcl/visualization/cloud_viewer.h>
#include <ceres/rotation.h>

#include "imu_tk/visualization.h"
#include "imu_tk/gnuplot_i.h"

class imu_tk::Plot
{
public:
  Plot(){ hplot_ = gnuplot_init(); };
  ~Plot(){ gnuplot_close(hplot_); };
  
  gnuplot_ctrl *getHandle() {return hplot_; };
private:
  gnuplot_ctrl *hplot_;
};

imu_tk::PlotPtr imu_tk::createPlot()
{
  return imu_tk::PlotPtr ( new imu_tk::Plot() );
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

imu_tk::VisualizerPtr imu_tk::createVisualizer(const std::string win_name)
{
  return imu_tk::VisualizerPtr( new imu_tk::Visualizer( win_name ) );

}

void imu_tk::waitForKey()
{
  while (getchar()!='\n');  
}

template <typename _T> 
  void imu_tk::plotSamples ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<_T> >& samples, 
                             imu_tk::DataInterval_<_T> range )
{
  range = imu_tk::checkInterval( samples, range );
  int n_pts = range.end_idx - range.start_idx + 1;;
  std::vector<double> ts(n_pts), x(n_pts), y(n_pts), z(n_pts);
    
  gnuplot_resetplot(plot->getHandle());
  gnuplot_setstyle(plot->getHandle(), "lines") ;
  
  _T base_time = samples[0].timestamp();
  for( int i = range.start_idx; i <= range.end_idx; i++)
  {
    ts[i] = double(samples[i].timestamp() - base_time);
    x[i] = double(samples[i].x());
    y[i] = double(samples[i].y());
    z[i] = double(samples[i].z());
  }  

  gnuplot_plot_xy(plot->getHandle(), ts.data(), x.data(), n_pts, "x") ;
  gnuplot_plot_xy(plot->getHandle(), ts.data(), y.data(), n_pts, "y") ;
  gnuplot_plot_xy(plot->getHandle(), ts.data(), z.data(), n_pts, "z") ;
}

template <typename _T> 
  void imu_tk::plotIntervals ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<_T> >& samples, 
                               const std::vector< imu_tk::DataInterval_<_T> >& intervals,
                               imu_tk::DataInterval_<_T> range )
{
  range = imu_tk::checkInterval( samples, range );
  int n_pts = range.end_idx - range.start_idx + 1, 
              n_intervals = intervals.size();
  std::vector<double> ts(n_pts), intervals_plot(n_pts);
  
  imu_tk::plotSamples<_T> (plot, samples, range );
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
 
  _T base_time = samples[0].timestamp();
  for( int i = range.start_idx; i <= range.end_idx; i++)
  {
    ts[i] = double(samples[i].timestamp() - base_time);
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
    intervals_plot[i] = val;
  }
  gnuplot_plot_xy(plot->getHandle(), ts.data(), intervals_plot.data(), n_pts, "intervals") ;
}

template <typename _T> 
  void imu_tk::showFrame( imu_tk::VisualizerPtr vis, const _T quat[4], const _T t[4], std::string name )
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
  
void imu_tk::blockVisualizer(imu_tk::VisualizerPtr vis, int time )
{
  if (time <= 0)
    vis->getHandle()->spin();
  else
    vis->getHandle()->spinOnce( time );
}

template void imu_tk::plotSamples<double> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<double> >& samples, 
                                            imu_tk::DataInterval_<double> range );
template void imu_tk::plotSamples<float> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<float> >& samples, 
                                           imu_tk::DataInterval_<float> range );
template void imu_tk::plotIntervals<double> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<double> >& samples, 
                                              const std::vector< imu_tk::DataInterval_<double> >& intervals,
                                              imu_tk::DataInterval_<double> range );
template void imu_tk::plotIntervals<float> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData_<float> >& samples, 
                                             const std::vector< imu_tk::DataInterval_<float> >& intervals,
                                             imu_tk::DataInterval_<float> range );


template void imu_tk::showFrame<double>( imu_tk::VisualizerPtr vis, const double quat[4], const double t[4], 
                                         std::string name );
template void imu_tk::showFrame<float>( imu_tk::VisualizerPtr vis, const float quat[4], const float t[4], 
                                        std::string name );

template void imu_tk::showLine<double>( VisualizerPtr vis, const double p0[4], const double p1[4], 
                                double r, double g, double b, std::string name );
template void imu_tk::showLine<float>( VisualizerPtr vis, const float p0[4], const float p1[4], 
                                double r, double g, double b, std::string name );