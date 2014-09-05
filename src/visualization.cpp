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

void imu_tk::waitForKey()
{
  while (getchar()!='\n');  
}

template <typename _T> 
  void imu_tk::plotSamples ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<_T> >& samples, 
                             imu_tk::DataInterval<_T> range )
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
  void imu_tk::plotIntervals ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<_T> >& samples, 
                               const std::vector< imu_tk::DataInterval<_T> >& intervals,
                               imu_tk::DataInterval<_T> range )
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

template void imu_tk::plotSamples<double> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<double> >& samples, 
                                            imu_tk::DataInterval<double> range );
template void imu_tk::plotSamples<float> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<float> >& samples, 
                                           imu_tk::DataInterval<float> range );
template void imu_tk::plotIntervals<double> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<double> >& samples, 
                                              const std::vector< imu_tk::DataInterval<double> >& intervals,
                                              imu_tk::DataInterval<double> range );
template void imu_tk::plotIntervals<float> ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData<float> >& samples, 
                                             const std::vector< imu_tk::DataInterval<float> >& intervals,
                                             imu_tk::DataInterval<float> range );