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
  return PlotPtr ( new Plot() );
}

void imu_tk::waitForKey()
{
  while (getchar()!='\n');  
}

void imu_tk::plotSamples ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData >& samples )
{
  int n_pts = samples.size();
  double ts[n_pts], x[n_pts], y[n_pts], z[n_pts];
  
  gnuplot_resetplot(plot->getHandle());
  gnuplot_setstyle(plot->getHandle(), "lines") ;
  
  double base_time = samples[0].timestamp();
  for( int i = 0; i < n_pts; i++)
  {
    ts[i] = samples[i].timestamp() - base_time;
    x[i] = samples[i].x();
    y[i] = samples[i].y();
    z[i] = samples[i].z();
  }  

  gnuplot_plot_xy(plot->getHandle(), ts, x, n_pts, "x") ;
  gnuplot_plot_xy(plot->getHandle(), ts, y, n_pts, "y") ;
  gnuplot_plot_xy(plot->getHandle(), ts, z, n_pts, "z") ;
}

void imu_tk::plotIntervals ( imu_tk::PlotPtr plot, const std::vector< imu_tk::TriadData >& samples, const std::vector< imu_tk::DataInterval >& intervals )
{
  int n_pts = samples.size(), n_intervals = intervals.size();
  double ts[n_pts], intervals_plot[n_pts];
  
  plotSamples (plot, samples );
  double max = 0;
  for( int i = 0; i < n_pts; i++ )
  {
    if( samples[i].x() > max ) max = samples[i].x();
    if( samples[i].y() > max ) max = samples[i].y();
    if( samples[i].z() > max ) max = samples[i].z();    
  }
  
  double step_h = max/2, val = 0;
  int interval_idx = 0;
 
  double base_time = samples[0].timestamp();
  for( int i = 0; i < n_pts; i++ )
  {
    ts[i] = samples[i].timestamp() - base_time;
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
  gnuplot_plot_xy(plot->getHandle(), ts, intervals_plot, n_pts, "intervals") ;
}
