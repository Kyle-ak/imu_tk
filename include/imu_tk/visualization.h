#pragma once

#include <string>
#include <boost/shared_ptr.hpp>

#include "imu_tk/base.h"


namespace imu_tk
{
  
class Plot
{
public:
  Plot();
  ~Plot(){};
  
  template <typename _T> 
    void plotSamples( const std::vector< TriadData_<_T> > &samples,
                      DataInterval range = DataInterval() );
  template <typename _T> 
    void plotIntervals( const std::vector< TriadData_<_T> > &samples,
                        const std::vector< DataInterval > &intervals,
                        DataInterval range = DataInterval() );
private:

  /* Pimpl idiom */
  class PlotImpl; 
  boost::shared_ptr< PlotImpl > plot_impl_ptr_;
};

void waitForKey();

class Visualizer;
typedef boost::shared_ptr< Visualizer > VisualizerPtr;
VisualizerPtr createVisualizer( const std::string win_name = "" );

  
template <typename _T> 
  void showFrame( VisualizerPtr vis, const _T quat[4], const _T t[4], std::string name = "frame" );
template <typename _T> 
  void showLine( VisualizerPtr vis, const _T p0[4], const _T p1[4], 
                 double r, double g, double b, std::string name );

void blockVisualizer( VisualizerPtr vis, int time = 0 );

}