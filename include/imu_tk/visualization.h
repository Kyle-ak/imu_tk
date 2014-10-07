#pragma once

#include <string>
#include <boost/shared_ptr.hpp>

#include "imu_tk/base.h"


namespace imu_tk
{
class Plot;
typedef boost::shared_ptr< Plot > PlotPtr;
PlotPtr createPlot();

class Visualizer;
typedef boost::shared_ptr< Visualizer > VisualizerPtr;
VisualizerPtr createVisualizer( const std::string win_name = "" );

void waitForKey();

template <typename _T> 
  void plotSamples( PlotPtr plot, const std::vector< TriadData_<_T> > &samples,
                    DataInterval_<_T> range = DataInterval_<_T> ( -1, -1 ) );
template <typename _T> 
  void plotIntervals( PlotPtr plot, const std::vector< TriadData_<_T> > &samples,
                      const std::vector< DataInterval_<_T> > &intervals,
                      DataInterval_<_T> range = DataInterval_<_T> ( -1, -1 ) );
  
template <typename _T> 
  void showFrame( VisualizerPtr vis, const _T quat[4], const _T t[4], std::string name = "frame" );
template <typename _T> 
  void showLine( VisualizerPtr vis, const _T p0[4], const _T p1[4], 
                 double r, double g, double b, std::string name );

void blockVisualizer( VisualizerPtr vis, int time = 0 );
}