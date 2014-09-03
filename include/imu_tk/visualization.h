#pragma once

#include "imu_tk/base.h"
#include <boost/shared_ptr.hpp>


namespace imu_tk
{
class Plot;
typedef boost::shared_ptr< Plot > PlotPtr;
PlotPtr createPlot();
void waitForKey();

template <typename _T> 
  void plotSamples( PlotPtr plot, const std::vector< TriadData<_T> > &samples,
                    DataInterval range = DataInterval ( -1, -1 ) );
template <typename _T> 
  void plotIntervals( PlotPtr plot, const std::vector< TriadData<_T> > &samples,
                      const std::vector< DataInterval > &intervals,
                      DataInterval range = DataInterval ( -1, -1 ) );
}