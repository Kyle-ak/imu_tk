#pragma once

#include "imu_tk/common.h"
#include <boost/shared_ptr.hpp>


namespace imu_tk
{
  class Plot;
  typedef boost::shared_ptr< Plot > PlotPtr;
  PlotPtr createPlot();
  void waitForKey();
  
  void plotSamples( PlotPtr plot, const std::vector< TriadData > &samples );
  void plotIntervals( PlotPtr plot, const std::vector< TriadData > &samples,
                      const std::vector< DataInterval > &intervals );
}