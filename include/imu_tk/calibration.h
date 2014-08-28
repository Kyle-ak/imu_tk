#pragma once

#include <vector>

#include "imu_tk/common.h"

namespace imu_tk
{

void staticIntervalsDetector ( const std::vector< imu_tk::TriadData > &samples,
                               double threshold, std::vector< imu_tk::DataInterval > &intervals,
                               int win_size = 101 );

class MultiPosCalib
{

};

}
