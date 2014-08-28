#include <iostream>

#include "imu_tk/dataset_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

int main(int argc, char** argv)
{
  if( argc < 2 )
    return -1;
  

  vector< TriadData > acc_data, gyro_data;
  
  cout<<"Importing IMU data from the Matlab matrix file : "<< argv[1]<<endl;
  
  importMatlabData( argv[1], acc_data, gyro_data );
  
  Array3d variance = dataVariance( acc_data, DataInterval( 0, 3000) );
  double magnitude_th = (variance*variance).sum();
  std::vector< imu_tk::DataInterval > statc_intervals;
  
  cout<<statc_intervals.size()<<endl;
  
//   for( int i = 0; i < acc_data.size(); i++)
//   {
//     cout<<acc_data[i].timestamp()<<" "
//         <<acc_data[i].x()<<" "<<acc_data[i].y()<<" "<<acc_data[i].z()<<" "
//         <<gyro_data[i].x()<<" "<<gyro_data[i].y()<<" "<<gyro_data[i].z()<<endl;
//   }
//   cout<<"Read "<<acc_data.size()<<" tuples"<<endl;
  
  PlotPtr plot = createPlot();
  
  for (int i = 1; i < 10; i++)
  {
    staticIntervalsDetector ( acc_data, i*magnitude_th, statc_intervals );
    plotIntervals( plot, acc_data, statc_intervals ) ;
    waitForKey();
  }
  
  return 0;
}