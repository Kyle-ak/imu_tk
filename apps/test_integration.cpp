#include <iostream>

#include "imu_tk/io_utils.h"
#include "imu_tk/calibration.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"
#include "imu_tk/visualization.h"

using namespace std;
using namespace imu_tk;
using namespace Eigen;

int main(int argc, char** argv)
{
  if( argc < 2 )
    return -1;

  vector< TriadData > acc_data, gyro_data, mag_data;
  
  cout<<"Importing IMU data from file : "<< argv[1]<<endl;  
  importAsciiData( argv[1], acc_data, gyro_data, mag_data, 
                   imu_tk::TIMESTAMP_UNIT_USEC, imu_tk::DATASET_SPACE_SEPARATED);
  
  Vector3d gyro_bias = dataMean( gyro_data, DataInterval(100, 3000));
  CalibratedTriad bias_calib;
  bias_calib.setBias(gyro_bias);

  for(int i = 0; i < gyro_data.size(); i++)
    gyro_data[i] = bias_calib.unbias(gyro_data[i]);

  Vector3d mag_mean = dataMean( mag_data, DataInterval(100, 3000));
  
  VisualizerPtr vis = createVisualizer();
  
  Eigen::Vector4d quat(1.0, 0, 0, 0); // Identity quaternion
  double t[3] = {0, 0, 0};
  showFrame(vis, quat.data(), t, "ref");
  showLine(vis, t, mag_mean.data(), 1,1,1, "init_mag");
  
  for(int i = 3000; i < gyro_data.size() - 1; i++)
  {
    if( !(i%100) )
      std::cout<<i/100<<std::endl;
    double dt = gyro_data[i+1].timestamp() - gyro_data[i].timestamp();
    
    quatIntegrationStepRK4( quat, gyro_data[i].data(), gyro_data[i + 1].data(), dt, quat );
    showFrame(vis, quat.data(), t, "cur");
    showLine( vis, t, mag_data[i].data().data(), 1, 1, 1, "mag" );
    blockVisualizer(vis, 10);
  }
  std::cout<<"Done"<<std::endl;
  blockVisualizer(vis);
  return 0;
}