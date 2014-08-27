#include "imu_tk/dataset_utils.h"


#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

using namespace std;

void imu_tk::importMatlabData ( const char *filename, 
                                std::vector< imu_tk::TriadData > &data )
{
  data.clear();
  
  string line;
  ifstream infile;
  uint64_t ts;
  double d[3];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%%s %%lf %%lf %%lf ;", SCNu64);
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format,&ts, &d[0], &d[1], &d[2]);
      if ( res != 4 )
        break;
      data.push_back( TriadData(ts, d ));
    }
    infile.close();
  }
}

void imu_tk::importMatlabData ( const char *filename,
                                std::vector< imu_tk::TriadData > &data0,
                                std::vector< imu_tk::TriadData > &data1 )
{
  data0.clear();
  data1.clear();
  
  string line;
  ifstream infile;
  uint64_t ts;
  double d[6];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%%s %%lf %%lf %%lf %%lf %%lf %%lf ;", SCNu64);
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format,&ts, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]);
      if ( res != 7 )
        break;
      data0.push_back( TriadData(ts, d ));
      data1.push_back( TriadData(ts, d + 3 ));
    }
    infile.close();
  }
}

void imu_tk::importMatlabData ( const char *filename,
                                std::vector< imu_tk::TriadData > &data0,
                                std::vector< imu_tk::TriadData > &data1,
                                std::vector< imu_tk::TriadData > &data2 )
{
  data0.clear();
  data1.clear();
  data2.clear();
  
  string line;
  ifstream infile;
  uint64_t ts;
  double d[9];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%%s %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf ;", SCNu64);
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format,&ts, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7], &d[8]);
      if ( res != 10 )
        break;
      data0.push_back( TriadData(ts, d ));
      data1.push_back( TriadData(ts, d + 3 ));
      data2.push_back( TriadData(ts, d + 6 ));
    }
    infile.close();
  }
}