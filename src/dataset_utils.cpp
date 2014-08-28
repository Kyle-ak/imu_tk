#include "imu_tk/dataset_utils.h"


#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

using namespace std;

void imu_tk::importMatlabData ( const char *filename, 
                                std::vector< TriadData > &samples,
                                TimestampUnit unit )
{
  samples.clear();
  
  string line;
  ifstream infile;
  double ts, d[3];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%lf %%lf %%lf %%lf ;");
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2]);
      if ( res != 4 )
        break;
      ts /= unit;
      samples.push_back( TriadData(ts, d ));
    }
    infile.close();
  }
}

void imu_tk::importMatlabData ( const char *filename,
                                std::vector< TriadData > &samples0,
                                std::vector< TriadData > &samples1,
                                TimestampUnit unit )
{
  samples0.clear();
  samples1.clear();
  
  string line;
  ifstream infile;
  double ts, d[6];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf ;");
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2],
                        &d[3], &d[4], &d[5]);
      if ( res != 7 )
        break;
      ts /= unit;
      samples0.push_back( TriadData(ts, d ));
      samples1.push_back( TriadData(ts, d + 3 ));
    }
    infile.close();
  }
}

void imu_tk::importMatlabData ( const char *filename,
                                std::vector< TriadData > &samples0,
                                std::vector< TriadData > &samples1,
                                std::vector< TriadData > &samples2,
                                TimestampUnit unit )
{
  samples0.clear();
  samples1.clear();
  samples2.clear();
  
  string line;
  ifstream infile;
  double ts, d[9];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    sprintf(format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf ;");
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2], 
                        &d[3], &d[4], &d[5], &d[6], &d[7], &d[8]);
      if ( res != 10 )
        break;
      ts /= unit;
      samples0.push_back( TriadData(ts, d ));
      samples1.push_back( TriadData(ts, d + 3 ));
      samples2.push_back( TriadData(ts, d + 6 ));
    }
    infile.close();
  }
}