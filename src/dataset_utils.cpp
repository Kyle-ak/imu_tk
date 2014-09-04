#include "imu_tk/dataset_utils.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

using namespace std;

template <typename _T> 
  void imu_tk::importAsciiData ( const char *filename, 
                                 vector< TriadData<_T> > &samples,
                                 TimestampUnit unit, DatasetType type )
{
  samples.clear();
  
  string line;
  ifstream infile;
  double ts, d[3];
  
  infile.open ( filename );
  if (infile.is_open()) 
  { 
    char format[266];
    switch( type )
    {
      case DATASET_COMMA_SEPARATED:
        sprintf(format,"%%lf, %%lf, %%lf, %%lf");
        break;
      case DATASET_SPACE_SEPARATED:
      default:
        sprintf(format,"%%lf %%lf %%lf %%lf");
        break;
    }

    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2]);
      if ( res != 4 )
        break;
      ts /= unit;
      samples.push_back( TriadData<_T>(_T(ts), _T(d[0]), _T(d[1]), _T(d[2])) );
    }
    infile.close();
  }
}

template <typename _T> 
  void imu_tk::importAsciiData ( const char *filename,
                                 vector< TriadData<_T> > &samples0,
                                 vector< TriadData<_T> > &samples1,
                                 TimestampUnit unit, DatasetType type )
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
    switch( type )
    {
      case DATASET_COMMA_SEPARATED:
        sprintf(format,"%%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf");
        break;
      case DATASET_SPACE_SEPARATED:
      default:
        sprintf(format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf");
        break;
    }
    
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2],
                        &d[3], &d[4], &d[5]);
      if ( res != 7 )
        break;
      ts /= unit;
      samples0.push_back( TriadData<_T>(_T(ts), _T(d[0]), _T(d[1]), _T(d[2])) );
      samples1.push_back( TriadData<_T>(_T(ts), _T(d[3]), _T(d[4]), _T(d[5])) );
    }
    infile.close();
  }
}

template <typename _T> 
  void imu_tk::importAsciiData ( const char *filename,
                                 vector< TriadData<_T> > &samples0,
                                 vector< TriadData<_T> > &samples1,
                                 vector< TriadData<_T> > &samples2,
                                 TimestampUnit unit, DatasetType type )
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
    switch( type )
    {
      case DATASET_COMMA_SEPARATED:
        sprintf(format,"%%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf");
        break;
      case DATASET_SPACE_SEPARATED:
      default:
        sprintf(format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf");
        break;
    }
    
    while ( getline (infile,line) )
    {
      int res = sscanf (line.data(), format, &ts, &d[0], &d[1], &d[2], 
                        &d[3], &d[4], &d[5], &d[6], &d[7], &d[8]);
      if ( res != 10 )
        break;
      ts /= unit;
      samples0.push_back( TriadData<_T>(_T(ts), _T(d[0]), _T(d[1]), _T(d[2])) );
      samples1.push_back( TriadData<_T>(_T(ts), _T(d[3]), _T(d[4]), _T(d[5])) );
      samples2.push_back( TriadData<_T>(_T(ts), _T(d[6]), _T(d[7]), _T(d[8])) );
    }
    infile.close();
  }
}

template void imu_tk::importAsciiData<double>( const char *filename, 
                                               vector< TriadData<double> > &samples,
                                               TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float>( const char *filename, 
                                                vector< TriadData<float> > &samples,
                                                TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<double> ( const char *filename,
                                                vector< TriadData<double> > &samples0,
                                                vector< TriadData<double> > &samples1,
                                                TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float> ( const char *filename,
                                               vector< TriadData<float> > &samples0,
                                               vector< TriadData<float> > &samples1,
                                               TimestampUnit unit, DatasetType type );  
template void imu_tk::importAsciiData<double> ( const char *filename,
                                                vector< TriadData<double> > &samples0,
                                                vector< TriadData<double> > &samples1,
                                                vector< TriadData<double> > &samples2,
                                                TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float> ( const char *filename,
                                               vector< TriadData<float> > &samples0,
                                               vector< TriadData<float> > &samples1,
                                               vector< TriadData<float> > &samples2,
                                               TimestampUnit unit, DatasetType type );