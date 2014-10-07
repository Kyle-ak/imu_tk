#include "imu_tk/io_utils.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

using namespace std;

template <typename _T>
void imu_tk::importAsciiData ( const char *filename,
                               vector< TriadData_<_T> > &samples,
                               TimestampUnit unit, DatasetType type )
{
  samples.clear();

  string line;
  ifstream infile;
  double ts, d[3];

  infile.open ( filename );
  if ( infile.is_open() )
  {
    char format[266];
    switch ( type )
    {
    case DATASET_COMMA_SEPARATED:
      sprintf ( format,"%%lf, %%lf, %%lf, %%lf" );
      break;
    case DATASET_SPACE_SEPARATED:
    default:
      sprintf ( format,"%%lf %%lf %%lf %%lf" );
      break;
    }

    int l = 0;
    while ( getline ( infile,line ) )
    {
      int res = sscanf ( line.data(), format, &ts, &d[0], &d[1], &d[2] );
      if ( res != 4 )
      {
        cout<<"importAsciiData(): error importing data in line "<<l<<", exit"<<endl;
      }
      else
      {
        ts /= unit;
        samples.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[0] ), _T ( d[1] ), _T ( d[2] ) ) );
      }
      l++;
    }
    infile.close();
  }
}

template <typename _T>
void imu_tk::importAsciiData ( const char *filename,
                               vector< TriadData_<_T> > &samples0,
                               vector< TriadData_<_T> > &samples1,
                               TimestampUnit unit, DatasetType type )
{
  samples0.clear();
  samples1.clear();

  string line;
  ifstream infile;
  double ts, d[6];

  infile.open ( filename );
  if ( infile.is_open() )
  {
    char format[266];
    switch ( type )
    {
    case DATASET_COMMA_SEPARATED:
      sprintf ( format,"%%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf" );
      break;
    case DATASET_SPACE_SEPARATED:
    default:
      sprintf ( format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf" );
      break;
    }

    int l = 0;
    while ( getline ( infile,line ) )
    {
      int res = sscanf ( line.data(), format, &ts, &d[0], &d[1], &d[2],
                         &d[3], &d[4], &d[5] );
      if ( res != 7 )
      {
        cout<<"importAsciiData(): error importing data in line "<<l<<", exit"<<endl;
      }
      else
      {
        ts /= unit;
        samples0.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[0] ), _T ( d[1] ), _T ( d[2] ) ) );
        samples1.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[3] ), _T ( d[4] ), _T ( d[5] ) ) );
      }
      l++;
    }
    infile.close();
  }
}

template <typename _T>
void imu_tk::importAsciiData ( const char *filename,
                               vector< TriadData_<_T> > &samples0,
                               vector< TriadData_<_T> > &samples1,
                               vector< TriadData_<_T> > &samples2,
                               TimestampUnit unit, DatasetType type )
{
  samples0.clear();
  samples1.clear();
  samples2.clear();

  string line;
  ifstream infile;
  double ts, d[9];

  infile.open ( filename );
  if ( infile.is_open() )
  {
    char format[266];
    switch ( type )
    {
    case DATASET_COMMA_SEPARATED:
      sprintf ( format,"%%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf" );
      break;
    case DATASET_SPACE_SEPARATED:
    default:
      sprintf ( format,"%%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf %%lf" );
      break;
    }

    int l = 0;
    while ( getline ( infile,line ) )
    {
      int res = sscanf ( line.data(), format, &ts, &d[0], &d[1], &d[2],
                         &d[3], &d[4], &d[5], &d[6], &d[7], &d[8] );
      if ( res != 10 )
      {
        cout<<"importAsciiData(): error importing data in line "<<l<<", exit"<<endl;
      }
      else
      {
        ts /= unit;
        samples0.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[0] ), _T ( d[1] ), _T ( d[2] ) ) );
        samples1.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[3] ), _T ( d[4] ), _T ( d[5] ) ) );
        samples2.push_back ( TriadData_<_T> ( _T ( ts ), _T ( d[6] ), _T ( d[7] ), _T ( d[8] ) ) );
      }
      l++;
    }
    infile.close();
  }
}

template void imu_tk::importAsciiData<double> ( const char *filename,
    vector< TriadData_<double> > &samples,
    TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float> ( const char *filename,
    vector< TriadData_<float> > &samples,
    TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<double> ( const char *filename,
    vector< TriadData_<double> > &samples0,
    vector< TriadData_<double> > &samples1,
    TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float> ( const char *filename,
    vector< TriadData_<float> > &samples0,
    vector< TriadData_<float> > &samples1,
    TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<double> ( const char *filename,
    vector< TriadData_<double> > &samples0,
    vector< TriadData_<double> > &samples1,
    vector< TriadData_<double> > &samples2,
    TimestampUnit unit, DatasetType type );
template void imu_tk::importAsciiData<float> ( const char *filename,
    vector< TriadData_<float> > &samples0,
    vector< TriadData_<float> > &samples1,
    vector< TriadData_<float> > &samples2,
    TimestampUnit unit, DatasetType type );
