// Copyright (c) 2010, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 \author liljekrans
 \file Trakstar.cpp
 \date Oct 27, 2011
 \brief Class for TrakStar Sensor reading
 */

#include "stdafx.h"
#include "ATC3DG.h"
#include "Trakstar.hpp"
#include <iostream>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rw/rw.hpp>

#include <sys/time.h>

using namespace std;
using namespace rw::common;

using namespace caros;


namespace
{

//#define TRAKSTAR_DATA_RECORD_SIZE                       8


// Change both when changin
//typedef DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
//#define TRAKSTAR_RECORDS_ENUM_TYPE            DOUBLE_POSITION_QUATERNION_TIME_Q_BUTTON

//typedef DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
//#define TRAKSTAR_RECORDS_ENUM_TYPE            DOUBLE_POSITION_MATRIX_TIME_Q_BUTTON

// Get all information possible
typedef DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON_RECORD TRAKSTAR_RECORDS_TYPE;
#define TRAKSTAR_RECORDS_ENUM_TYPE DOUBLE_POSITION_ANGLES_MATRIX_QUATERNION_TIME_Q_BUTTON

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//
//  MACROS for simplifying the procedure calls
//
// This macro will set a system parameter and call the error handler if there is
// an error reported. Note These macros do not print to the standard output the
// set value

#define SET_SYSTEM_PARAMETER(type, value, l)                                    \
        {                                                                           \
            type##_TYPE buffer = value;                                             \
            _errorCode = SetSystemParameter(type, &buffer, sizeof(buffer));         \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

#define SET_SENSOR_PARAMETER(sensor, type, value, l)                            \
        {                                                                           \
            type##_TYPE buffer = value;                                             \
            type##_TYPE *pBuffer = &buffer;                                         \
            _errorCode = SetSensorParameter(sensor, type, pBuffer, sizeof(buffer)); \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

#define SET_TRANSMITTER_PARAMETER(xmtr, type, value, l)                         \
        {                                                                           \
            type##_TYPE buf = value;                                                \
            type##_TYPE *pBuf = &buf;                                               \
            _errorCode = SetTransmitterParameter(xmtr, type, pBuf, sizeof(buf));        \
            if(_errorCode!=BIRD_ERROR_SUCCESS) errorHandler(_errorCode, l);         \
        }

// In order for the above macros to compile without error it is necessary
// to provide typedefs for all the XXX_TYPEs that are generated by "type##_TYPE"
typedef short SELECT_TRANSMITTER_TYPE;
typedef double POWER_LINE_FREQUENCY_TYPE;
// AGC_MODE_TYPE already defined as an enumerated type
typedef double MEASUREMENT_RATE_TYPE;
typedef short REPORT_RATE_TYPE;
typedef double MAXIMUM_RANGE_TYPE;
typedef BOOL METRIC_TYPE;
// DATA_FORMAT_TYPE already defined as an enumerated type
typedef DOUBLE_ANGLES_RECORD ANGLE_ALIGN_TYPE;
typedef DOUBLE_ANGLES_RECORD REFERENCE_FRAME_TYPE;
typedef BOOL XYZ_REFERENCE_FRAME_TYPE;
// HEMISPHERE_TYPE already defined as an enumerated type
typedef BOOL FILTER_AC_WIDE_NOTCH_TYPE;
typedef BOOL FILTER_AC_NARROW_NOTCH_TYPE;
typedef double FILTER_DC_ADAPTIVE_TYPE;
typedef ADAPTIVE_PARAMETERS FILTER_ALPHA_PARAMETERS_TYPE;
typedef BOOL FILTER_LARGE_CHANGE_TYPE;
typedef QUALITY_PARAMETERS QUALITY_TYPE;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// This is a simplified error handler.
// This error handler takes the error code and passes it to the GetErrorText()
// procedure along with a buffer to place an error message string.
// This error message string can then be output to a user display device
// like the console
// Specific error codes should be parsed depending on the application.
//
void errorHandler(int error, int lineNum)
{
  char buffer[1024];
  int currentError = error;
  int nextError;

  do
  {
    nextError = GetErrorText(currentError, buffer, sizeof(buffer), SIMPLE_MESSAGE);
    ROS_ERROR_STREAM(buffer);
    currentError = nextError;
  } while (currentError != BIRD_ERROR_SUCCESS);
}

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}


}

Trakstar::Trakstar():ATC3DG_(NULL),init_status_(Trakstar::TRAKSTAR_STATUS_INITIALIZING)
{

}

Trakstar::~Trakstar()
{
  // First stop polling
  stopPolling();
  _initThread.join();
  CloseBIRDSystem();
}

void Trakstar::initialize(bool block)
{

  // Initialize system by calling _InitializeBird in a thread
  _initThread = boost::thread(boost::bind(&Trakstar::initializeSystem, this));
  if (block)
    _initThread.join();
}

/** 
 * Initialize System by calling library function InitializeBIRDSystem.
 * Should run on plugin startup and thus start in a thread so it doesn't increase startup time
 */
int Trakstar::initializeSystem()
{
  ROS_DEBUG("Setting status!");
  TrakstarStatus initStatusLocal = TRAKSTAR_STATUS_STOPPED;

  ROS_DEBUG("Creating ATC3DG handle!");
  // first initialize variables
  ATC3DG_ = new tagSYSTEM_CONFIGURATION();

  // Set initStatusLocal to "initializing"
  initStatusLocal = TRAKSTAR_STATUS_INITIALIZING;

  // Update init_status_ immediately
  init_status_ = initStatusLocal;

  // Initialize Bird system
  ROS_DEBUG_STREAM("Initializing Trakstar System... This takes some seconds.");

  _errorCode = InitializeBIRDSystem();
  if (_errorCode != BIRD_ERROR_SUCCESS)
  {
    errorHandler(_errorCode, __LINE__);
    initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized

    ROS_ERROR_STREAM("ERROR WHEN INITIALIZING TRAKSTAR");
    ROS_ERROR_STREAM("  error code: " << _errorCode);
    ROS_ERROR_STREAM("  Maybe you forgot to start ATCdaemon64?");
    ROS_FATAL("Trakstar cannot function if not ATCdaemon64 is running! closing!");
  }
  else
  {
    //Log::log().info() << "Initialization successfull." << endl;
    ROS_DEBUG_STREAM("Initialization successfull.");

    // Set system parameters

    // Measurement rate, 80 is standard
    //SET_SYSTEM_PARAMETER( MEASUREMENT_RATE, 80, __LINE__ );

    // Metric (use millimeters)
    SET_SYSTEM_PARAMETER(METRIC, true, __LINE__);

    // Range
    //SET_SYSTEM_PARAMETER(MAXIMUM_RANGE,	72.0, __LINE__);

    // Report Rate (how fast does the box prepare new data). reportRate / ( 3*measure_rate ).
    // the RR is actually a report rate devisor. higher => slower report rate.
    // eg. reportrate = 120: 120 / (3*80) = 0.500seconds per update when using GetSynchronousRecord.
    // if using Asynchronous updates, identical values will be read if GetAsynch.. is called faster than data is prepared.
    // Similarly 1 / (3*80) = 4ms per update or approx. 240hz
    //SET_SYSTEM_PARAMETER( REPORT_RATE, 1, __LINE__ ); // 1 is default:

    //SET_SYSTEM_PARAMETER( POWER_LINE_FREQUENCY, 50, __LINE__ );

    _errorCode = GetBIRDSystemConfiguration(ATC3DG_);
    if (_errorCode != BIRD_ERROR_SUCCESS)
    {
      errorHandler(_errorCode, __LINE__);
      initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized
    }
    else
    {
      ROS_DEBUG_STREAM("Trakstar system configuration information read.");
      ROS_DEBUG_STREAM("Number Boards          = " << ATC3DG_->numberBoards);
      ROS_DEBUG_STREAM("Number Sensors         = " << ATC3DG_->numberSensors);
      ROS_DEBUG_STREAM("Number Transmitters    = " << ATC3DG_->numberTransmitters);

      ROS_DEBUG_STREAM("System AGC mode	     = " << ATC3DG_->agcMode);
      ROS_DEBUG_STREAM("Maximum Range          = " << ATC3DG_->maximumRange);
      ROS_DEBUG_STREAM("Measurement Rate       = " << ATC3DG_->measurementRate);
      ROS_DEBUG_STREAM("Metric Mode            = " << ATC3DG_->metric);
      ROS_DEBUG_STREAM("Line Frequency         = " << ATC3DG_->powerLineFrequency);
      ROS_DEBUG_STREAM("Transmitter ID Running = " << ATC3DG_->transmitterIDRunning);

      // Transmitter set:

      DOUBLE_ANGLES_RECORD anglesRecord = {0, 0, 180};

      SET_TRANSMITTER_PARAMETER(0, REFERENCE_FRAME, anglesRecord, __LINE__);
      SET_TRANSMITTER_PARAMETER(0, XYZ_REFERENCE_FRAME, true, __LINE__);

      // We now know the number of sensors attached. Setup _record etc.
      raw_values_.resize( ATC3DG_->numberSensors*sizeof(TRAKSTAR_RECORDS_TYPE) );
      _records.resize(ATC3DG_->numberSensors);
      _recordsTmp.resize(ATC3DG_->numberSensors);

      //
      // GET TRANSMITTER CONFIGURATION
      //
      // The call to GetTransmitterConfiguration() performs a similar task to the
      // GetSensorConfiguration() call. It also returns a status in the filled
      // structure which indicates whether a transmitter is attached to this
      // port or not. In a single transmitter system it is only necessary to
      // find where that transmitter is in order to turn it on and use it.
      //
      transmitters_.resize(ATC3DG_->numberTransmitters);
      for (int i = 0; i < ATC3DG_->numberTransmitters; i++)
      {
        transmitters_[i] = ownedPtr( new tagTRANSMITTER_CONFIGURATION() );
        _errorCode = GetTransmitterConfiguration(i, transmitters_[i].get());
        if (_errorCode != BIRD_ERROR_SUCCESS)
        {
          errorHandler(_errorCode, __LINE__);
          initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized
        }
        else
        {
          // We have successfully initialized the system.
          initStatusLocal = TRAKSTAR_STATUS_STARTED;
        }
      }

      // Setup sensors
      sensors_.resize(ATC3DG_->numberSensors);
      for (int i = 0; i < ATC3DG_->numberSensors; i++)
      {
        sensors_[i] = ownedPtr( new tagSENSOR_CONFIGURATION() );
        _errorCode = GetSensorConfiguration(i, sensors_[i].get());
        if (_errorCode != BIRD_ERROR_SUCCESS)
          errorHandler(_errorCode, __LINE__);

        // Hemisphere, top..
        SET_SENSOR_PARAMETER(i, HEMISPHERE, TOP, __LINE__);

        // Data format
        SET_SENSOR_PARAMETER(i, DATA_FORMAT, TRAKSTAR_RECORDS_ENUM_TYPE, __LINE__);

        // filtering

      }
      // Read sensor's configuration
      if (0)
      {
        ROS_DEBUG_STREAM("Sensor's configuration:");
        for (int i = 0; i < ATC3DG_->numberSensors; i++)
        {
          ROS_DEBUG_STREAM("Sensor " << (i + 1) << ":");
          //
          // DATA_FORMAT
          //
          {
            DATA_FORMAT_TYPE buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, DATA_FORMAT, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("DATA_FORMAT: "<< buffer);
          }
          //
          // ANGLE_ALIGN
          //
          {
            DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, ANGLE_ALIGN, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("ANGLE_ALIGN: " << buffer.a << "," << buffer.e << "," <<  buffer.r);
          }
          //
          // HEMISPHERE
          //
          {
            HEMISPHERE_TYPE buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, HEMISPHERE, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("HEMISPHERE: " << buffer);
          }
          //
          // FILTER_AC_WIDE_NOTCH
          //
          {
            BOOL buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, FILTER_AC_WIDE_NOTCH, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("FILTER_AC_WIDE_NOTCH: " << buffer);
          }
          //
          // FILTER_AC_NARROW_NOTCH
          //
          {
            BOOL buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, FILTER_AC_NARROW_NOTCH, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("FILTER_AC_NARROW_NOTCH: " << buffer);
          }
          //
          // FILTER_DC_ADAPTIVE
          //
          {
            double buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, FILTER_DC_ADAPTIVE, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("FILTER_DC_ADAPTIVE: " << buffer);
          }
          //
          // FILTER_ALPHA_PARAMETERS
          //
          {
            ADAPTIVE_PARAMETERS buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, FILTER_ALPHA_PARAMETERS, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("FILTER_ALPHA_PARAMETERS:");
            ROS_DEBUG_STREAM("    Alpha max " << buffer.alphaMax[0] << "," << buffer.alphaMax[1]  << "," <<
                   buffer.alphaMax[2]  << "," <<  buffer.alphaMax[3]  << "," <<  buffer.alphaMax[4]  << "," <<
                   buffer.alphaMax[5]  << "," <<  buffer.alphaMax[6]);
            ROS_DEBUG_STREAM("    Alpha Min " << buffer.alphaMin[0] << "," << buffer.alphaMin[1] << "," <<
                   buffer.alphaMin[2] << "," << buffer.alphaMin[3] << "," << buffer.alphaMin[4] << "," << buffer.alphaMin[5] << "," <<
                   buffer.alphaMin[6]);
            ROS_DEBUG_STREAM("    Vm " << "," << buffer.vm[0] << "," << buffer.vm[1] << "," << buffer.vm[2] << "," <<
                   buffer.vm[3] << "," << buffer.vm[4] << "," << buffer.vm[5] << "," << buffer.vm[6]);
            ROS_DEBUG_STREAM("    On/Off " << buffer.alphaOn);
          }
          //
          // FILTER_LARGE_CHANGE
          //
          {
            BOOL buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, FILTER_LARGE_CHANGE, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("FILTER_LARGE_CHANGE: " << buffer);
          }
          //
          // QUALITY
          //
          {
            QUALITY_PARAMETERS buffer, *pBuffer = &buffer;
            _errorCode = GetSensorParameter(i, QUALITY, pBuffer, sizeof(buffer));
            if (_errorCode != BIRD_ERROR_SUCCESS)
              errorHandler(_errorCode, __LINE__);
            ROS_DEBUG_STREAM("QUALITY: " << "," << buffer.error_offset << "," <<
                             buffer.error_sensitivity << "," << buffer.error_slope << "," <<
                             buffer.filter_alpha);
          }

        }
      }
    }
  }

  init_status_ = initStatusLocal;
  return _errorCode;
}

Trakstar::TrakstarStatus Trakstar::getInitStatus()
{
  return init_status_;
}

bool Trakstar::isInitialized(){
  return init_status_==TRAKSTAR_STATUS_STARTED;
}

int Trakstar::getNumberSensorsAttached()
{
  if (init_status_)
    return _sensorsAttached;
  else
    return -1;
}

bool Trakstar::startPolling()
{

  // If we were running. Stop first, and then start.
  if (!_flagStopPoll)
  {
    stopPolling();
  }

  bool ret = false;
  // Search for transmitters. Turn the first one on. (there is only one)
  for (short id = 0; id < ATC3DG_->numberTransmitters; id++)
  {
    if (transmitters_[id]->attached)
    {
      // Transmitter selection is a system function.
      // Using the SELECT_TRANSMITTER parameter we send the id of the
      // transmitter that we want to run with the SetSystemParameter() call
      _errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
      if (_errorCode != BIRD_ERROR_SUCCESS)
      {
        errorHandler(_errorCode, __LINE__);
        return ret;
      }
      break;
    }
  }

  // Set the data format type for each attached sensor.
  for (int i = 0; i < ATC3DG_->numberSensors; i++)
  {
    DATA_FORMAT_TYPE type = TRAKSTAR_RECORDS_ENUM_TYPE
    ;
    _errorCode = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
    if (_errorCode != BIRD_ERROR_SUCCESS)
      errorHandler(_errorCode, __LINE__);
  }

  // Count how many sensors are attached by fetching a single record from all and reading status

  _errorCode = GetSynchronousRecord(ALL_SENSORS, &raw_values_[0], raw_values_.size());
  if (_errorCode != BIRD_ERROR_SUCCESS)
    errorHandler(_errorCode, __LINE__);

  // Read sensorStatus
  _sensorsAttached = 0;
  std::stringstream attached_sensors_ss;
  attached_sensors_ss << "Attached sensors: ";
  bool firstRun = true;
  for (int i = 0; i < ATC3DG_->numberSensors; i++)
  {
    unsigned int status = GetSensorStatus(i);
    if ((status & NOT_ATTACHED) != 0)
      continue;
    _sensorsAttached++;
    if (!firstRun)
      attached_sensors_ss << ", ";
    firstRun = false;
    attached_sensors_ss << (i + 1);
  }
  ROS_DEBUG_STREAM( attached_sensors_ss.str() );

  _flagStopPoll = false;
  // Start polling thread
  ret = true;

  return ret;
}

void Trakstar::pollData()
{
  // scan the sensors (all) non blocking call

  _errorCode = GetAsynchronousRecord(ALL_SENSORS, &raw_values_[0], raw_values_.size());
  if (_errorCode != BIRD_ERROR_SUCCESS)
    errorHandler(_errorCode, __LINE__);


  TRAKSTAR_RECORDS_TYPE* raw_records = (TRAKSTAR_RECORDS_TYPE*)&raw_values_[0];
  // Get status of sensors (only updates after a Get***Record call)
  for (short id = 0; id < ATC3DG_->numberSensors; id++)
  {
    // get the status of the last data record
    // only report the data if everything is okay
    ULONG status = GetSensorStatus(id);

    // Set default state
    _recordsTmp[id].status = status;
    _recordsTmp[id].valid = false;
    if (status == VALID_STATUS)
    {
      _analogButtonOn = (bool)(raw_records[id].button);
      _recordsTmp[id].analogButtonOn = _analogButtonOn;

      // Copy raw data into records
      _recordsTmp[id].pos[0] = raw_records[id].x;
      _recordsTmp[id].pos[1] = raw_records[id].y;
      _recordsTmp[id].pos[2] = raw_records[id].z;
      _recordsTmp[id].rot(3) = raw_records[id].q[0]; // Scalar component
      _recordsTmp[id].rot(0) = raw_records[id].q[1]; // qx
      _recordsTmp[id].rot(1) = raw_records[id].q[2]; // qy
      _recordsTmp[id].rot(2) = raw_records[id].q[3]; // qz
      _recordsTmp[id].time = raw_records[id].time;
      _recordsTmp[id].quality = raw_records[id].quality * 1.0 / 65536.0;
      _recordsTmp[id].valid = true;

    }
    else if (status == (SATURATED | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Sensor[" << id << "] is saturated.");
    }
    else if (status == (OUT_OF_MOTIONBOX | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Sensor[" << id << "] is out of range.");
    }
    else if (status == (NOT_ATTACHED | GLOBAL_ERROR))
    {
      // Don't tell us that a sensor is not attached. We probably(hopefully!) know
      // This would be a place to debug for sensor-data not read if that error present.
      //ROS_ERROR_STREAM("No sensor [" << id << "] attached! ");
    }
    else if (status == (NO_TRANSMITTER_RUNNING | GLOBAL_ERROR))
    {
      ROS_ERROR_STREAM("Transmitter not ready!");
    }
    else
    {
      ROS_ERROR_STREAM("Sensor[" << id << "]: status not valid: " << status - 1);
    }
  }

  _records = _recordsTmp;
}

void Trakstar::stopPolling()
{

  // Signal to _poll to stop polling
  _flagStopPoll = true;

  // Get active transmitter
  short buffer, *pBuffer = &buffer;
  _errorCode = GetSystemParameter(SELECT_TRANSMITTER, pBuffer, sizeof(buffer));
  if (_errorCode != BIRD_ERROR_SUCCESS)
  {
    errorHandler(_errorCode, __LINE__);
    return;
  }

  // Stop transmitter if one was turned on
  if (buffer != -1)
  {
    short id = -1;
    _errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (_errorCode != BIRD_ERROR_SUCCESS)
      errorHandler(_errorCode, __LINE__);
  }
}

std::vector<Trakstar::PoseData> Trakstar::getData(void)
{
  pollData();
  return _records;
}

std::string Trakstar::getSensorStatusString(int errorCode)
{
  if (errorCode == VALID_STATUS)
  {
    return "OK";

  }
  else if (errorCode & NOT_ATTACHED)
  {
    return "NOT CONNECTED";
  }
  else if (errorCode & OUT_OF_MOTIONBOX)
  {
    return "RANGE";
  }
  else if (errorCode & SATURATED)
  {
    return "SATURATED";
  }

  // else
  stringstream ss;
  ss << "e" << errorCode;
  return ss.str();
}

int Trakstar::getNumberSensorsSupported()
{
  return ATC3DG_->numberSensors;
}
