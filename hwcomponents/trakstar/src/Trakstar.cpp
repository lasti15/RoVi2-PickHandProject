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

namespace {

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

}

// forward declaration of error handler
void errorHandler(int error, int lineNum);
timespec diff(timespec start, timespec end);

Trakstar::Trakstar() {

	_initStatus = -1;
}

Trakstar::~Trakstar() {
	// First stop polling
	stopPolling();
	_initThread.join();
	CloseBIRDSystem();
}

void Trakstar::InitializeSystem(bool block) {

	// Initialize system by calling _InitializeBird in a thread
	_initThread = boost::thread(boost::bind(&Trakstar::InitializeBird, this));
	if (block)
		_initThread.join();
}

/** 
 * Initialize System by calling library function InitializeBIRDSystem.
 * Should run on plugin startup and thus start in a thread so it doesn't increase startup time
 */
int Trakstar::InitializeBird() {
	int initStatusLocal = TRAKSTAR_STATUS_STOPPED;

	// Set initStatusLocal to "initializing"
	initStatusLocal = TRAKSTAR_STATUS_INITIALIZING;

	// Update _initStatus immediately
	_initStatus = initStatusLocal;

	// Initialize Bird system
	std::cout << "Initializing Trakstar System... This takes some seconds."
			<< endl;

	_errorCode = InitializeBIRDSystem();
	if (_errorCode != BIRD_ERROR_SUCCESS) {
		errorHandler(_errorCode, __LINE__);
		initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized

		ROS_ERROR_STREAM("ERROR WHEN INITIALIZING TRAKSTAR");
		ROS_ERROR_STREAM("  error code: " << _errorCode);
		ROS_ERROR_STREAM("  Maybe you forgot to start ATCdaemon64?");
		ROS_FATAL("Trakstar cannot function if not ATCdaemon64 is running! closing!");

	} else {
		//Log::log().info() << "Initialization successfull." << endl;
		std::cout << "Initialization successfull." << endl;

		// Set system parameters

		// Measurement rate, 80 is standard
		//SET_SYSTEM_PARAMETER( MEASUREMENT_RATE, 80, __LINE__ );

		// Metric (use millimeters)
		SET_SYSTEM_PARAMETER( METRIC, true, __LINE__);

		// Range
		//SET_SYSTEM_PARAMETER(MAXIMUM_RANGE,	72.0, __LINE__);

		// Report Rate (how fast does the box prepare new data). reportRate / ( 3*measure_rate ).
		// the RR is actually a report rate devisor. higher => slower report rate.
		// eg. reportrate = 120: 120 / (3*80) = 0.500seconds per update when using GetSynchronousRecord.
		// if using Asynchronous updates, identical values will be read if GetAsynch.. is called faster than data is prepared.
		// Similarly 1 / (3*80) = 4ms per update or approx. 240hz
		//SET_SYSTEM_PARAMETER( REPORT_RATE, 1, __LINE__ ); // 1 is default:

		//SET_SYSTEM_PARAMETER( POWER_LINE_FREQUENCY, 50, __LINE__ );

		_errorCode = GetBIRDSystemConfiguration(&_ATC3DG);
		if (_errorCode != BIRD_ERROR_SUCCESS) {
			errorHandler(_errorCode, __LINE__);
			initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized
		} else {
			std::cout << "Trakstar system configuration information read."
					<< endl;
			std::cout << "Number Boards          = " << _ATC3DG.numberBoards
					<< endl;
			std::cout << "Number Sensors         = " << _ATC3DG.numberSensors
					<< endl;
			std::cout << "Number Transmitters    = "
					<< _ATC3DG.numberTransmitters << endl << endl;

			std::cout << "System AGC mode	     = " << _ATC3DG.agcMode << endl;
			std::cout << "Maximum Range          = " << _ATC3DG.maximumRange
					<< endl;
			std::cout << "Measurement Rate       = " << _ATC3DG.measurementRate
					<< endl;
			std::cout << "Metric Mode            = " << _ATC3DG.metric << endl;
			std::cout << "Line Frequency         = "
					<< _ATC3DG.powerLineFrequency << endl;
			std::cout << "Transmitter ID Running = "
					<< _ATC3DG.transmitterIDRunning << endl;

			// Transmitter set:

			DOUBLE_ANGLES_RECORD anglesRecord = { 0, 0, 180 };

			SET_TRANSMITTER_PARAMETER(0, REFERENCE_FRAME, anglesRecord,
					__LINE__);
			SET_TRANSMITTER_PARAMETER(0, XYZ_REFERENCE_FRAME, true, __LINE__);

			// We now know the number of sensors attached. Setup _record etc.
			_rawValues.resize(_ATC3DG.numberSensors);
			_records.resize(_ATC3DG.numberSensors);
			_recordsTmp.resize(_ATC3DG.numberSensors);

			//
			// GET TRANSMITTER CONFIGURATION
			//
			// The call to GetTransmitterConfiguration() performs a similar task to the
			// GetSensorConfiguration() call. It also returns a status in the filled
			// structure which indicates whether a transmitter is attached to this
			// port or not. In a single transmitter system it is only necessary to
			// find where that transmitter is in order to turn it on and use it.
			//
			_pXmtr.resize(_ATC3DG.numberTransmitters);
			for (int i = 0; i < _ATC3DG.numberTransmitters; i++) {
				_errorCode = GetTransmitterConfiguration(i, &_pXmtr[i]);
				if (_errorCode != BIRD_ERROR_SUCCESS) {
					errorHandler(_errorCode, __LINE__);
					initStatusLocal = TRAKSTAR_STATUS_STOPPED; // Failed. Not initialized
				} else {

					// We have successfully initialized the system.
					initStatusLocal = TRAKSTAR_STATUS_STARTED;
				}
			}

			// Setup sensors
			_pSensor.resize(_ATC3DG.numberSensors);
			for (int i = 0; i < _ATC3DG.numberSensors; i++) {
				_errorCode = GetSensorConfiguration(i, &_pSensor[i]);
				if (_errorCode != BIRD_ERROR_SUCCESS)
					errorHandler(_errorCode, __LINE__);

				// Hemisphere, top..
				SET_SENSOR_PARAMETER(i, HEMISPHERE, TOP, __LINE__);

				// Data format
				SET_SENSOR_PARAMETER(i, DATA_FORMAT, TRAKSTAR_RECORDS_ENUM_TYPE,
						__LINE__);

				// filtering

			}
			// Read sensor's configuration
			if (0) {
				cout << "Sensor's configuration:" << endl;
				for (int i = 0; i < _ATC3DG.numberSensors; i++) {
					cout << "Sensor " << (i + 1) << ":" << endl;
					//
					// DATA_FORMAT
					//
					{
						DATA_FORMAT_TYPE buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, DATA_FORMAT, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("DATA_FORMAT: %d\n", buffer);
					}
					//
					// ANGLE_ALIGN
					//
					{
						DOUBLE_ANGLES_RECORD buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, ANGLE_ALIGN, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("ANGLE_ALIGN: %6.2f, %6.2f, %6.2f\n", buffer.a,
								buffer.e, buffer.r);
					}
					//
					// HEMISPHERE
					//
					{
						HEMISPHERE_TYPE buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, HEMISPHERE, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("HEMISPHERE: %4x\n", buffer);
					}
					//
					// FILTER_AC_WIDE_NOTCH
					//
					{
						BOOL buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, FILTER_AC_WIDE_NOTCH,
								pBuffer, sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						cout << "FILTER_AC_WIDE_NOTCH: " << buffer << endl;
					}
					//
					// FILTER_AC_NARROW_NOTCH
					//
					{
						BOOL buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i,
								FILTER_AC_NARROW_NOTCH, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						cout << "FILTER_AC_NARROW_NOTCH: " << buffer << endl;
					}
					//
					// FILTER_DC_ADAPTIVE
					//
					{
						double buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, FILTER_DC_ADAPTIVE,
								pBuffer, sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("FILTER_DC_ADAPTIVE: %5.2f\n", buffer);
					}
					//
					// FILTER_ALPHA_PARAMETERS
					//
					{
						ADAPTIVE_PARAMETERS buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i,
								FILTER_ALPHA_PARAMETERS, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("FILTER_ALPHA_PARAMETERS:\n");
						printf(
								"    Alpha max   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
								buffer.alphaMax[0], buffer.alphaMax[1],
								buffer.alphaMax[2], buffer.alphaMax[3],
								buffer.alphaMax[4], buffer.alphaMax[5],
								buffer.alphaMax[6]);
						printf(
								"    Alpha Min   %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
								buffer.alphaMin[0], buffer.alphaMin[1],
								buffer.alphaMin[2], buffer.alphaMin[3],
								buffer.alphaMin[4], buffer.alphaMin[5],
								buffer.alphaMin[6]);
						printf(
								"    Vm          %5d, %5d, %5d, %5d, %5d, %5d, %5d\n",
								buffer.vm[0], buffer.vm[1], buffer.vm[2],
								buffer.vm[3], buffer.vm[4], buffer.vm[5],
								buffer.vm[6]);
						printf("    On/Off      %5d\n", buffer.alphaOn);
					}
					//
					// FILTER_LARGE_CHANGE
					//
					{
						BOOL buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, FILTER_LARGE_CHANGE,
								pBuffer, sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("FILTER_LARGE_CHANGE: %d\n", buffer);
					}
					//
					// QUALITY
					//
					{
						QUALITY_PARAMETERS buffer, *pBuffer = &buffer;
						_errorCode = GetSensorParameter(i, QUALITY, pBuffer,
								sizeof(buffer));
						if (_errorCode != BIRD_ERROR_SUCCESS)
							errorHandler(_errorCode, __LINE__);
						printf("QUALITY: %d, %d, %d, %d\n", buffer.error_offset,
								buffer.error_sensitivity, buffer.error_slope,
								buffer.filter_alpha);
					}

				}
			}
		}
	}

	_initStatus = initStatusLocal;
	return _errorCode;
}

int Trakstar::getInitStatus() {
	return _initStatus;
}

int Trakstar::getNumberSensorsAttached() {
	if (_initStatus)
		return _sensorsAttached;
	else
		return -1;
}

bool Trakstar::startPolling() {

	// If we were running. Stop first, and then start.
	if (!_flagStopPoll) {
		stopPolling();
	}

	bool ret = false;
	// Search for transmitters. Turn the first one on. (there is only one)
	for (short id = 0; id < _ATC3DG.numberTransmitters; id++) {
		if (_pXmtr[id].attached) {

			// Transmitter selection is a system function.
			// Using the SELECT_TRANSMITTER parameter we send the id of the
			// transmitter that we want to run with the SetSystemParameter() call
			_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id,
					sizeof(id));
			if (_errorCode != BIRD_ERROR_SUCCESS) {

				errorHandler(_errorCode, __LINE__);

				return ret;
			}

			break;
		}
	}

	// Set the data format type for each attached sensor.
	for (int i = 0; i < _ATC3DG.numberSensors; i++) {
		DATA_FORMAT_TYPE type = TRAKSTAR_RECORDS_ENUM_TYPE;
		_errorCode = SetSensorParameter(i, DATA_FORMAT, &type, sizeof(type));
		if (_errorCode != BIRD_ERROR_SUCCESS)
			errorHandler(_errorCode, __LINE__);
	}

	// Count how many sensors are attached by fetching a single record from all and reading status
	_errorCode = GetSynchronousRecord(ALL_SENSORS, &_rawValues[0],
			sizeof(_rawValues[0]) * _rawValues.size());
	if (_errorCode != BIRD_ERROR_SUCCESS)
		errorHandler(_errorCode, __LINE__);

	// Read sensorStatus
	_sensorsAttached = 0;
	cout << "Attached sensors: ";
	bool firstRun = true;
	for (int i = 0; i < _ATC3DG.numberSensors; i++) {
		unsigned int status = GetSensorStatus(i);

		if ((status & NOT_ATTACHED) == 0) {
			_sensorsAttached++;
			if (!firstRun) {
				cout << ", ";
			} else {
				firstRun = false;
			}

			cout << (i + 1);

		}
	}
	cout << endl;

	_flagStopPoll = false;
	// Start polling thread
	_pollThread = boost::thread(boost::bind(&Trakstar::pollData, this));

	ret = true;

	return ret;
}

void Trakstar::pollData() {
	// For measuring measurement-cycle-rate
	//timespec time1, time2;

	while (1) {

		if (!_flagStopPoll) {

			// Sleep to enable other threads to read data.
			boost::this_thread::sleep(boost::posix_time::milliseconds(2));

			// scan the sensors (all) (blocking call - if REPORT_RATE is low, this will also block RWS plugin ( = lag)
			// , because it reads the values using mutex)

			_errorCode = GetSynchronousRecord(ALL_SENSORS, &_rawValues[0],
					sizeof(_rawValues[0]) * _rawValues.size());
			if (_errorCode != BIRD_ERROR_SUCCESS)
				errorHandler(_errorCode, __LINE__);

			// Measure how fast we are polling
			// Will output eg. "time: 240.000 hz" per cycle and often crash the GUI at those speeds.
			//clock_gettime(CLOCK_REALTIME, &time2);
			//cout << "time: " << 1/ (((double)diff(time1,time2).tv_sec) + ((double)diff(time1,time2).tv_nsec / 1000000000)) << " hz" << endl;
			//clock_gettime(CLOCK_REALTIME, &time1);

			// Get status of sensors (only updates after a Get***Record call)
			//cout << "status: ";
			for (short id = 0; id < _ATC3DG.numberSensors; id++) {

				// get the status of the last data record
				// only report the data if everything is okay

				ULONG status = GetSensorStatus(id);

				// Set default state
				_recordsTmp[id].status = status;
				_recordsTmp[id].valid = false;
				//cout << status << ", ";
				if (status == VALID_STATUS) {
					_analogButtonOn = (bool) (_rawValues[id].button);
					_recordsTmp[id].analogButtonOn = _analogButtonOn;

					// Copy raw data into records
					_recordsTmp[id].pos[0] = _rawValues[id].x;
					_recordsTmp[id].pos[1] = _rawValues[id].y;
					_recordsTmp[id].pos[2] = _rawValues[id].z;
					_recordsTmp[id].rot(3) = _rawValues[id].q[0]; // Scalar component
					_recordsTmp[id].rot(0) = _rawValues[id].q[1]; // qx
					_recordsTmp[id].rot(1) = _rawValues[id].q[2]; // qy
					_recordsTmp[id].rot(2) = _rawValues[id].q[3]; // qz
					_recordsTmp[id].time = _rawValues[id].time;
					_recordsTmp[id].quality = _rawValues[id].quality * 1.0
							/ 65536.0;
					_recordsTmp[id].valid = true;

				} else if (status == (SATURATED | GLOBAL_ERROR)) {
					cout << "Sensor[" << id << "] is saturated." << endl;
				} else if (status == (OUT_OF_MOTIONBOX | GLOBAL_ERROR)) {
					//cout << "Sensor[" << id << "] is out of range." << endl;
					// Don't spam this...
				} else if (status == (NOT_ATTACHED | GLOBAL_ERROR)) {
					// Don't tell us that a sensor is not attached. We probably(hopefully!) know
					// This would be a place to debug for sensor-data not read if that error present.
					//cout << "Not attached sensor" << endl;

				} else if (status == (NO_TRANSMITTER_RUNNING | GLOBAL_ERROR)) {
					//cout << "Transmitter not ready." << endl;					
				} else {
					cout << "Sensor[" << id << "]: status not valid: "
							<< status - 1 << endl;
				}
			}

			//cout << endl;

			// Mutex is scoped and is thus released automatically.
			{
				// Get mutex so data can be updated
				boost::mutex::scoped_lock lock(_mutexSensorValues);
				_records = _recordsTmp;
			}
		} else {
			break;
		}
	}

}

void Trakstar::stopPolling() {

	// Signal to _poll to stop polling
	_flagStopPoll = true;
	_pollThread.join();

	// Get active transmitter
	short buffer, *pBuffer = &buffer;
	_errorCode = GetSystemParameter(SELECT_TRANSMITTER, pBuffer,
			sizeof(buffer));
	if (_errorCode != BIRD_ERROR_SUCCESS) {
		errorHandler(_errorCode, __LINE__);
		return;
	}

	// Stop transmitter if one was turned on
	if (buffer != -1) {
		short id = -1;
		_errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
		if (_errorCode != BIRD_ERROR_SUCCESS)
			errorHandler(_errorCode, __LINE__);
	}
}

std::vector<Trakstar::PoseData> Trakstar::getData(void) {

	// Wait until we can get mutex
	boost::mutex::scoped_lock lock(_mutexSensorValues);
	// Mutex is scoped and is thus released automatically.
	/*cout << "status: ";
	 for (int i = 0; i < _records.size(); i++) {
	 cout << _records[i].status << ", ";
	 }
	 cout << endl;*/
	return _records;
}

std::string Trakstar::getSensorStatusString(int errorCode) {
	if (errorCode == VALID_STATUS) {
		return "OK";

	} else if (errorCode & NOT_ATTACHED) {
		return "NOT CONNECTED";

	} else if (errorCode & OUT_OF_MOTIONBOX) {
		return "RANGE";

	} else if (errorCode & SATURATED) {
		return "SATURATED";
	}

	// else
	stringstream ss;
	ss << "e" << errorCode;
	return ss.str();
}

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
void errorHandler(int error, int lineNum) {
	char buffer[1024];
	int currentError = error;
	int nextError;

	do {
		nextError = GetErrorText(currentError, buffer, sizeof(buffer),
				SIMPLE_MESSAGE);
		std::cout << buffer << endl;
		currentError = nextError;
	} while (currentError != BIRD_ERROR_SUCCESS);
}

timespec diff(timespec start, timespec end) {
	timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}
