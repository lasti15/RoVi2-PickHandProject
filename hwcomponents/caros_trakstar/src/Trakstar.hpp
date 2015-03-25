#ifndef CAROS_TRAKSTAR_HPP_
#define CAROS_TRAKSTAR_HPP_

#include <ros/ros.h>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <boost/thread/thread.hpp>

#include <vector>

namespace caros {

/**
 * @brief this class serves as a cpp wrapper for the trakstar driver interface
 */
class Trakstar
{
public:
  //! init status defines
  typedef enum{ TRAKSTAR_STATUS_STOPPED=-1 //! error occurred and initialization stopped
    ,TRAKSTAR_STATUS_INITIALIZING=0 //! still initializing
    , TRAKSTAR_STATUS_STARTED=1 //!
  } TrakstarStatus;
  //! forward declaration
  struct PoseData;
public:
  //! constructor
  Trakstar();
  //! destructor
  virtual ~Trakstar();

  /**
   * @brief initialize the trakstar sensor. This may take several 10's of seconds. Hence
   * an option for performing the call non blocking is added. Use is initialized to
   * check if initialization is done. And getInitStatus in order to know if
   * something blocked the initialization.
   */
  void initialize(bool block = true);

  //! check if driver is initialized
  bool isInitialized();

  //! get status of initialization
  TrakstarStatus getInitStatus();

  //! get data of all pose sensors
  std::vector<PoseData> getData(void);

  /**
   * @brief start the transmitter and start polling the poses of all
   * active sensors.
   * @return true if polling started successfully, else false.
   */
  bool startPolling();
  void stopPolling();
  bool isPolling()
  {
    return !_flagStopPoll;
  }

  int getNumberSensorsAttached();
  int getNumberSensorsSupported();

  std::string getSensorStatusString(int errorcode);

public:
  struct PoseData
  {
    rw::math::Vector3D<> pos;
    rw::math::Quaternion<> rot;
    double time;
    double quality; // quality from 0 to 1
    bool valid;
    unsigned long status;
    bool calibStatus;
    bool analogButtonOn;
  };

private:
  struct tagSYSTEM_CONFIGURATION *ATC3DG_;   // a pointer to a single instance of the system class

  std::vector<rw::common::Ptr<struct tagSENSOR_CONFIGURATION> > sensors_;  // a pointer to an array of sensor objects
  std::vector<rw::common::Ptr<struct tagTRANSMITTER_CONFIGURATION> > transmitters_;    // a pointer to an array of transmitter objects
  //std::vector<struct tagTRAKSTAR_RECORDS_TYPE*> _rawValues;
  std::vector<char> raw_values_;

  std::vector<PoseData> _records, _recordsTmp;

  int _errorCode;                     // used to hold error code returned from procedure call

  int _sensorsAttached;       // Is updated with the actual number of connected sensors at startPolling()

  boost::thread _initThread;

  bool _flagStopPoll;
  bool _analogButtonOn;

  TrakstarStatus init_status_;

  int initializeSystem();
  void pollData();
};

}

#endif /* TRAKSTAR_HPP_ */
