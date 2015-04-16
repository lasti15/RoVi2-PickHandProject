#ifndef CAROS_COMMON_CAROS_SERVICE_CLIENT_H
#define CAROS_COMMON_CAROS_SERVICE_CLIENT_H

#include <caros/exceptions.h>

#include <ros/ros.h>

#include <string>

/* Fully defined in the header file as the major functions are templated */

namespace caros
{
/**
 * @brief this class implements a wrapper around the ros::ServiceClient class/object and provide builtin reconnection
 *functionality for persistent connections.
 *
 * Connections are not created before they are actually being used (e.g. one of the call functions)
 */
class carosServiceClient
{
 public:
  /**
   * @breif Constructor
   * @param[in] nodehandle used to obtain/create ros::ServiceClient object (could be converted into a version that
   * doesn't require a nodehandle)
   * @param[in] connectionIdentifier specifying the service name to connect to (excluding the namespace)
   * @param[in] serviceNamespace specifying the namespace of where the service is to be found
   * @param[in] usePersistentConnection specify the persistence of the connections (this can be changed at run time)
   */
  carosServiceClient(ros::NodeHandle nodehandle, const std::string& connectionIdentifier,
                     const std::string& serviceNamespace, const bool usePersistentConnection)
      : nodehandle_(nodehandle),
        connectionIdentifier_(connectionIdentifier),
        usePersistentConnection_(usePersistentConnection)
  {
    serviceName_ = serviceNamespace + "/" + connectionIdentifier_;
  }

  /**
   * @brief Destructor
   */
  virtual ~carosServiceClient()
  { /* Nothing for now */
  }

  /**
   * @brief call the service
   * @param[in] srv the service communication type
   * @param[in] persistence use persistent (i.e. true) or non-persistent (i.e. false) connection
   */
  template <typename T>
  bool call(T& srv, const bool persistence)
  {
    usePersistentConnection(persistence);
    return call<T>(srv);
  }

  /**
   * @brief call the service
   * @param[in] srv the service communication type
   */
  template <typename T>
  bool call(T& srv)
  {
    prepareConnection<T>();

    bool srvCallSuccess = false;

    if (not serviceClient_.exists())
    {
      THROW_CAROS_UNAVAILABLE_SERVICE("The service " << serviceClient_.getService() << " does not exist.");
    }

    srvCallSuccess = serviceClient_.call(srv);
    if (not srvCallSuccess)
    {
      THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << serviceClient_.getService());
    }

    return srvCallSuccess;
  }

  /**
   * @brief make sure the connection is shutdown if necessary (i.e. a persistent connection that is still valid)
   */
  void shutdown()
  {
    if (serviceClient_.isPersistent() && serviceClient_.isValid())
    {
      ROS_DEBUG_STREAM("Shutting down persistent connection (id: " << connectionIdentifier_ << ")");
      serviceClient_.shutdown();
    }
  }

  /**
   * @brief change persistence of the connection(s) to be established
   * @param[in] persistence use persistent (i.e. true) or non-persistent (i.e. false) connection
   */
  void usePersistentConnection(const bool persistence)
  {
    usePersistentConnection_ = persistence;
  }

  /**
   * @brief is it currently set to use persistent or non-persistent connection(s)
   * @returns a boolean indicating the persistence (i.e. true for persistent connections and false for non-persistent
   * connections)
   */
  bool isUsingPersistentConnection()
  {
    return usePersistentConnection_;
  }

 protected:
  /**
   * @brief Make sure to setup an appropriate connection or change it based on changed persistence preferences
   */
  template <typename T>
  void prepareConnection()
  {
    bool createNewConnection = false;

    /* The ordering is important to make sure that the switch between persistent and non-persistent connection is
     * happening */
    /* Switch from persistent to non-persistent connection */
    if (serviceClient_.isPersistent() && not usePersistentConnection_)
    {
      ROS_DEBUG_STREAM("Switching from persistent to non-persistent connection (id: " << connectionIdentifier_ << ")");
      if (serviceClient_.isValid())
      {
        ROS_DEBUG_STREAM("Shutting down the old persistent connection (id: " << connectionIdentifier_ << ")");
        serviceClient_.shutdown();
      }
      createNewConnection = true;
    }
    /* Switch from non-persistent to persistent connection */
    /* the .isValid() test is to make sure that this case is not wrongly chosen when a new persistent connection should
     * be created */
    else if (not serviceClient_.isPersistent() && serviceClient_.isValid() && usePersistentConnection_)
    {
      ROS_DEBUG_STREAM("Switching from non-persistent to persistent connection (id: " << connectionIdentifier_ << ")");
      createNewConnection = true;
    }
    /* Reestablish the persistent connection */
    else if (serviceClient_.isPersistent() && not serviceClient_.isValid())
    {
      ROS_DEBUG_STREAM("Reconnecting the persistent connection with (id: " << connectionIdentifier_ << ")");
      createNewConnection = true;
    }
    /* No previous connection setup */
    else if (not serviceClient_.isValid())
    {
      ROS_DEBUG_STREAM("Setting up new connection (id: " << connectionIdentifier_ << ")");
      createNewConnection = true;
    }

    if (createNewConnection)
    {
      /* Can use the ros::service::createClient call and not have to rely on having a nodehandle */
      // serviceClient_ = ros::service::createClient<T>(serviceName_, usePersistentConnection_);
      serviceClient_ = nodehandle_.serviceClient<T>(serviceName_, usePersistentConnection_);
    }
  }

 protected:
  ros::NodeHandle nodehandle_;
  ros::ServiceClient serviceClient_;
  std::string connectionIdentifier_;
  std::string serviceName_;
  bool usePersistentConnection_;
};

}  // end namespace

#endif  // include guard
