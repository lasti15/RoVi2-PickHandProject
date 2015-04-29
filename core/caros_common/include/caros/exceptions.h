#ifndef CAROS_COMMON_EXCEPTIONS_H
#define CAROS_COMMON_EXCEPTIONS_H

#include <stdexcept>
#include <string>
#include <sstream>

/**
 * \addtogroup Exceptions CAROS Exceptions
 * Use the THROW_CAROS_<...> macros to throw the corresponding exceptions.
 *
 * THROW_CAROS_UNAVAILABLE_SERVICE(...) and THROW_CAROS_BAD_SERVICE_CALL(...) are primarily for use within service interface proxies (SIP), and the corresponding exceptions are to be caught in the client code making use of the SIP.
 * @{
 */

/*
 * @brief Throw an unavailableService exception with the message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_UNAVAILABLE_SERVICE("The service " << serviceName << " is unavailable.");
 * \endcode
 */
#define THROW_CAROS_UNAVAILABLE_SERVICE(ostreamExpression) \
  do                                                       \
  {                                                        \
    std::ostringstream CAROS__message;                     \
    CAROS__message << ostreamExpression;                   \
    throw caros::unavailableService(CAROS__message.str()); \
  } while (0)

/*
 * @brief Throw a badServiceCall exception with the message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  THROW_CAROS_BAD_SERVICE_CALL("An unexpected error happened while calling the service " << serviceName);
 * \endcode
 */
#define THROW_CAROS_BAD_SERVICE_CALL(ostreamExpression) \
  do                                                    \
  {                                                     \
    std::ostringstream CAROS__message;                  \
    CAROS__message << ostreamExpression;                \
    throw caros::badServiceCall(CAROS__message.str());  \
  } while (0)

/**
 * @}
 */

namespace caros
{
/**
 * \addtogroup Exceptions
 * @{
 */

/**
 * @brief unavailable service exception.
 *
 * Used when the requested service is unavailable.
 */
class unavailableService : public std::runtime_error
{
 public:
  explicit unavailableService(const std::string& what) : runtime_error(what)
  {
    /* Empty */
  }

  virtual ~unavailableService() throw()
  {
    /* Empty */
  }
};

/**
 * @brief bad service call exception.
 *
 * Used when an (unknown) error occured while requesting a service.
 */
class badServiceCall : public std::runtime_error
{
 public:
  explicit badServiceCall(const std::string& what) : runtime_error(what)
  {
    /* Empty */
  }
  virtual ~badServiceCall() throw()
  {
    /* Empty */
  }
};

/**
 * @}
 */

}

#endif /* CAROS_COMMON_EXCEPTIONS_H */
