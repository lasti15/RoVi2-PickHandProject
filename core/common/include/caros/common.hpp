#ifndef CAROS_COMMON_H_
#define CAROS_COMMON_H_

#include <rw/math.hpp>
#include <caros_common/Q.h>

/**
 * \brief This is documentation for the namespace caros
 */
namespace caros {
  /**
   * \brief Convert the ROS message caros::Q to the RobWork type rw::math::Q
   * \param[in] q
   */
  rw::math::Q fromRos(const caros::Q& q) {
    rw::math::Q res(q.data.size());
    for (size_t i = 0; i < q.data.size(); ++i) {
      res(i) = q.data[i];
    }
    return res;
  }

  caros::Q toRos(const rw::math::Q& q) {
    caros::Q res;
    res.data.resize(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
      res.data[i] = static_cast<double>(q(i));
    }
    return res;
  }
}

#endif /* CAROS_COMMON_H_ */
