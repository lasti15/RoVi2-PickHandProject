#include <caros_msgs/caros.hpp>

/* Verify that the caros_msgs/Q type has not changed using the MD5 value or similar */

namespace caros {
  rw::math::Q fromRos(const caros_msgs::Q& q) {
    rw::math::Q res(q.data.size());
    for (size_t i = 0; i < q.data.size(); ++i) {
      res(i) = q.data[i];
    }
    return res;
  }

  caros_msgs::Q toRos(const rw::math::Q& q) {
    caros_msgs::Q res;
    res.data.resize(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
      res.data[i] = static_cast<double>(q(i));
    }
    return res;
  }
}
