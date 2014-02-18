#ifndef RWROS_H_
#define CAROS_H_

#include <rw/math.hpp>
#include <caros_msgs/Q.h>

namespace caros {
  rw::math::Q fromRos(const caros_msgs::Q& q);
  caros_msgs::Q toRos(const rw::math::Q& q);
}

#endif /* CAROS_H_ */
