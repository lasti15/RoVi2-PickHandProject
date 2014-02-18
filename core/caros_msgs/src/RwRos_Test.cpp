#include <caros_msgs/RwRos.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
  rw::math::Q rhs(3);
  rhs(0) = 1.2;
  rhs(1) = 1.3;
  rhs(2) = 1.4;
  rw::math::Q lhs(3);
  lhs(0) = 1.5;
  lhs(1) = 1.6;
  lhs(2) = 1.7;

  std::cout << "Dot: " << rwros::RwDot(rhs, lhs) << std::endl;

  return 0;
}
