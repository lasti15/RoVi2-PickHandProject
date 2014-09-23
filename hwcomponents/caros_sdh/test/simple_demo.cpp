#include <caros/GripperSIProxy.hpp>
#include <rw/math/Q.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(SimpleDemo, moveFingers) {
    ros::NodeHandle nh;
    caros::GripperSIProxy gsip(nh, "sdh_test_node");

    rw::math::Q q(7, 0.1);

    EXPECT_TRUE(gsip.moveQ(q));
    /* As of this writing, moveQ(q) is only signalling that the action is to be performed (i.e. not a blocking function) */
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sdh_simpledemo");

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
