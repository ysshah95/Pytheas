/**
 * BSD 3-Clause License

 * Copyright (c) 2018, Yash Shah
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/**
 * @file test_nodes_functionality.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief This file is used to test nodes created in source code
 *  
 * @copyright BSD 3-Clause License
 */

// Including header files
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "turtlebot.hpp"
#include "controlMotion.hpp"
#include "cam.hpp"
#include "testSetUp.hpp"



/**
 * @brief To test determine action function of MotionController class in a situation when there is no obstacle
 * @param none
 * @return none
 */
TEST(CallbackFunctionTesting, doesNotCollideTest) {
  ros::NodeHandle nh;
  TestSetUp test;
  ros::Rate loop_rate(2);
  // register to check number of publishers to /mobile_base/commands/velocity
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000, &TestSetUp::velocityCallback, &test);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();
  ASSERT_EQ(1, velocity.getNumPublishers());
  ASSERT_EQ(1, laserPub.getNumSubscribers());

  // create dummy Twist velocity message which will be used for comparation
  geometry_msgs::Twist msg;
  msg.linear.x = 0.25;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  // Prepare the data
  // create dummy laser scan msg to provide a fake non-obstacle collision
  size_t num_readings = 50;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_max;
  }
  
  laserPub.publish(scan);
  loop_rate.sleep();
  ros::spinOnce();
  // confirm velocity is expected
  EXPECT_EQ(0, std::memcmp(&msg, &test.twist, sizeof(msg))) << test.twist.linear.x;
}

/**
 * @brief To test determine action function of MotionController class in a situation when there is an obstacle
 * @param none
 * @return none
 */
TEST(TestingCallbacks, collisionTest) {
  ros::NodeHandle nh;
  TestSetUp test;
  ros::Rate loop_rate(2);
  // register to check number of publishers to /mobile_base/commands/velocity
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000, &TestSetUp::velocityCallback, &test);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();
  ASSERT_EQ(1, velocity.getNumPublishers());
  ASSERT_EQ(1, laserPub.getNumSubscribers());

  // create dummy Twist velocity message which will be used for comparation
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 1.0;

  // Prepare the data
  // create dummy laser scan msg to provide a fake non-obstacle collision
  size_t num_readings = 50;
  sensor_msgs::LaserScan scan;
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / 40) / (num_readings);
  scan.range_min = 0.5;
  scan.range_max = 0.0;
  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for (auto& i : scan.ranges) {
    i = scan.range_max;
  }
  
  laserPub.publish(scan);
  loop_rate.sleep();
  ros::spinOnce();
  // confirm velocity is expected
  EXPECT_EQ(0, std::memcmp(&msg, &test.twist, sizeof(msg))) << test.twist.angular.z;
}

/**
 * @brief To test whether the velocity publisher is initialize properly or not
 * @param none
 * @return none
 */
TEST(CallbackFunctionTesting, velocity_callback) {
  ros::NodeHandle nh;
  TestSetUp test;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /scan
  ros::Subscriber velocity = nh.subscribe("/mobile_base/commands/velocity", 1000,
    &TestSetUp::velocityCallback, &test);
  loop_rate.sleep();  
  EXPECT_EQ(1, velocity.getNumPublishers());
}

/**
 * @brief To test whether the camera subscriber is initialize properly or not
 * @param none
 * @return none
 */
TEST(CallbackFunctionTesting, camClass_callback) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /camera/rgb/image_raw
  ros::Publisher cameraPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",100);
  loop_rate.sleep();
  EXPECT_EQ(1, cameraPub.getNumSubscribers());
}

/**
 * @brief To test whether the laser subscriber is initialize properly or not
 * @param none
 * @return none
 */
TEST(CallbackFunctionTesting, laserData_callback) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /scan
  ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/scan",100);
  loop_rate.sleep();  
  EXPECT_EQ(1, laserPub.getNumSubscribers());
}


/**
 * @brief To test whether the camera callback function
 * @param none
 * @return none
 */
TEST(TestingCallbacks, camera_callback) {
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  // register to check number of Subscribers to /camera/rgb/image_raw
  ros::Publisher cameraPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",100);
  loop_rate.sleep();
  EXPECT_EQ(1, cameraPub.getNumSubscribers());
}

/**
 * @brief Main function for testing, runs all the tests that were declared
 *        with TEST()
 * @param argc  The argc as int
 * @param argv  The argv as char array
 * @return 0, if everything is successful
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "test_nodes_functionality");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();;
} 