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
 * @file test_service_functionality.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief This file is used to test ROS Services created
 *  
 * @copyright BSD 3-Clause License
 */

// Including gtest header file
#include <gtest/gtest.h>

// Including ros header file
#include <pytheas/changeSpeedService.h>
#include <pytheas/togglePauseMotion.h>
#include <pytheas/changeThresholdService.h>
#include <pytheas/takeImageService.h>
#include "ros/ros.h"
#include "ros/service_client.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "turtlebot.hpp"
#include "controlMotion.hpp"
#include "cam.hpp"


/**
 * @brief Testing the changeThresholdService
 */
TEST(test_Services, change_threshold_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<pytheas::changeThresholdService>("changeThresholdService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  pytheas::changeThresholdService srv;
  srv.request.threshold = 1.5;
  client.call(srv);
  EXPECT_TRUE(srv.response.resp);
}

/**
 * @brief Testing the changeLinearSpeedService
 */
TEST(test_Services, change_speed_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<pytheas::changeSpeedService>("changeSpeedService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  pytheas::changeSpeedService srv;
  srv.request.speed = 1.0;
  client.call(srv);
  EXPECT_TRUE(srv.response.resp);
}

/**
 * @brief Testing the togglePauseMotion Service
 */
TEST(test_Services, control_motion_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<pytheas::togglePauseMotion>("togglePauseMotionService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  pytheas::togglePauseMotion srv;
  // Continue Motion
  srv.request.pause = false;
  client.call(srv);
  EXPECT_TRUE(srv.response.resp);
  // Stop Motion
  srv.request.pause = true;
  client.call(srv);
  EXPECT_TRUE(srv.response.resp);
}

/**
 * @brief Testing the takeImageService
 */
TEST(test_Services, take_image_service) {
  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<pytheas::takeImageService>("takeImageService");
  bool exists(client.waitForExistence(ros::Duration(2)));
  ASSERT_TRUE(exists);
  pytheas::takeImageService srv;
  srv.request.flag = true;
  client.call(srv);
  EXPECT_TRUE(srv.response.resp);
}

/**
 * @brief Main function for testing, runs all the tests that were declared
 *        with TEST()
 * @param argc  The argc as int
 * @param argv  The argv as char array
 * @return 0, if everything is successful
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "test_services");
  ::testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
