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
 * @file detectObjectTest.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief Unit tests for the DetectObject class
 * 
 * This file is used to run all unit tests for the DetectObject ROS node
 * 
 * @copyright BSD 3-Clause License
 */

// Including ROS, C++ and user defined header files
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "detectObject.hpp"

/**
 * @brief Test the ability for the obstacle detector to properly detect an obstacle
 */
TEST(ObjectDetectTestSuite, detect_obstacle) {
  std::shared_ptr<DetectObject> detectObject{nullptr};
  detectObject = std::make_shared<DetectObject>(1.0);

  // Create dummy laser scan to provide a fake collision:
  sensor_msgs::LaserScan collideMsg;
  collideMsg.angle_min = 0;
  collideMsg.angle_max = 0;
  collideMsg.angle_increment = 0;
  collideMsg.time_increment = 0;
  collideMsg.scan_time = 0;
  collideMsg.range_min = 0;
  collideMsg.range_max = 0;
  collideMsg.ranges.push_back(0);
  collideMsg.intensities.push_back(0);

  EXPECT_TRUE(detectObject->detectObstacle(collideMsg));

  // Create dummy laser scan to provide a fake non-collision:
  sensor_msgs::LaserScan freeMsg;
  freeMsg.angle_min = 0;
  freeMsg.angle_max = 0;
  freeMsg.angle_increment = 0;
  freeMsg.time_increment = 0;
  freeMsg.scan_time = 0;
  freeMsg.range_min = 0;
  freeMsg.range_max = 0;
  freeMsg.ranges.push_back(2.0);
  freeMsg.intensities.push_back(0);

  EXPECT_FALSE(detectObject->detectObstacle(freeMsg));
}

/**
 * @brief Test the ability to set a new threshold for the obstacle detector
 */
TEST(ObjectDetectTestSuite, set_threshold) {
  std::shared_ptr<DetectObject> detectObject{nullptr};
  detectObject = std::make_shared<DetectObject>(1.0);
  detectObject->setDistanceThreshold(2.0);

  EXPECT_EQ(2.0, detectObject->getDistanceThreshold());
}

/**
 * @brief Test the ability to get the current threshold for the obstacle detector
 */
TEST(ObjectDetectTestSuite, get_threshold) {
  std::shared_ptr<DetectObject> detectObject{nullptr};
  detectObject = std::make_shared<DetectObject>(1.0);

  EXPECT_EQ(1.0, detectObject->getDistanceThreshold());
}
