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
 * @file controlMotion.hpp
 * @auther Yash Shah
 * @version 1.0
 * @brief ControlMotion class definition
 * 
 * Definition of the ROS controlMotion support methods.
 * 
 * @copyright BSD 3-Clause License
 */

#ifndef INCLUDE_CONTROLMOTION_HPP_
#define INCLUDE_CONTROLMOTION_HPP_

// Including C++ and ROS header files
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
// including user-defined Header file and service files
#include <pytheas/changeSpeedService.h>
#include <pytheas/togglePauseMotion.h>
#include <pytheas/changeThresholdService.h>
// Include user defined header files
#include "detectObject.hpp"
#include "controlMotion.hpp"

/**
 * @brief controlMotion class handles determining turtlebot control actions
 */
class ControlMotion {
 public:
  /**
   * @brief ControlMotion constructor
   * @param forwardspeed of type double
   * @return none
   */
  explicit ControlMotion(double forwardSpeed);

   /**
   * @brief Determine a turtlebot action based on results from 
   *        the obstacle detector
   * @param Reference to a variable of type sensor_msgs::LaserScan
   * @return void
   */
  void determineAction(const sensor_msgs::LaserScan::ConstPtr& msg);

   /**
   * @brief change the forward speed of the robot
   * @param speed of type double
   * @return none
   */
  void setForwardSpeed(double speed);

   /**
   * @brief return the current forward speed of the robot
   * @param none
   * @return robot's linear speed data of type double
   */
  double getForwardSpeed();

	/**
	 * @brief return the current vehicle action
   * @param none
   * @return robot's velocity data of type geometry_msgs::Twis
	 */
  geometry_msgs::Twist getVehicleAction();

  /**
   * @brief Response to the change speed service to set forward speed
   * @param Reference to a request variable of type defined in 
   *        changeSpeedService.srv file
   * @param Reference to a response variable of type defined in 
   *        changeSpeedService.srv file
   * @return a boolean value of success or failure
   */
  bool changeSpeed(pytheas::changeSpeedService::Request &req,
                   pytheas::changeSpeedService::Response &resp);

  /**
   * @brief Response to the change threshold service to set distance threshold
   * @param Reference to a request variable of type defined in 
   *        changeThresholdService.srv file
   * @param Reference to a response variable of type defined in 
   *        changeThresholdService.srv file
   * @return a boolean value of success or failure
   */
  bool changeThreshold(pytheas::changeThresholdService::Request &req,
                       pytheas::changeThresholdService::Response &resp);

  /**
   * @brief Response to the toggle pause motion service
   * @param Reference to a request variable of type defined in 
   *        togglePauseMotion.srv file
   * @param a reference to a response variable of type defined in 
   *        togglePauseMotion.srv file
   * @return a boolean value of success or failure
   */
  bool togglePause(pytheas::togglePauseMotion::Request &req,
                   pytheas::togglePauseMotion::Response &resp);


 private:
  /**
   * @brief container for an obstacle detector for the turtlebot
   */
  std::shared_ptr<DetectObject> detectObject{nullptr};

   /**
   * @brief container for the forward speed of the turtlebot
   */
  double forwardSpeed;

  /**
	 * @brief Container for Twist message to be sent the vehicle on next "drive" command
	 */
  geometry_msgs::Twist vehicleAction;

  /**
   * @brief Flag to denote if we should pause the robot or continue motion
   */
  bool pauseMotion;

  /**
   * @brief Flag to denote that we have entered a "collision" state and are turning
   */
  bool obstaclePresent;

  /**
   * @brief Counter for how long we have been spinning trying to find a free space
   */
  int obstacleCounter;
};

#endif  // INCLUDE_CONTROLMOTION_HPP_
