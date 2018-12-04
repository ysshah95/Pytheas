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

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "detectObject.hpp"


/**
 * @brief controlMotion class handles determining turtlebot control actions
 */
class ControlMotion {
 public:
  /**
   * @brief ControlMotion constructor
   */
  ControlMotion(double forwardSpeed);

   /**
   * @brief Determine a turtlebot action based on results from the obstacle detector
   */
  geometry_msgs::Twist determineAction();

   /**
   * @brief change the forward speed of the robot
   */
  void setForwardSpeed(double speed) {
    forwardSpeed = speed;
  };
   /**
   * @brief return the current forward speed of the robot
   */
  double getForwardSpeed() {
    return forwardSpeed;
  };

 private:
  /**
   * @brief container for an obstacle detector for the turtlebot
   */
  DetectObject *detectObject;

   /**
   * @brief container for the forward speed of the turtlebot
   */
  double forwardSpeed;
};