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
 * @file cam.hpp
 * @auther Yash Shah
 * @version 1.0
 * @brief Camera class definition
 * 
 * Definition of the ROS Camera support methods.
 * 
 * @copyright BSD 3-Clause License
 */

#ifndef INCLUDE_CAM_HPP_
#define INCLUDE_CAM_HPP_

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pytheas/takeImageService.h>

#include <string>
#include <vector>




/**
 * @brief Cam class handles viewing onboard imagery and taking images
 */

class Cam {
 public:
  /**
   * @brief Cam constructor
   */
  Cam();

  /**
   * @brief Take an image of the current RGB camera view for later analysis
   */
  bool takeImage(pytheas::takeImageService::Request &req,
    pytheas::takeImageService::Response &resp);

  /**
	 * @brief Camera topic callback takes a picture if flag has been set
	 */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

  std::vector<std::string> getSavedImageFilenames();

 private:
  /**
   * @brief container for the filenames of each saved image
   */
  std::vector<std::string> savedImages;

  /**
   * @brief Flag denoting whether or not to take an image on next receipt of camera topic
   */
  bool takeImageFlag;

  /**
   * @brief Container for takeImage service
   */
  ros::ServiceClient cameraClient;

  /**
   * @brief Node handler for subscribing to service and topics
   */
  ros::NodeHandle nh;
};

#endif  // INCLUDE_CAM_HPP_
