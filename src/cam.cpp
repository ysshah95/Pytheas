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
 * @file cam.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief Camera class implementation
 * 
 * Implementation of the ROS Camera support methods.
 * 
 * @copyright BSD 3-Clause License
 */

// including C++, ROS and service Header files
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <vector>
#include <string>

// including the headers for OpenCV's image processing and GUI modules
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// including user-defined header file
#include "cam.hpp"

Cam::Cam() :
  takeImageFlag(false) {
  // Register client to "takeImage" service
  cameraClient = nh.serviceClient < pytheas::takeImageService >
    ("takeImageService");
}

void Cam::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (takeImageFlag) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Save OpenCV Image to file:
    cv::namedWindow("Image window");
    cv::imshow("Image window", cv_ptr->image);
    int count = savedImages.size() + 1;
    std::ostringstream filename;
    filename << "turtleBotImage_" << count << ".jpg";
    cv::imwrite(filename.str(), cv_ptr->image);
    ROS_INFO("Saving image %s to ~/.ros/", filename.str().c_str());
    // Add filename to list of saved images:
    savedImages.push_back(filename.str());
    // Reset Flag:
    takeImageFlag = false;
  }
}

bool Cam::takeImage(pytheas::takeImageService::Request &req,
  pytheas::takeImageService::Response &resp) {
  resp.resp = true;
  ROS_INFO_STREAM("Set flag to [true], save the next available image frame.");
  takeImageFlag = resp.resp;
  return true;
}

std::vector<std::string> Cam::getSavedImageFilenames() {
    return savedImages;
}
