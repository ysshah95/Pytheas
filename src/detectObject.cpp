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
 * @file detectObject.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief DetectObject class implementation.
 * 
 * Implementation of the ROS DetectObject support methods. 
 * 
 * @copyright BSD 3-Clause License
 */

// Include c++, ROS and service header files
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pytheas/changeThresholdService.h>
#include "detectObject.hpp"

DetectObject::DetectObject(double threshold)
    : distanceThreshold(threshold) {
}

bool DetectObject::detectObstacle(
        const sensor_msgs::LaserScan msg) {
    // Check if any scan from the laser is less than 'distancethreshold' meters
    //  from the front of the robot. If so, a collision is about to occur
    for (auto i : msg.ranges) {
        if (i < distanceThreshold) {
            return true;
        }
    }
    // Return false if we are collision free
    return false;
}

double DetectObject::getDistanceThreshold() {
     return distanceThreshold;
}

void DetectObject::setDistanceThreshold(double threshold) {
     distanceThreshold = threshold;
}
