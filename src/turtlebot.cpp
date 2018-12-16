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
 * @file turtlebot.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief Turtlebot class implementation.
 * 
 * Implementation of the ROS Turtlebot support methods. 
 * 
 * @copyright BSD 3-Clause License
 */

// Including C++, ROS and user defined header files
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pytheas/takeImageService.h>
#include "turtlebot.hpp"

Turtlebot::Turtlebot()
        : publishedMessagesCount(0) {
    controlMotion = std::make_shared<ControlMotion>(0.25);
    cam = std::make_shared<Cam>();

    // Set up subscribers
    cameraSub = nh.subscribe < sensor_msgs::Image
            > ("/camera/rgb/image_raw", 500, &Cam::cameraCallback, cam.get());

    laserSub = nh.subscribe < sensor_msgs::LaserScan
            > ("/scan", 500, &ControlMotion::determineAction,
            controlMotion.get());

    // Register services with the master
    takeImageServer = nh.advertiseService("takeImageService", &Cam::takeImage,
                                            cam.get());

    changeThresholdServer = nh.advertiseService(
        "changeThresholdService", &ControlMotion::changeThreshold,
                                            controlMotion.get());

    changeSpeedServer = nh.advertiseService("changeSpeedService",
                                            &ControlMotion::changeSpeed,
                                            controlMotion.get());

    togglePauseServer = nh.advertiseService("togglePauseMotionService",
                                            &ControlMotion::togglePause,
                                            controlMotion.get());

    // Set up publisher:
    drivePub = nh.advertise < geometry_msgs::Twist
            > ("/mobile_base/commands/velocity", 1000);
}

void Turtlebot::drive() {
    // Grab current vehicle action:
    geometry_msgs::Twist vehicleCommand = controlMotion->getVehicleAction();
    // Publish command:
    drivePub.publish(vehicleCommand);
    // Increment published message counter:
    publishedMessagesCount++;
}

int Turtlebot::getPublishedMessagesCount() {
    return publishedMessagesCount;
}
