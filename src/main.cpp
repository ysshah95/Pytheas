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
 * @file main.cpp
 * @auther Yash Shah
 * @version 1.0
 * @brief main cpp file for the project repository
 * 
 * This file is the main cpp file for the repository. 
 * 
 * @copyright BSD 3-Clause License
 */

// Including c++ and ROS header files
#include <memory>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
// Including user defined header files
#include "turtlebot.hpp"
#include "cam.hpp"
#include "controlMotion.hpp"
#include "detectObject.hpp"

int main(int argc, char **argv) {
    // Initialize ROS and name our node "FinalProject"
    ros::init(argc, argv, "FinalProject");

    // Handle for the process node. Will handle initialization and
    //   cleanup of the node
    ros::NodeHandle n;

    // Set up the publisher rate to 10 Hz
    ros::Rate loop_rate(10);

    // Turtlebot container using smart pointer
    std::shared_ptr<Turtlebot> turtlebot{nullptr};
    turtlebot = std::make_shared<Turtlebot>();

    while (ros::ok()) {
        turtlebot->drive();

        // "Spin" a callback in case we set up any callbacks
        ros::spinOnce();

        // Sleep for the remaining time until we hit our 10 Hz rate
        loop_rate.sleep();
    }

    return 0;
}
