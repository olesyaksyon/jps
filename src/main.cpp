//
// Created by olesya on 07.01.19.
//

#include "jps_main.h"

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "jps_main.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "jps");

    jps_main unit;
    unit.process();

    ros::spin();
    return 0;
}