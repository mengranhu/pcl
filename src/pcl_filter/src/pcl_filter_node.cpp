//
// Created by ryze on 20-7-1.
//

#include "../include/pcl_filter_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_filter");

    ros::NodeHandle nh;

    PclFilterCore core(nh);
    return 0;
}
