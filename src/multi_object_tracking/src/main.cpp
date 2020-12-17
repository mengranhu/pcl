#include "../include/bbox.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bbox_node");

    ros::NodeHandle nh;

    Bbox core(nh);
    return 0;
}
