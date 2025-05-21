#include "nour_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "nour_base_node");
    NourBase nour;
    ros::spin();
    return 0;
}
