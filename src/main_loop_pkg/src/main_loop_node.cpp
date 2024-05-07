/* -------------------------------------------------------------------------- */
/*                           HEADER FILES INCLUSION                           */
/* -------------------------------------------------------------------------- */
#include "main_loop_pkg/main_loop_class.h"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"main_loop_node");
    ros::NodeHandle node_handler;
    MainLoopClass main_loop_node(node_handler);
    ros::spin();
    return 0;
}
/* -------------------------------------------------------------------------- */