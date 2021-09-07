#include "main.h"


int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "traj_generator_and_visualizor");
    ros::NodeHandle n("~");
    TrajectoryGeneratorWaypoint traj(n);

    ros::AsyncSpinner spinner(7); // Use 5 threads -> 5 callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
