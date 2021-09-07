#include "main.h"

int main(int argc, char **argv){

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::init(argc, argv, "drone_simulator");
//    ros::NodeHandle n("~");
    ros::NodeHandle nh("~");
    Simulator sim(n);

    ros::AsyncSpinner spinner(7); // Use 5 threads -> 5 callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
