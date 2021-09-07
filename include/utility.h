#ifndef UTILITY_H
#define UTILITY_H

#include <chrono>
#include <math.h>

#include <ros/ros.h>
//generate random pose
#include <random>
#include <iostream>

using namespace std::chrono; 
using namespace std;

/////////// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

high_resolution_clock::time_point start; //global, to use tic()
void tic(){
   start = high_resolution_clock::now();
}
void toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%.3f ms spent", duration.count()/1000.0);
}
void toc(string text){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%s %.3f ms spent", text.c_str(), duration.count()/1000.0);
}

double vel_saturation(double vel, double sat){
	if (vel>sat)
	    vel = sat;
	if (vel<-sat)
	    vel = -sat;
	return vel;
}

float Random(float min, float max) {
  random_device rn;
  default_random_engine rnd{ rn() };
  uniform_real_distribution<float> range(min, max);
  return range( rnd );
}

double euclidean_dist(double x1, double y1, double z1, double x2, double y2, double z2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2));
}


#endif
