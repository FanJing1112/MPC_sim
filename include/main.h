#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include "utility.h"

//// C++ headers
#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>


//// PCL
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//// CV
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>

//// Linear Algebra
#include <Eigen/Eigen>
#include <OsqpEigen/OsqpEigen.h>

//// Conversions
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

//// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//For rqt_plot
#include <nmpc_gen/Info.h>

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace OsqpEigen;


//// main class
class Simulator{
  public:
    ros::NodeHandle nh;
    ros::Subscriber detect_sub;
    ros::Subscriber odom_sub;
    ros::Publisher local_vel_pub;
    ros::Timer thread;
    ros::Publisher drone_traj_pub;
    ros::Publisher target_traj_pub;
    ros::Publisher mpc_check_pub;
    ros::Publisher waypts_pub;
    ros::Publisher head_pub;
    ros::Publisher Info_pub;

    Matrix4f body_T_world=Matrix4f::Identity();

    bool detect_check=false, depth_check=false, drone_pose_check_first=false, drone_pose_check=false, debug=false, deep_debug=false, traj_gen_check=false;
    double traj_gen_hz, queuing_time, real_width, temp_depth, min_dist, max_dist, corridor_r, v_max, a_max, min_prob, time_allocation, predict_time=0.0;
    int test, count = 0, C_coeff1, point_numbers = 0, n_order = 0, N=0, min_size = 0, close_i=0, count2 = 0, last=0, last_t = 0, count3=0, sim_type=1;
    double curr_vel_x, curr_vel_y, curr_vel_z, curr_x, curr_y, curr_z, curr_roll, curr_pitch, curr_yaw=0;
    double k_yaw, rand_coeff, C_coeff2;
    double goal_x, goal_y, goal_z, goal_vx, goal_vy, goal_vz=0;
    const double PI = 3.1415926;
    std::string depth_topic, bbox_topic, body_base, fixed_frame;

    geometry_msgs::PoseStamped drone_pose;
    nav_msgs::Odometry odom;

    geometry_msgs::Twist velocity;
    nmpc_gen::Info info;
    vector<pcl::PointXYZ> queued_pts, way_pts;
    vector<double> xx;
    vector<double> yy;
    vector<double> zz;
    vector<MatrixXd> queued_M1, queued_M2, queued_M3, accumulated_M1, accumulated_M2, accumulated_M3, accumulated_waypoint;
    vector<VectorXd> queued_ts, accumulated_ts, way_pos, way_vel, drone_pos;
    VectorXd ts, M1, M2, M3;
    Vector3f world_detected_point = Vector3f::Zero(3);

    //Start/end constraint
    MatrixXd Vel = MatrixXd::Zero(3,2);
    MatrixXd Acc = MatrixXd::Zero(3,2);
    MatrixXd Jerk = MatrixXd::Zero(3,2);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void traj_gen_timer_func(const ros::TimerEvent& event);
    void drone_controller(const ros::TimerEvent& event);
    void dummy_controller();

    //Trajectory generator
    MatrixXd PolyQPGeneration(const Eigen::MatrixXd &waypts, const Eigen::VectorXd &Time, const int n_order,
                                     const Eigen::MatrixXd &Vel, const Eigen::MatrixXd &Acc, const Eigen::MatrixXd &Jerk, const double Corridor_r);
    MatrixXd MPC(const Eigen::MatrixXd &x_start, const Eigen::MatrixXd &ref_traj, const Eigen::MatrixXd &u_start, const double dt,const int N);

    int Factorial(int x);
    VectorXd arrangeT(const Eigen::MatrixXd &waypts);
    VectorXd calc_tvec(const double t, const int n_order, const int r);
    MatrixXd getQ(const int n, const int r, const double t1, const double t2);
    // class constructor
    Simulator(ros::NodeHandle& n) : nh(n){
        ///// params
        nh.param("/debug", debug, true);
        nh.param("/deep_debug", deep_debug, false);
        nh.param("/queuing_time", queuing_time, 1.0);
        nh.param("/point_numbers", point_numbers, 10);
        nh.param("/traj_gen_hz", traj_gen_hz, 3.333);
        nh.param("/min_dist", min_dist, 0.5);
        nh.param("/C_coeff1", C_coeff1, 1);
        nh.param("/C_coeff2", C_coeff2, 0.1);
        nh.param("/max_dist", max_dist, 2.5);
        nh.param("/corridor_r", corridor_r, 0.5);
        nh.param("/time_allocation", time_allocation, 1.5);
        nh.param("/v_max", v_max, 5.0);
        nh.param("/sim_type", sim_type, 1);
        nh.param("/a_max", a_max, 10.0);
        nh.param("/n_order", n_order, 7);
        nh.param("/min_size", min_size, 2);
        nh.param("/predict_time", predict_time, 1.0);
        nh.param("/rand_coeff", rand_coeff, 1.0);

        nh.param("/k_yaw", k_yaw, 0.5);


        nh.param<std::string>("/fixed_frame", fixed_frame, "/map");

        odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, &Simulator::odom_callback, this);
        drone_traj_pub = nh.advertise<nav_msgs::Path>("/drone_traj", 10);
        target_traj_pub = nh.advertise<nav_msgs::Path>("/target_traj", 10);
        mpc_check_pub = nh.advertise<nav_msgs::Path>("/mpc_predict", 10);
        waypts_pub = nh.advertise<visualization_msgs::MarkerArray>("/waypts", 10);
        local_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        head_pub = nh.advertise<visualization_msgs::Marker>( "/heading", 0 );
        Info_pub = nh.advertise<nmpc_gen::Info>( "/Info", 10 );
        thread = nh.createTimer(ros::Duration(1/traj_gen_hz), &Simulator::drone_controller, this);

        ROS_WARN("class heritated, starting node...");
        tic();
    }
};

void Simulator::odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    odom = *msg;
    curr_x = odom.pose.pose.position.x;
    curr_y = odom.pose.pose.position.y;
    curr_z = odom.pose.pose.position.z;    
    double temp_vx = odom.twist.twist.linear.x; //vel x, y are in local coordinates....
    double temp_vy = odom.twist.twist.linear.y;
    curr_vel_z = odom.twist.twist.linear.z;
    tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(curr_roll, curr_pitch, curr_yaw);
    curr_vel_x = temp_vx*cos(curr_yaw) - temp_vy*sin(curr_yaw);
    curr_vel_y = temp_vx*sin(curr_yaw) + temp_vy*cos(curr_yaw);
    //visualize drone traj
    VectorXd pos = VectorXd::Zero(3);
    pos(0)=curr_x; pos(1)=curr_y; pos(2)=curr_z;
    drone_pos.push_back(pos);
//    if (int(drone_pos.size())>100){
//        drone_pos.erase(drone_pos.begin());
//    }
    nav_msgs::Path drone_traj;
    drone_traj.header.stamp = ros::Time::now();
    drone_traj.header.frame_id = fixed_frame;
    for(int i=0; i<int(drone_pos.size()); i++){
        geometry_msgs::PoseStamped temp;
        temp.header.stamp = ros::Time::now();
        temp.header.frame_id = fixed_frame;
        temp.pose.position.x = drone_pos[i](0); temp.pose.position.y = drone_pos[i](1); temp.pose.position.z = drone_pos[i](2);
        drone_traj.poses.push_back(temp);
    }
    drone_traj_pub.publish(drone_traj);

    //visualize UAV
    visualization_msgs::Marker heading_vector;
    heading_vector.header.stamp = ros::Time::now();
    heading_vector.header.frame_id = fixed_frame;
    heading_vector.id = 0;
    heading_vector.pose.orientation.w = 1.0;
    heading_vector.color.g = 1.0f;
    heading_vector.color.a = 1.0;
    heading_vector.scale.x = 0.1;
    heading_vector.scale.y = 0.2;
    heading_vector.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point start_p, end_p;
    start_p.x = curr_x;
    start_p.y = curr_y;
    start_p.z = curr_z;
    heading_vector.points.push_back(start_p);
    double l = 2/abs(cos(curr_pitch));
    end_p.x = curr_x+cos(curr_yaw)/l;
    end_p.y = curr_y+sin(curr_yaw)/l;
    end_p.z = curr_z+sin(curr_pitch)/cos(curr_pitch)/l;
    heading_vector.points.push_back(end_p);
    head_pub.publish(heading_vector);
}

void Simulator::drone_controller(const ros::TimerEvent& event){

    //Generate point
    //rand need change
    if(sim_type == 1){
      world_detected_point(0)= Random(world_detected_point(0)-rand_coeff/3, world_detected_point(0)+rand_coeff);
      world_detected_point(1)=Random(world_detected_point(1)-rand_coeff/3, world_detected_point(1)+rand_coeff);
      world_detected_point(2)=Random(1.7, 1.8);
    }
    //Small fast circular
    if(sim_type == 2){
      world_detected_point(0)=5*(cos(PI/60*(count3))-1);
      world_detected_point(1)=5*sin(PI/60*(count3));
      world_detected_point(2)=1.5+count3/30;
    }
    //S shape
    if(sim_type == 3){
      world_detected_point(0)=Random(10*sin(PI/100*(count3))-rand_coeff/66,10*sin(PI/100*(count3))+rand_coeff/66);
      world_detected_point(1)=Random(count3/10-rand_coeff/66,count3/10+rand_coeff/66);
      world_detected_point(2)=Random(1.7, 1.8);
    }
    //Large, slow circular
    if(sim_type == 4){
      world_detected_point(0)=10*(cos(PI/120*(count3))-1);
      world_detected_point(1)=10*sin(PI/120*(count3));
      world_detected_point(2)=1.5+count3/60;
    }
    count3=count3+3;

    auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
    if(duration.count()/1000000.0 > queuing_time){
        pcl::PointXYZ ppp;
        ppp.x = world_detected_point(0); ppp.y = world_detected_point(1); ppp.z = world_detected_point(2);
        queued_pts.push_back(ppp);
        start = high_resolution_clock::now();
        detect_check=true;
    }

    if(detect_check){
        double x = double(queued_pts[last].x);
        double y = double(queued_pts[last].y);
        double z = double(queued_pts[last].z);
        xx.push_back(x); yy.push_back(y); zz.push_back(z);
        for (int l=last+1; l<int(queued_pts.size()); l++){
            float temp_d = euclidean_dist(x,y,z,double(queued_pts[l].x),double(queued_pts[l].y),double(queued_pts[l].z));
            if (temp_d>min_dist && temp_d < max_dist){
                 xx.push_back(double(queued_pts[l].x)); yy.push_back(double(queued_pts[l].y)); zz.push_back(double(queued_pts[l].z));
                 x = double(queued_pts[l].x); y = double(queued_pts[l].y); z = double(queued_pts[l].z);
                 count2=l;
            }
        }
        if (int(xx.size())>min_size){
            ROS_INFO("Trajectory generation");
            MatrixXd waypts = MatrixXd::Zero(3,int(xx.size()));
            for (int i=0 ; i<int(xx.size()); i++){
                waypts(0,i)=xx[i];
                waypts(1,i)=yy[i];
                waypts(2,i)=zz[i];
            }
            accumulated_waypoint.push_back(waypts);
            int k=0;

            //Visualize waypts
            visualization_msgs::MarkerArray waypoints;
            for (int i=0; i<int(accumulated_waypoint.size()); i++) {
                MatrixXd temp_waypts = accumulated_waypoint[i];
                for (int j=0; j<temp_waypts.cols();j++){
                    visualization_msgs::Marker waypts;
                    waypts.header.frame_id = fixed_frame;
                    waypts.header.stamp = ros::Time::now();
                    waypts.type=2;
                    waypts.id = k;
                    waypts.pose.position.x = temp_waypts(0,j);
                    waypts.pose.position.y = temp_waypts(1,j);
                    waypts.pose.position.z = temp_waypts(2,j);
                    waypts.pose.orientation.w = 1.0;
                    waypts.scale.x = 0.2;
                    waypts.scale.y = 0.2;
                    waypts.scale.z = 0.2;
                    waypts.color.a = 1.0;
                    waypts.color.r = 0.5;
                    k++;
                    waypoints.markers.push_back(waypts);
                }
            }
            waypts_pub.publish(waypoints);

            ts = arrangeT(waypts);
            MatrixXd xxx = waypts.row(0);
            MatrixXd yyy = waypts.row(1);
            MatrixXd zzz = waypts.row(2);
            Vel(0,0)=Vel(0,1);
            Vel(1,0)=Vel(1,1);
            Vel(2,0)=Vel(2,1);
            //Test. mean value of final position
            int j = int(xx.size());
            double d = euclidean_dist(double(waypts(0,j-2)),double(waypts(1,j-2)),double(waypts(2,j-2)),double(waypts(0,j-1)),double(waypts(1,j-1)),double(waypts(2,j-1)));
            Vel(0,1)=(waypts(0,j-1)-waypts(0,j-2))/d;
            Vel(1,1)=(waypts(1,j-1)-waypts(1,j-2))/d;
            Vel(2,1)=(waypts(2,j-1)-waypts(2,j-2))/d;

            M1 = PolyQPGeneration(xxx,ts,n_order,Vel.row(0),Acc.row(0),Jerk.row(0),corridor_r);
            M2 = PolyQPGeneration(yyy,ts,n_order,Vel.row(1),Acc.row(1),Jerk.row(1),corridor_r);
            M3 = PolyQPGeneration(zzz,ts,n_order,Vel.row(2),Acc.row(2),Jerk.row(2),corridor_r);

            //queue of the poly_segment
            queued_M1.push_back(M1); queued_M2.push_back(M2); queued_M3.push_back(M3); queued_ts.push_back(ts);
            accumulated_M1.push_back(M1); accumulated_M2.push_back(M2); accumulated_M3.push_back(M3); accumulated_ts.push_back(ts);
            if(debug){
              ROS_WARN("Size of queued_M: %d, size of accumulated_M: %d", int(queued_M1.size()), int(accumulated_M1.size()));
            }
            if (int(queued_M1.size())>point_numbers/min_size){
                queued_M1.erase(queued_M1.begin());
                queued_M2.erase(queued_M2.begin());
                queued_M3.erase(queued_M3.begin());
                queued_ts.erase(queued_ts.begin());
            }
            for (int i=0; i<int(ts.size())-1; i++){
                VectorXd pos = VectorXd::Zero(3);
                VectorXd vel = VectorXd::Zero(3);
                double t = ts(i);
                pos(0) = M1(8*i+0)+M1(8*i+1)*t+M1(8*i+2)*pow(t,2)+M1(8*i+3)*pow(t,3)+M1(8*i+4)*pow(t,4)+M1(8*i+5)*pow(t,5)+M1(8*i+6)*pow(t,6)+M1(8*i+7)*pow(t,7);
                pos(1) = M2(8*i+0)+M2(8*i+1)*t+M2(8*i+2)*pow(t,2)+M2(8*i+3)*pow(t,3)+M2(8*i+4)*pow(t,4)+M2(8*i+5)*pow(t,5)+M2(8*i+6)*pow(t,6)+M2(8*i+7)*pow(t,7);
                pos(2) = M3(8*i+0)+M3(8*i+1)*t+M3(8*i+2)*pow(t,2)+M3(8*i+3)*pow(t,3)+M3(8*i+4)*pow(t,4)+M3(8*i+5)*pow(t,5)+M3(8*i+6)*pow(t,6)+M3(8*i+7)*pow(t,7);
                vel(0) = M1(8*i+1)+M1(8*i+2)*2*pow(t,1)+M1(8*i+3)*3*pow(t,2)+M1(8*i+4)*4*pow(t,3)+M1(8*i+5)*5*pow(t,4)+M1(8*i+6)*6*pow(t,5)+M1(8*i+7)*7*pow(t,6);
                vel(1) = M2(8*i+1)+M2(8*i+2)*2*pow(t,1)+M2(8*i+3)*3*pow(t,2)+M2(8*i+4)*4*pow(t,3)+M2(8*i+5)*5*pow(t,4)+M2(8*i+6)*6*pow(t,5)+M2(8*i+7)*7*pow(t,6);
                vel(2) = M3(8*i+1)+M3(8*i+2)*2*pow(t,1)+M3(8*i+3)*3*pow(t,2)+M3(8*i+4)*4*pow(t,3)+M3(8*i+5)*5*pow(t,4)+M3(8*i+6)*6*pow(t,5)+M3(8*i+7)*7*pow(t,6);
                way_pos.push_back(pos);
                way_vel.push_back(vel);
            }
            last_t = ts(int(ts.size()-1));
            if (int(way_pos.size())>point_numbers){
                way_pos.erase(way_pos.begin());
                way_vel.erase(way_vel.begin());
            }

            //visualize target traj
            nav_msgs::Path target_traj;
            target_traj.header.stamp = ros::Time::now();
            target_traj.header.frame_id = fixed_frame;
            for (int i=0 ; i<int(accumulated_M1.size()); i++){
                VectorXd temp_M1(accumulated_M1[i]); VectorXd temp_M2(accumulated_M2[i]); VectorXd temp_M3(accumulated_M3[i]); VectorXd temp_ts(accumulated_ts[i]);
                for(int j=0 ; j<int(temp_ts.size())-1; j++){
                    for (double t=temp_ts(j); t<temp_ts(j+1); t+=0.05) {
                        geometry_msgs::PoseStamped temp;
                        temp.header.stamp = ros::Time::now();
                        temp.header.frame_id = fixed_frame;
                        temp.pose.position.x = temp_M1(8*j+0)+temp_M1(8*j+1)*t+temp_M1(8*j+2)*pow(t,2)+temp_M1(8*j+3)*pow(t,3)+temp_M1(8*j+4)*pow(t,4)+temp_M1(8*j+5)*pow(t,5)+temp_M1(8*j+6)*pow(t,6)+temp_M1(8*j+7)*pow(t,7);
                        temp.pose.position.y = temp_M2(8*j+0)+temp_M2(8*j+1)*t+temp_M2(8*j+2)*pow(t,2)+temp_M2(8*j+3)*pow(t,3)+temp_M2(8*j+4)*pow(t,4)+temp_M2(8*j+5)*pow(t,5)+temp_M2(8*j+6)*pow(t,6)+temp_M2(8*j+7)*pow(t,7);
                        temp.pose.position.z = temp_M3(8*j+0)+temp_M3(8*j+1)*t+temp_M3(8*j+2)*pow(t,2)+temp_M3(8*j+3)*pow(t,3)+temp_M3(8*j+4)*pow(t,4)+temp_M3(8*j+5)*pow(t,5)+temp_M3(8*j+6)*pow(t,6)+temp_M3(8*j+7)*pow(t,7);
                        target_traj.poses.push_back(temp);
                    }
                }
            }
            target_traj_pub.publish(target_traj);

            last=count2;
            traj_gen_check=true;
        }
        xx.clear(); yy.clear(); zz.clear();
    }

//    UAV Control
    if(traj_gen_check){
        double dt = 1/traj_gen_hz;
        ROS_INFO("Tracking start");
        //For controller
        VectorXd temp_pos(way_pos[0]), temp_vel(way_vel[0]);
        double distance = euclidean_dist(temp_pos(0), temp_pos(1), temp_pos(2), curr_x, curr_y, curr_z);
        for (int i=1 ; i<int(way_pos.size()); i++){
            VectorXd temp_pos(way_pos[i]), temp_vel(way_vel[i]);
            if (euclidean_dist(temp_pos(0), temp_pos(1), temp_pos(2), curr_x, curr_y, curr_z)<distance){
                distance = euclidean_dist(temp_pos(0), temp_pos(1), temp_pos(2), curr_x, curr_y, curr_z);
                if (i>close_i-1){
                    close_i = i;
                }
                else if (i == int(way_pos.size())-1){
                    close_i = i;
                }
            }
        }
        N = int(way_pos.size())-close_i;
        MatrixXd curr_pos(3,1);
        curr_pos << curr_x, curr_y, curr_z;
        MatrixXd u_start(3,1);
        u_start << curr_vel_x, curr_vel_y, curr_vel_z;
        MatrixXd ref_pts(3,N);
//        for (int i=close_i ; i<int(way_pos.size()); i++){
        for (int i=0 ; i<N; i++){
            VectorXd temp2_pos(way_pos[i+close_i]);
            ref_pts(0,i) = temp2_pos(0);
            ref_pts(1,i) = temp2_pos(1);
            ref_pts(2,i) = temp2_pos(2);
        }
        MatrixXd u(3,1);
//        cout<<N<<endl;
//        cout<<ref_pts<<endl;
        ROS_INFO("ref_first=(%.1f, %.1f, %.1f) close i =%d", ref_pts(0,0), ref_pts(1,0), ref_pts(2,0), close_i);
        u = MPC(curr_pos, ref_pts, u_start, dt, N);

        double ux = u(0,0);
        double uy = u(1,0);
        double uz = u(2,0);

        nav_msgs::Path mpc_traj;
        mpc_traj.header.stamp = ros::Time::now();
        mpc_traj.header.frame_id = fixed_frame;
        float x2=curr_x,y2=curr_y,z2=curr_z;
        for (int i=0 ; i<int(u.size()); i=i+3){
            geometry_msgs::PoseStamped temp;
            temp.header.stamp = ros::Time::now();
            temp.header.frame_id = fixed_frame;
            x2 = x2+dt*u(i,0);
            y2 = y2+dt*u(i+1,0);
            z2 = z2+dt*u(i+2,0);
            temp.pose.position.x = x2;
            temp.pose.position.y = y2;
            temp.pose.position.z = z2;
            mpc_traj.poses.push_back(temp);
        }
        mpc_check_pub.publish(mpc_traj);
        double desired_yaw = 0.0;
        if (close_i == int(way_pos.size())-1){
            double t=last_t+predict_time/2;
            int i=M1.size()/8-1;
            MatrixXd M1(queued_M1[int(queued_M1.size())-1]); MatrixXd M2(queued_M2[int(queued_M1.size())-1]); MatrixXd M3(queued_M3[int(queued_M1.size())-1]);

            double goal_x = M1(8*i+0)+M1(8*i+1)*t+M1(8*i+2)*pow(t,2)+M1(8*i+3)*pow(t,3)+M1(8*i+4)*pow(t,4)+M1(8*i+5)*pow(t,5)+M1(8*i+6)*pow(t,6)+M1(8*i+7)*pow(t,7);
            double goal_y = M2(8*i+0)+M2(8*i+1)*t+M2(8*i+2)*pow(t,2)+M2(8*i+3)*pow(t,3)+M2(8*i+4)*pow(t,4)+M2(8*i+5)*pow(t,5)+M2(8*i+6)*pow(t,6)+M2(8*i+7)*pow(t,7);
//            desired_yaw = atan2(goal_y-curr_y, goal_x-curr_x);
        }
        else{
//            desired_yaw = atan2(way_pos[int(way_pos.size())-1](1)-curr_y, way_pos[int(way_pos.size())-1](0)-curr_x);
        }
        desired_yaw = atan2(u(1,0),u(0,0));
        velocity.linear.x = ux;
        velocity.linear.y = uy;
        velocity.linear.z = uz;
        velocity.angular.x = 0.0;
        velocity.angular.y = 0.0;
        if(abs(desired_yaw-curr_yaw)>abs(desired_yaw+curr_yaw)){
            velocity.angular.z = k_yaw*(desired_yaw+curr_yaw);
        }
        else{
            velocity.angular.z = k_yaw*(desired_yaw-curr_yaw);
        }
        if(debug){
            ROS_INFO("ux : %.1f uy : %.1f uz : %.1f d_yaw: %.1f c_yaw: %.1f", ux, uy, uz, desired_yaw, curr_yaw);
//              ROS_INFO("d_v: (%.1f, %.1f, %.1f) c_v: (%.1f, %.1f, %.1f)", ux, uy, uz, curr_vel_x, curr_vel_y,curr_vel_z);
              ROS_INFO("roll: %.1f, pitch: %.1f, yaw: %.1f", curr_roll, curr_pitch, curr_yaw);
        }
        local_vel_pub.publish(velocity);

        info.vel_x = curr_vel_x;
        info.vel_y = curr_vel_y;
        info.vel_z = curr_vel_z;
        info.velocity = euclidean_dist(curr_vel_x,curr_vel_y,curr_vel_z,0,0,0);
        info.yaw = curr_yaw;
        Info_pub.publish(info);
        ros::spinOnce();
    }
    else{
        dummy_controller();
    }
}

void Simulator::dummy_controller(){
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;
    local_vel_pub.publish(velocity);
}

MatrixXd Simulator::MPC(const Eigen::MatrixXd &x_start,
                        const Eigen::MatrixXd &ref_traj,
                        const Eigen::MatrixXd &u_start,
                        const double dt,
                        const int N){
//  p=position x,y,z
//  u=velocity x,y,z control
    int dim = 3;

//    Cost function
    SparseMatrix<double> Q(dim*N,dim*N);
    VectorXd C=VectorXd::Zero(dim*N);
    MatrixXd B_bar=MatrixXd::Zero(dim,N*dim);
    MatrixXd F(dim,1);
    MatrixXd D=MatrixXd::Zero(dim*N,1);
    for (int j=0; j<dim; j++){
        D(j,0)=u_start(j);
    }
    MatrixXd A_bar=MatrixXd::Zero(N*dim,N*dim);
    for (int i=0; i<N; i++){
        for (int l=0; l<dim; l++){
            A_bar(dim*i+l,dim*i+l)=-1;
            if (i>0){
                A_bar(dim*i+l,dim*(i-1)+l)=1;
            }
            B_bar(l,dim*i+l)=dt;
            F(l,0)=ref_traj(l,i)-x_start(l,0);
        }
        //Position distance
        if (C_coeff1 == 1){
            Q = Q + (N-i)*B_bar.transpose()*B_bar;
            C = C - (N-i)*VectorXd(B_bar.transpose()*F);
        }
        if (C_coeff1 == 2){
            Q = Q + pow((N-i),2)*B_bar.transpose()*B_bar;
            C = C - pow((N-i),2)*VectorXd(B_bar.transpose()*F);
        }
        //Best
        if (C_coeff1 == 3){
            Q = Q + B_bar.transpose()*B_bar;
            C = C - VectorXd(B_bar.transpose()*F);
        }
        if (C_coeff1 == 4){
            Q = Q + (i+1)*B_bar.transpose()*B_bar;
            C = C - (i+1)*VectorXd(B_bar.transpose()*F);
        }
        if (C_coeff1 == 5){
            Q = Q + pow((i+1),2)*B_bar.transpose()*B_bar;
            C = C - pow((i+1),2)*VectorXd(B_bar.transpose()*F);
        }
    }
    //Input difference
    Q = Q + C_coeff2*A_bar.transpose()*A_bar;
    C = C + C_coeff2*VectorXd(A_bar.transpose()*D);

//    For constraint
    SparseMatrix<double> L(dim*N,dim*N);
    VectorXd u = VectorXd::Zero(dim*N);
    VectorXd l = VectorXd::Zero(dim*N);
    //    Velocity constraints
    for (int i = 0; i < dim*N ; i++){
        L.insert(i,i)=1;
        u(i)=v_max;
        l(i)=-v_max;
    }

    OsqpEigen::Solver mpcsolver;
    //settings
    mpcsolver.settings()->setWarmStart(true);
    //Set initial data of the QP solver
    mpcsolver.data()->setNumberOfVariables(dim*N);
    mpcsolver.data()->setNumberOfConstraints(dim*N);

    if(!mpcsolver.data()->setHessianMatrix(Q)) return D;

    if(!mpcsolver.data()->setGradient(C)) return D;

    if(!mpcsolver.data()->setLinearConstraintsMatrix(L)) return D;

    if(!mpcsolver.data()->setLowerBound(l)) return D;

    if(!mpcsolver.data()->setUpperBound(u)) return D;

    //Initiate the solver
    if(!mpcsolver.initSolver()) return D;

    if(!mpcsolver.solve()){
      ROS_WARN("Controller Error");
      return D;
    }

    VectorXd x = mpcsolver.getSolution();

    return x;
}

MatrixXd Simulator::PolyQPGeneration(const Eigen::MatrixXd &waypts,
                                     const Eigen::VectorXd &ts,
                                     const int n_order,
                                     const Eigen::MatrixXd &Vel,
                                     const Eigen::MatrixXd &Acc,
                                     const Eigen::MatrixXd &Jerk,
                                     const double Corridor_r){
    //Number of Polynomials
    int n_poly = waypts.cols()-1;
    //Number of Coefficient in each polynomial
    int n_coef = n_order+1;
    //Generate Q(n_coef,n_coef)
    SparseMatrix<double> Q_all(n_coef*n_poly,n_coef*n_poly);
    //blkdiag Q1,Q2,...
    for (int i = 0; i < n_coef*n_poly ; i++){
        for (int j = 0; j < n_coef*n_poly ; j++){
            int i1 = i/n_coef;
            int j1 = j/n_coef;
            if (i1==j1){
                MatrixXd Q = getQ(n_order,4,ts(i1),ts(i1+1));
                int i2 = i - i1*n_coef;
                int j2 = j - j1*n_coef;
                Q_all.insert(i,j) = Q(i2,j2);
            }
        }
    }
    VectorXd b_all = VectorXd::Zero(n_coef*n_poly);
    //Generate Constraints
    SparseMatrix<double> A(7*n_poly+4,n_coef*n_poly);
    VectorXd b_lower = VectorXd::Zero(7*n_poly+4);
    VectorXd b_upper = VectorXd::Zero(7*n_poly+4);
    /*--------------------Equality Constraint---------------------*/
    //Start,End Constraint(p, v, a, j)(8 equation)

    for (int i = 0; i < n_coef; i++){
        A.insert(0,i) = calc_tvec(ts(0),n_order,0)(i); b_lower(0) = waypts(0,0); b_upper(0) = waypts(0,0);
        A.insert(1,i) = calc_tvec(ts(0),n_order,1)(i); b_lower(1) = Vel(0); b_upper(1) = Vel(0);
        A.insert(2,i) = calc_tvec(ts(0),n_order,2)(i); b_lower(2) = Acc(0); b_upper(2) = Acc(0);
        A.insert(3,i) = calc_tvec(ts(0),n_order,3)(i); b_lower(3) = Jerk(0); b_upper(3) = Jerk(0);
    }

    for (int i = n_coef*(n_poly-1); i < n_coef*n_poly; i++){
        A.insert(4,i) = calc_tvec(ts(n_poly),n_order,0)(i-n_coef*(n_poly-1)); b_lower(4) = waypts(n_poly); b_upper(4) = waypts(n_poly);
        A.insert(5,i) = calc_tvec(ts(n_poly),n_order,1)(i-n_coef*(n_poly-1)); b_lower(5) = Vel(1); b_upper(5) = Vel(1);
        A.insert(6,i) = calc_tvec(ts(n_poly),n_order,2)(i-n_coef*(n_poly-1)); b_lower(6) = Acc(1); b_upper(6) = Acc(1);
        A.insert(7,i) = calc_tvec(ts(n_poly),n_order,3)(i-n_coef*(n_poly-1)); b_lower(7) = Jerk(1); b_upper(7) = Jerk(1);
    }
    int neq=8;
    //Continuous Constraint(point, velocity, acceleration, jerk)(4*(n_poly-1) equations)
    for (int i = 1; i < n_poly ; i++){
        for (int j = 0 ; j < n_coef; j++){
            A.insert(neq,n_coef*(i-1)+j) = calc_tvec(ts(i),n_order,0)(j);
            A.insert(neq,n_coef*i+j) = -calc_tvec(ts(i),n_order,0)(j);
            A.insert(neq+1,n_coef*(i-1)+j) = calc_tvec(ts(i),n_order,1)(j);
            A.insert(neq+1,n_coef*i+j) = -calc_tvec(ts(i),n_order,1)(j);
            A.insert(neq+2,n_coef*(i-1)+j) = calc_tvec(ts(i),n_order,2)(j);
            A.insert(neq+2,n_coef*i+j) = -calc_tvec(ts(i),n_order,2)(j);
            A.insert(neq+3,n_coef*(i-1)+j) = calc_tvec(ts(i),n_order,3)(j);
            A.insert(neq+3,n_coef*i+j) = -calc_tvec(ts(i),n_order,3)(j);
        }
        neq=neq+4;
    }
    /*-----------------Inequality Constraint----------------------*/
    for (int i = 0; i < n_poly; i++){
        //Corridor constraints(n_poly equation)
        for (int j = 0; j < n_coef; j++){
            A.insert(4*n_poly+4+i,n_coef*i+j) = calc_tvec(ts(i),n_order,0)(j);
            b_lower(4*n_poly+4+i)=waypts(i)-Corridor_r; b_upper(4*n_poly+4+i)=waypts(i)+Corridor_r;
        }
        //Velocity, acceleration constraints(2*n_poly equation)
        for (int j = 0; j < n_coef; j++){
            A.insert(5*n_poly+4+2*i,n_coef*i+j) = calc_tvec(ts(i),n_order,1)(j);
            b_upper(5*n_poly+4+2*i)=v_max; b_lower(5*n_poly+4+2*i)=-v_max;
            A.insert(5*n_poly+4+2*i+1,n_coef*i+j) = calc_tvec(ts(i),n_order,2)(j);
            b_upper(5*n_poly+4+2*i+1)=a_max; b_lower(5*n_poly+4+2*i+1)=-a_max;
        }
    }
    /*-----------------Solve Quadratic Programming using OSQP-----------*/
    //Initiate the solver
    OsqpEigen::Solver solver2;
    //settings
    solver2.settings()->setWarmStart(true);
    //Set initial data of the QP solver
    solver2.data()->setNumberOfVariables(n_coef*n_poly);
    solver2.data()->setNumberOfConstraints(7*n_poly+4);

    if(!solver2.data()->setHessianMatrix(Q_all)) return b_all;

    if(!solver2.data()->setGradient(b_all)) return b_all;

    if(!solver2.data()->setLinearConstraintsMatrix(A)) return b_all;

    if(!solver2.data()->setLowerBound(b_lower)) return b_all;

    if(!solver2.data()->setUpperBound(b_upper)) return b_all;

    //Initiate the solver
    if(!solver2.initSolver()){
      return b_all;
    }
    if(!solver2.solve()) {
      ROS_WARN("Trajectory planner error");
      VectorXd p = Simulator::PolyQPGeneration(waypts, ts, n_order, Vel, Acc, Jerk, Corridor_r*1.2);
      return p;
    }

    //Get polynomial coefficient
    VectorXd p = solver2.getSolution();
    return p;
}

int Simulator::Factorial(int x)
{
  //Factorial Funation(ex Factorial(5)=5!)
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

MatrixXd Simulator::getQ(const int n, const int r, const double t1, const double t2){
    //n=polynomial order
    //r=derivative order
    //t1:start time, t2=end time
    MatrixXd Q = MatrixXd::Zero(n+1,n+1);
    for (int i = r; i < n+1; i++)
    {
        for (int j = r; j < n+1; j++)
        {
            Q(i,j)=2*(Factorial(i)/Factorial(i-r))*(Factorial(j)/Factorial(j-r))
                    /(i+j-2*r+1)*(pow(t2,i+j-2*r+1)-pow(t1,i+j -2*r+1));
        }
    }
    return Q;
}

//Allocate time for each waypt.
VectorXd Simulator::arrangeT(const Eigen::MatrixXd &waypts){
    int num = waypts.cols();
    VectorXd ts = VectorXd::Zero(num);
    double dist;
    ts(0) = 0;
    for (int i = 0; i < num-1; i++){
        dist = euclidean_dist(double(waypts(0,i)),double(waypts(1,i)),double(waypts(2,i)),double(waypts(0,i+1)),double(waypts(1,i+1)),double(waypts(2,i+1)));
        ts(i+1) = ts(i)+dist/v_max*time_allocation;
    }
    return ts;
}

VectorXd Simulator::calc_tvec(const double t, const int n_order, const int r){
    VectorXd tvec = VectorXd::Zero(n_order+1);
    for (int i = r; i < n_order+1; i++){
        tvec(i) = Factorial(i)/Factorial(i-r)*pow(t,i-r);
    }
    return tvec;
}

#endif
