// #include <ros/ros.h>
// #include <vector>
// #include <math.h>

// #include <test_msgs/Test.h>
// #include <lattice_planning/Traj.h>
// #include <rs_perception/PerceptionListMsg.h>
// #include <sensor_msgs/Imu.h>
// #include <nav_msgs/Odometry.h>
// #include <fsd_common_msgs/Gnss.h>
// #include <fsd_common_msgs/CarState.h>
// #include <fsd_common_msgs/CarStateDt.h>
// #include <fsd_common_msgs/Trajectory.h>
// #include <fsd_common_msgs/TrajectoryPoint.h>
// #include <nav_msgs/Path.h>
// #include <std_msgs/Header.h>
// #include <geometry_msgs/Pose2D.h>
// #include <ros/time.h>
// #include<memory>///智能指针

#include "subscribe.h"
using hqplanner::Subscribe;
int main(int argc, char **argv) {
  ros::init(argc, argv, "hqplanner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    //更新Subscribe中订阅的信息
    Subscribe::UpdateSubscribeInfo(nh);
    ros::spinOnce();  //调用回调函数更新Subscribe中订阅的信息？？？？

    /*...TODO...*/

    loop_rate.sleep();
  }
}
