#include <ros/ros.h>
#include <vector> //import nump as np
#include <math.h>

#include <test_msgs/Test.h>
#include <lattice_planning/Traj.h>
#include <rs_perception/PerceptionListMsg.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <fsd_common_msgs/Gnss.h>
#include <fsd_common_msgs/CarState.h>
#include <fsd_common_msgs/CarStateDt.h>
#include <fsd_common_msgs/Trajectory.h>
#include <fsd_common_msgs/TrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>

//-------------------------------------------
#include<memory>///智能指针 
class HQPlanner
{
private:
    /* data */
public:




    HQPlanner(/* args */);
    ~HQPlanner();

    private:
    std::vector<std::unique_ptr<Task>>tasks_;
};

HQPlanner::HQPlanner(/* args */)
{
}

HQPlanner::~HQPlanner()
{
}
