



#ifndef STATE_H
#define STATE_H

#include "std_msgs/Bool.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include "test_cpp/auction_auction.h"
#include "ros/ros.h"
#include "test_cpp/obstacle_points.h"

#include "nav_msgs/Odometry.h"


namespace bt_state
{

    // MAV
    
    struct mav_state
    {
        geometry_msgs::Pose mavPose;
        geometry_msgs::Pose goalPose;
        geometry_msgs::Pose homePosition;

        test_cpp::auction_auction currentTask;

        test_cpp::obstacle_points obstacles_points;

        ros::NodeHandle nodeHandle;
        ros::Publisher pub_uavWP;

        bool isFlying = false;
        bool taskIsNew = false;

        bool taskIsActive = false;

        bool canBidForNewTask = true; // use for stopping uav from bidding, maybe after a "crash" TODO implement

        bool motorFail = false;

        // for plotting
        ros::Publisher pub_goal_plot;
        ros::Publisher pub_cancel_explore;
    };




    mav_state* statePtr;
    bool homeSet = false;
      // use the nav_msgs::odometry?
    void mavPoseCB(const nav_msgs::Odometry& msg)
    {
        statePtr->mavPose = msg.pose.pose;

        if(homeSet == false)
        {
            statePtr->homePosition = msg.pose.pose;
            homeSet = true;
        }
    }
/*
    void mavPoseCB(const geometry_msgs::Pose& msg)
    {
        statePtr->mavPose = msg;

        if(homeSet == false)
        {
            statePtr->homePosition = msg;
            homeSet = true;
        }
    }
*/



    void obstaclePointsCB(const test_cpp::obstacle_points& msg)
    {
        statePtr->obstacles_points = msg;
    }

    void motorFailCB(const std_msgs::Bool msg)
    {
        statePtr->motorFail = msg.data;
    }

    void initMAVStateCBPtr(mav_state& state)
    {
        statePtr = &state;
        //ros::NodeHandle nodeHandle;
        //ros::Subscriber sub_pose = nodeHandle.subscribe("/pose", 1000, bt_state::mavPoseCB);

        // init publishers
        state.pub_uavWP = state.nodeHandle.advertise<geometry_msgs::PoseStamped>("uavGoal", 100);


        state.pub_goal_plot = state.nodeHandle.advertise<geometry_msgs::Point>("plot/explore_goal", 100);
        state.pub_cancel_explore = state.nodeHandle.advertise<geometry_msgs::Point>("plot/explore_cancel", 100);


    }










    // walker
    
    struct walker_state
    {
        geometry_msgs::Pose walkerPose;


        ros::NodeHandle nodeHandle;
    };

};










#endif

