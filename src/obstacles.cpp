

#include "obstacles.h"

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

std::vector<bt_project::obstacle_point> obstaclePositions(5);


void updateObstacle1(const nav_msgs::Odometry& msg)
{
    obstaclePositions[0].pos.x = msg.pose.pose.position.x;//msg.position.x;
    obstaclePositions[0].pos.y = msg.pose.pose.position.y;//msg.position.y;
    obstaclePositions[0].pos.z = msg.pose.pose.position.z;//msg.position.z;

}

void updateObstacle2(const nav_msgs::Odometry& msg)
{
    obstaclePositions[1].pos.x = msg.pose.pose.position.x;
    obstaclePositions[1].pos.y = msg.pose.pose.position.y;
    obstaclePositions[1].pos.z = msg.pose.pose.position.z;

}
void updateObstacle3(const nav_msgs::Odometry& msg)
{
    obstaclePositions[2].pos.x = msg.pose.pose.position.x;
    obstaclePositions[2].pos.y = msg.pose.pose.position.y;
    obstaclePositions[2].pos.z = msg.pose.pose.position.z;

}
void updateObstacle4(const nav_msgs::Odometry& msg)
{
    obstaclePositions[3].pos.x = msg.pose.pose.position.x;
    obstaclePositions[3].pos.y = msg.pose.pose.position.y;
    obstaclePositions[3].pos.z = msg.pose.pose.position.z;
}
void updateObstacle5(const nav_msgs::Odometry& msg)
{
    obstaclePositions[4].pos.x = msg.pose.pose.position.x;
    obstaclePositions[4].pos.y = msg.pose.pose.position.y;
    obstaclePositions[4].pos.z = msg.pose.pose.position.z;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacles");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(100);

    for(bt_project::obstacle_point& p : obstaclePositions)
    {
        p.pos.x = -MAXFLOAT; // init to a pos far away, to not collide if UAV does not exist
        p.min_distance = 0.33;
        p.max_distance = 1.2; //1.3
        p.min_value = 0.1;
        p.max_value = 0.4;

        p.field_coeff = -0.8;
        p.type = "inverse";
    }

    //test obstacle
    /*bt_project::obstacle_point p;
    p.pos.x = 0;
    p.pos.y = 0;
    p.pos.z = 1;


    p.min_distance = 0;
    p.max_distance = 1.5;
    p.min_value = 0.2;
    p.max_value = 5;

    p.field_coeff = -2;
    p.type = "inverse";


    obstaclePositions.push_back(p);
    */


    //ugly, but for testing
    ros::Subscriber sub_mav1 = nodeHandle.subscribe("odometry_UAV_1", 1000, updateObstacle1);
    ros::Subscriber sub_mav2 = nodeHandle.subscribe("odometry_UAV_2", 1000, updateObstacle2);    
    ros::Subscriber sub_mav3 = nodeHandle.subscribe("odometry_UAV_3", 1000, updateObstacle3);
    ros::Subscriber sub_mav4 = nodeHandle.subscribe("odometry_UAV_4", 1000, updateObstacle4);
    ros::Subscriber sub_mav5 = nodeHandle.subscribe("odometry_UAV_5", 1000, updateObstacle5);


    ros::Publisher pub_obstacles = nodeHandle.advertise<bt_project::obstacle_points>("obstacles_points", 1000);

    bt_project::obstacle_points msg;

    while(ros::ok())
    {
        ros::spinOnce();

        msg.obstacles = obstaclePositions;
        pub_obstacles.publish(msg);

        loop_rate.sleep();
    }

    return 0;
}



