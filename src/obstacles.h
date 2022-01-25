
#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "tf/LinearMath/Vector3.h"
#include "test_cpp/obstacle_point.h"
#include "test_cpp/obstacle_points.h"



namespace obstacle_avoidance
{
/*    struct point_obstacle //
    {
        tf::Vector3 pos;

        double min_distance = 0.1;
        double max_distance = 10;

        double min_value = 0;
        double max_value = 10;

        double field_coeff;
    };

    struct line_obstacle
    {
        geometry_msgs::Point pos_start;
        geometry_msgs::Point pos_end;

        double min_distance;
        double max_distance;  
    };*/


    tf::Vector3 getFieldFromPoints(const std::vector<test_cpp::obstacle_point>& obstacles, const tf::Vector3& point) 
    {
        tf::Vector3 field;
        field.setX(0);
        field.setY(0);
        field.setZ(0);

        

        //std::cout << field.getX() << std::endl;


        tf::Vector3 obstacle;
        tf::Vector3 diff;
        double dist;
        double value;
        for(test_cpp::obstacle_point obs : obstacles)
        {
            obstacle.setX(obs.pos.x);
            obstacle.setY(obs.pos.y);
            obstacle.setZ(obs.pos.z);
            

            //transform to tf/Vector3
            diff = obstacle - point;
            dist = diff.length();


            if( dist >= obs.min_distance && dist <= obs.max_distance)
            {
                diff.normalize();




                if(obs.type == "linear_decreasing") // magnitude decreases as obstacle gets closer
                {
                    diff = obs.field_coeff * dist * diff; //hmm, maybe do like below and use linear function for the magnitude between min and max in the intervall
                }
                else if(obs.type == "linear_increasing")
                {
                    diff = obs.field_coeff * (obs.min_value - obs.max_value) / (obs.max_distance - obs.min_distance) * (dist - obs.min_distance) * obs.max_value * diff;
                }
                else if(obs.type == "inverse")
                {
                    diff = obs.field_coeff / dist * diff;
                }
                else
                {
                    //unrecognized type
                    diff = diff * 0;
                    std::cout << "Unknown potential field type!!" << std::endl;
                }


                //clamp between min and max
                value = diff.length();
                if(value > obs.max_value)
                {
                    diff = diff.normalize() * obs.max_value;
                }
                else if(value < obs.min_value)
                {
                    diff = diff.normalize() * obs.max_value;
                }

                //add to total field strength

                field += diff;

                
            }
        }

        return field;
    }



    tf::Vector3 getGoalPointFromField(const tf::Vector3& field)
    {
        double maxLookAheadDistance = 0.8;
        double minLookAheadDistance = 0.01;

        tf::Vector3 wp;

        wp = field;

        double length2 = field.length2();

        if(length2 > pow(maxLookAheadDistance, 2))
        {
            wp = field.normalized() * maxLookAheadDistance;
        }
        else if(length2 < pow(minLookAheadDistance, 2))
        {
            wp = field.normalized() * minLookAheadDistance;
        }

        return wp;
    }







} // namespace obstacle_avoidance









#endif