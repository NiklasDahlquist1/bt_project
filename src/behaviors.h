

#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"


#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include <chrono>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <test_action/MoveToAction.h>
#include <actionlib/client/terminal_state.h>

#include "obstacles.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"




#include <random>



#include "state.h"

//using namespace BT;


namespace behaviors
{


    class SafetyChecks : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            SafetyChecks(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("safetychecks, pointer to state = null");
                }
                
                return BT::NodeStatus::SUCCESS;
            }
    };




    class IsFlying : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            IsFlying(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return { };
            }

            BT::NodeStatus tick() override
            {
                if(state == nullptr)
                {
                    ROS_INFO_STREAM("IsFlying, pointer to state = null");
                }

                if(state->isFlying == true)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                else if(state->isFlying == false)
                {
                    return BT::NodeStatus::FAILURE;
                }
                //
            }
    };

    class HasCollisionFreeWP : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            HasCollisionFreeWP(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("HasCollisionFreeWP, pointer to state = null");
                }
                //allways return failure to get a new point from the potential field at each step
                return BT::NodeStatus::FAILURE;

                //
            }
    };


    class CheckTaskFailUAVAtPoint : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
            bool failLastTick = false;
            
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            CheckTaskFailUAVAtPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("CheckTaskFailUAVAtPoint, pointer to state = null");
                }
                
                // check if the task has failed, to be able to sell it again

                //simulate motor failure
                if(state->motorFail == true)
                {
                    // set bool to stop bidding TODO implement
                    state->canBidForNewTask = false;
                    return BT::NodeStatus::FAILURE;
                }


                double failHeight = 1.8;

                    // test check heigh, to not break "noTaskTree", make sure that the task wont fail if UAV is already to high, 
                if(state->mavPose.position.z > failHeight && state->taskIsActive && state->goalPose.position.z > failHeight) // remove
                {

                    //must fail to ticks in a row (to allow another node to set goalPose...)
                    if(failLastTick == false)
                    {
                        failLastTick = true;
                    }
                    else
                    {
                        //publish current uav pos once to make sure the uav does not continue moving, TODO, maybe set goal point instead (and continue running a BT)?
                        geometry_msgs::PoseStamped p;

                        //set hold pos  
                        if(state->UAVAtTasFailedkHoldPointSet == false)
                        {
                            state->UAVTaskFailedHoldPoint = state->mavPose;

                            state->UAVAtTasFailedkHoldPointSet = true;
                        }
                        else
                        {

                        }

                        
                        p.pose = state->UAVTaskFailedHoldPoint;
                        p.header.stamp = ros::Time::now();
                        state->pub_uavWP.publish(p);

                        return BT::NodeStatus::FAILURE;
                    }
                }
                else
                {
                    state->UAVAtTasFailedkHoldPointSet = false;
                    failLastTick = false;
                    return BT::NodeStatus::SUCCESS;
                }

                return BT::NodeStatus::SUCCESS;
            }
    };

    class CheckTaskFailExplore : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
            bool failLastTick = false;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            CheckTaskFailExplore(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("CheckTaskFailUAVAtPoint, pointer to state = null");
                }


                // check if the task has failed, to be able to sell it again

                //simulate motor failure

                if(state->motorFail == true)
                {
                    // set bool to stop bidding TODO implement
                    state->canBidForNewTask = false;
                    return BT::NodeStatus::FAILURE;
                }
                
                // check if the task has failed, to be able to sell it again

                return BT::NodeStatus::SUCCESS;

            }
    };


    class UAVAtPoint : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            UAVAtPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("point"), BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtPoint, pointer to state = null");
                }



                BT::Optional<geometry_msgs::Point> msg = getInput<geometry_msgs::Point>("point");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }
                geometry_msgs::Point p;
                p = msg.value();

                //set ouput goal
                setOutput("goalPoint", p );


                //set goal pose f

                state->goalPose.position = p; //TODO, remove
                state->goalPose.orientation.x = 0;
                state->goalPose.orientation.y = 0;
                state->goalPose.orientation.z = 0;
                state->goalPose.orientation.w = 1;

                double error2 = pow(state->mavPose.position.x - p.x, 2) + pow(state->mavPose.position.y - p.y, 2) + pow(state->mavPose.position.z - p.z, 2);

                double tol = 0.1; // tolerance to accept current position
                if(error2 < pow(tol, 2))
                {
                    //ROS_INFO_STREAM("UAV at point, action SUCCESS");

                    //publish uav waypoint once more to make sure that the uav has the exact goal
                    geometry_msgs::PoseStamped p;
                    p.header.stamp = ros::Time::now();
                    p.pose.position = msg.value();
                    state->pub_uavWP.publish(p);
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };


class UAVAtPointOnce : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;

            bool visitedPoint = false;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            UAVAtPointOnce(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("point"), BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtPoint, pointer to state = null");
                }


                BT::Optional<geometry_msgs::Point> msg = getInput<geometry_msgs::Point>("point");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }
                geometry_msgs::Point p;
                p = msg.value();

                // check if point already visited
                if(this->visitedPoint == true)
                {
                    return BT::NodeStatus::SUCCESS;
                }

                //set ouput goal
                setOutput("goalPoint", p );


                //set goal pose f

                state->goalPose.position = p; //TODO, remove
                state->goalPose.orientation.x = 0;
                state->goalPose.orientation.y = 0;
                state->goalPose.orientation.z = 0;
                state->goalPose.orientation.w = 1;

                double error2 = pow(state->mavPose.position.x - p.x, 2) + pow(state->mavPose.position.y - p.y, 2) + pow(state->mavPose.position.z - p.z, 2);

                double tol = 0.1; // tolerance to accept current position
                if(error2 < pow(tol, 2))
                {
                    this->visitedPoint = true;
                    //ROS_INFO_STREAM("UAV at point, action SUCCESS");

                    //publish uav waypoint once more to make sure that the uav has the exact goal
                    geometry_msgs::PoseStamped p;
                    p.header.stamp = ros::Time::now();
                    p.pose.position = msg.value();
                    state->pub_uavWP.publish(p);
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };



    class GetPointFromPotentialField : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            GetPointFromPotentialField(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("goalPoint"), BT::OutputPort<geometry_msgs::Point>("wp") };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
                }

                BT::Optional<geometry_msgs::Point> msg = getInput<geometry_msgs::Point>("goalPoint");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }
                geometry_msgs::Point goalPoint;
                goalPoint = msg.value();


                std::vector<bt_project::obstacle_point> obs_action = state->obstacles_points.obstacles;//obs_action;//= state->obstacles_points.obstacles;

                bt_project::obstacle_point goalPF;
                //goalPF.pos.x = state->goalPose.position.x;
                //goalPF.pos.y = state->goalPose.position.y;
                //goalPF.pos.z = state->goalPose.position.z;
                goalPF.pos.x = goalPoint.x;
                goalPF.pos.y = goalPoint.y;
                goalPF.pos.z = goalPoint.z;


                goalPF.field_coeff = 1.5;
                goalPF.min_value = 0.1;
                goalPF.max_value = 5;
                goalPF.min_distance = 0;
                goalPF.max_distance = 1000;
                goalPF.type = "linear_decreasing";


                obs_action.push_back(goalPF);

                geometry_msgs::Point goal;

                tf::Vector3 posUAV;
                posUAV.setX(state->mavPose.position.x);
                posUAV.setY(state->mavPose.position.y);
                posUAV.setZ(state->mavPose.position.z);

                tf::Vector3 field = obstacle_avoidance::getFieldFromPoints(obs_action, posUAV);
                tf::Vector3 goalVec = obstacle_avoidance::getGoalPointFromField(field);
                goalVec = goalVec + posUAV; // add current position, goalVec is only the direction to move in
                goal.x = goalVec.getX();
                goal.y = goalVec.getY();
                goal.z = goalVec.getZ();
                
                
                //std::cout << "x: " << goal.position.x << " y: " << goal.position.y << " z: " << goal.position.z << std::endl;
                setOutput("wp", goal );
                
                return BT::NodeStatus::SUCCESS;
            }
    };




    // returns failure to stop the bt from quiting when a point is published
    class PublishUAVWayPoint : public BT::StatefulActionNode
    {
        private: 
            bt_state::mav_state* state;

            BT::NodeStatus logic()
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
                }

                BT::Optional<geometry_msgs::Point> msg = getInput<geometry_msgs::Point>("wp");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }
                //ros::NodeHandle n;
                //ros::Publisher pub = state->nodeHandle.advertise<geometry_msgs::PoseStamped>("uavGoal", 1000);

                //ROS_INFO_STREAM(msg.value());

                geometry_msgs::PoseStamped p;
                p.header.stamp = ros::Time::now();
                p.pose.position = msg.value();
                p.pose.orientation.x = 0;
                p.pose.orientation.y = 0;
                p.pose.orientation.z = 0;
                p.pose.orientation.w = 1;

                state->pub_uavWP.publish(p);

                return BT::NodeStatus::RUNNING; // allways return running?
            }
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            PublishUAVWayPoint(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("wp") };
            }
            

            BT::NodeStatus onStart()
            {
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }

            /*NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
                }

                Optional<geometry_msgs::Pose> msg = getInput<geometry_msgs::Pose>("goalPose");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }
                //ros::NodeHandle n;
                //ros::Publisher pub = state->nodeHandle.advertise<geometry_msgs::PoseStamped>("uavGoal", 1000);

                //ROS_INFO_STREAM(msg.value());

                geometry_msgs::PoseStamped p;
                p.header.stamp = ros::Time::now();
                p.pose = msg.value();
                state->pub_uavWP.publish(p);

                return NodeStatus::FAILURE; //
            }*/
    };




















    // statefulactionnodes

    class MoveToBTAction : public BT::StatefulActionNode
    {
    public:   

        actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
        test_action::MoveToGoal goal_;


        MoveToBTAction(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
        , actionClient_("mav_moveto_action", true)
        {
            ROS_INFO("Waiting for action server");
            // wait for the action server to start
            actionClient_.waitForServer();
            ROS_INFO("Server started");
            
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<geometry_msgs::Pose>("goalPose") };
        }

        BT::NodeStatus onStart()
        {
            //goal_.pose_goal.position.x = 12;
            //goal_.pose_goal.position.y = 0;
            //goal_.pose_goal.position.z = 4;
            
            BT::Optional<geometry_msgs::Pose> msg = getInput<geometry_msgs::Pose>("goalPose");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error());
            }
            goal_.pose_goal = msg.value();
            
            actionClient_.sendGoal(goal_);
            actionClient_.waitForResult(ros::Duration(0.01)); //wait for some response, otherwise the onRunning returns fail the first tick

            actionlib::SimpleClientGoalState state = actionClient_.getState();  
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            BT::Optional<geometry_msgs::Pose> msg = getInput<geometry_msgs::Pose>("goalPose");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error());
            }
            goal_.pose_goal = msg.value();
            actionClient_.sendGoal(goal_);



            actionlib::SimpleClientGoalState state = actionClient_.getState();    

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }
        } 

        void onHalted()
        {
            actionClient_.cancelAllGoals();
            //
        }
    };





    //Fly one meter above current position
    class TakeOff : public BT::StatefulActionNode
    {
    public:   

        actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
        test_action::MoveToGoal goal_;

        bt_state::mav_state* state;
        geometry_msgs::Pose startPose;

        TakeOff(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
        , actionClient_("mav_moveto_action", true)
        {
            ROS_INFO("Waiting for action server");
            // wait for the action server to start
            actionClient_.waitForServer();
            ROS_INFO("Server started");
            
        }


        void init(bt_state::mav_state* statePtr)
        {
            if(statePtr == nullptr)
            {
                //error
            }
            this->state = statePtr;
        }

        static BT::PortsList providedPorts()
        {
            return{};
        }

        BT::NodeStatus onStart()
        {
            //goal_.pose_goal.position.x = 12;
            //goal_.pose_goal.position.y = 0;
            //goal_.pose_goal.position.z = 4;


            startPose = this->state->mavPose;

            goal_.pose_goal = startPose;
            //add one meter to z
            goal_.pose_goal.position.z += 0.5;
            
            
            actionClient_.sendGoal(goal_);
            actionClient_.waitForResult(ros::Duration(0.01)); //wait for some response, otherwise the onRunning returns fail the first tick

            actionlib::SimpleClientGoalState state = actionClient_.getState();  
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                this->state->isFlying = true;
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            actionlib::SimpleClientGoalState state = actionClient_.getState();    

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                this->state->isFlying = true;
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }
        }

        void onHalted()
        {
            actionClient_.cancelAllGoals();
            //
        }


    };




    class LandAtPoint : public BT::StatefulActionNode
    {
    public:   

        actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
        test_action::MoveToGoal goal_;

        bt_state::mav_state* state;

        LandAtPoint(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config)
        , actionClient_("mav_moveto_action", true)
        {
            ROS_INFO("Waiting for action server");
            // wait for the action server to start
            actionClient_.waitForServer();
            ROS_INFO("Server started");
            
        }


        void init(bt_state::mav_state* statePtr)
        {
            if(statePtr == nullptr)
            {
                //error
            }
            this->state = statePtr;
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<geometry_msgs::Point>("landingPoint") };
        }

        BT::NodeStatus onStart()
        {
            BT::Optional<geometry_msgs::Point> msg_landingPoint = getInput<geometry_msgs::Point>("landingPoint");
            if (!msg_landingPoint)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg_landingPoint.error());
            }

            // set landing point as goal, 

            goal_.pose_goal.position = msg_landingPoint.value();
            goal_.pose_goal.orientation.x = 0;
            goal_.pose_goal.orientation.y = 0;
            goal_.pose_goal.orientation.z = 0;
            goal_.pose_goal.orientation.w = 1;
            
            actionClient_.sendGoal(goal_);
            actionClient_.waitForResult(ros::Duration(0.01)); //wait for some response, otherwise the onRunning returns fail the first tick

            actionlib::SimpleClientGoalState state = actionClient_.getState();  
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning()
        {
            actionlib::SimpleClientGoalState state = actionClient_.getState();    

            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                //ROS_INFO("Goal reached");
                return BT::NodeStatus::SUCCESS;
            } 
            else if (state == actionlib::SimpleClientGoalState::ACTIVE) 
            {
                //ROS_INFO("Moving towards goal");
                return BT::NodeStatus::RUNNING;
            } 
            else 
            {
                //ROS_INFO("Action failed to reach goal");
                return BT::NodeStatus::FAILURE;
            }
        }

        void onHalted()
        {
            actionClient_.cancelAllGoals();
            //
        }


    };








    class DelayOnce : public BT::StatefulActionNode
    {
        private: 
            bt_state::mav_state* state;

            ros::Time startTime;

            bool alreadyDelayed = false;

            BT::NodeStatus logic()
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
                }

                BT::Optional<double> msg = getInput<double>("delayTime");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }

                //ROS_INFO_STREAM("Time left" << startTime.toSec() - ros::Time::now().toSec() + msg.value());
                if(ros::Time::now().toSec() > startTime.toSec() + msg.value() || alreadyDelayed)
                {
                    alreadyDelayed = true;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }

        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            DelayOnce(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<double>("delayTime") };
            }

            BT::NodeStatus onStart()
            {
                startTime = ros::Time::now();
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };




    // allways return failure to get a new goal
    class ExploredSomeTime : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;

            ros::Time startTime;
            bool hasStartTime = false;
            bool hasSetStartPoint = false;

            bool remove = false;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            ExploredSomeTime(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<double>("exploreTime"), BT::InputPort<geometry_msgs::Point>("startPoint"), BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                if(this->hasStartTime == false)
                {
                    startTime = ros::Time::now();
                    this->hasStartTime = true;
                }

                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("ExploredSomeTime, pointer to state = null");
                }

                BT::Optional<double> msg_exploreTime = getInput<double>("exploreTime");
                if (!msg_exploreTime)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg_exploreTime.error());
                }
                BT::Optional<geometry_msgs::Point> msg_goalPoint = getInput<geometry_msgs::Point>("startPoint");
                if (!msg_goalPoint)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg_goalPoint.error());
                }

                
                // set output start point
                if(this->hasSetStartPoint == false)
                {
                    //TODO plotting, send first goal
                    state->pub_goal_plot.publish(msg_goalPoint.value());

                    setOutput("goalPoint", msg_goalPoint.value());
                    hasSetStartPoint = true;
                }

                //ROS_INFO_STREAM("Time left" << startTime.toSec() - ros::Time::now().toSec() + msg.value());
                if(ros::Time::now().toSec() > startTime.toSec() + msg_exploreTime.value())
                {
                    if(this->remove == false)
                    {
                        //TODO plotting, send stop pos
                        state->pub_cancel_explore.publish(state->mavPose.position);


                        // publish current position once to stop
                        geometry_msgs::PoseStamped p;
                        p.header.stamp = ros::Time::now();
                        p.pose.position = state->mavPose.position;
                        state->pub_uavWP.publish(p);

                        this->remove == true;
                    }

                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };


    class LandedAtPoint : public BT::SyncActionNode //TODO, return success if not flying??
    {
        private: 
            bt_state::mav_state* state;

            ros::Time startTime;
            bool hasStartTime = false;
            bool hasSetStartPoint = false;

            bool remove = false;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            LandedAtPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("pointToLandAt"), BT::OutputPort<geometry_msgs::Point>("landingPoint"), BT::OutputPort<geometry_msgs::Point>("pointAboveLanding") };
            }

            BT::NodeStatus tick() override
            {
                if(this->hasStartTime == false)
                {
                    startTime = ros::Time::now();
                    this->hasStartTime = true;
                }

                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("ExploredSomeTime, pointer to state = null");
                }

                BT::Optional<geometry_msgs::Point> msg_pointToLandAt = getInput<geometry_msgs::Point>("pointToLandAt");
                if (!msg_pointToLandAt)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg_pointToLandAt.error());
                }
                
                

                geometry_msgs::Point pointAboveLanding;
                pointAboveLanding = msg_pointToLandAt.value();
                pointAboveLanding.z += 0.5;
                // set output
                setOutput("pointAboveLanding", pointAboveLanding);
                setOutput("landingPoint", msg_pointToLandAt.value());






                double error2 = pow(state->mavPose.position.x - msg_pointToLandAt.value().x, 2) + 
                                pow(state->mavPose.position.y - msg_pointToLandAt.value().y, 2) + 
                                pow(state->mavPose.position.z - msg_pointToLandAt.value().z, 2);


                double tol = 0.1; // tolerance to accept current position
                if(error2 < pow(tol, 2))
                {
                    if(this->remove == false)
                    {
                        std::cout << "LANDED " << ros::this_node::getName() << std::endl;
                        this->state->isFlying = false;


                        // publish goal position once to stop
                        geometry_msgs::PoseStamped p;
                        p.header.stamp = ros::Time::now();
                        p.pose.position = msg_pointToLandAt.value();
                        state->pub_uavWP.publish(p);

                        this->remove = true;
                    }

                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    this->remove = false;
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };





    // allways return failure to get a new goal
    class HasRandomPoint : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;
        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            HasRandomPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("HasRandomPoint, pointer to state = null");
                }
                // allways return failure to get a new goal
                return BT::NodeStatus::FAILURE;

                //
            }
    };


    class GetNewRandomPoint : public BT::SyncActionNode
    {
        private: 
            bt_state::mav_state* state;

            tf::Vector3 direction;
            std::random_device rd;
            std::mt19937 e2;
            std::uniform_real_distribution<> dist;

        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;



                direction = tf::Vector3(1, 0, 0); // start exploring direction

            }

            GetNewRandomPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config), e2(rd()), dist(0, 1)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("GetNewRandomPoint, pointer to state = null");
                }

                //std::random_device rd;
                //std::mt19937 e2(rd());
                //std::uniform_real_distribution<> dist(0, 1);



                geometry_msgs::Point goal;
                // TODO, maybe make random points better
                //goal.x = dist(e2) * 10 - 5;
                //goal.y = dist(e2) * 10 - 5;
                //goal.z = dist(e2) * 3.8 + 0.8;

                goal = state->mavPose.position;
                goal.x += dist(e2) * 5 - 2.5; // move to a position somewhat close to the current position
                goal.y += dist(e2) * 5 - 2.5;
                goal.z += dist(e2) * 5 - 2.5;

/*
                tf::Matrix3x3 rot;
                tf::Quaternion rotQ;
                double r = 0;
                double p = 0.1;
                double y = 0.1;

                rotQ.setRPY(r, p, y);
                rot.getRotation(rotQ);
                //rot.setRPY(r, p, y);
                direction = rot * direction;

                std::cout << direction.length() << std::endl;
                double distance = 3;
                goal.x += direction.getX();
                goal.y += direction.getY();
                goal.z += direction.getZ();
*/

                if(goal.z < 0.8) // make sure we dont explore too low
                    goal.z = 0.8;


                //TODO plotting
                state->pub_goal_plot.publish(goal);

                setOutput("goalPoint", goal);
                return BT::NodeStatus::SUCCESS;
            }
    };



    class Explore : public BT::StatefulActionNode
    {
        private: 
            bt_state::mav_state* state;


            BT::NodeStatus logic()
            {

                return BT::NodeStatus::RUNNING;
            }

        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            Explore(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus onStart()
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("Explore, pointer to state = null");
                }
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };



} // end namespace







/*

// Template specialization to converts a string to Position2D.
namespace BT
{

    template <> inline geometry_msgs::Pose convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Pose output;
            output.position.x    = convertFromString<double>(parts[0]);
            output.position.y    = convertFromString<double>(parts[1]);
            output.position.z    = convertFromString<double>(parts[2]);
            output.orientation.x = convertFromString<double>(parts[3]);
            output.orientation.y = convertFromString<double>(parts[4]);
            output.orientation.z = convertFromString<double>(parts[5]);
            output.orientation.w = convertFromString<double>(parts[6]);
            return output;
        }
    }

    template <> inline geometry_msgs::Point convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Point output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            return output;
        }
    }


    template <> inline bool convertFromString(StringView str)
    {
        bool output;
        if (str == "true")
        {
            output = true;
        }
        else if (str == "false")
        {
            output = false;
        }
        else
        {
            throw RuntimeError("invalid input)");
        }

        return output;
    }
} // end namespace BT


*/








// Template specialization to convert.
namespace BT
{

    template <> inline geometry_msgs::Pose convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Pose output;
            output.position.x    = convertFromString<double>(parts[0]);
            output.position.y    = convertFromString<double>(parts[1]);
            output.position.z    = convertFromString<double>(parts[2]);
            output.orientation.x = convertFromString<double>(parts[3]);
            output.orientation.y = convertFromString<double>(parts[4]);
            output.orientation.z = convertFromString<double>(parts[5]);
            output.orientation.w = convertFromString<double>(parts[6]);
            return output;
        }
    }

    template <> inline geometry_msgs::Point convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Point output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            return output;
        }
    }


    template <> inline bool convertFromString(StringView str)
    {
        bool output;
        if (str == "true")
        {
            output = true;
        }
        else if (str == "false")
        {
            output = false;
        }
        else
        {
            throw RuntimeError("invalid input)");
        }

        return output;
    }
} // end namespace BT








#endif






