




#ifndef ACTIONS_H
#define ACTIONS_H

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



#include "state.h"

using namespace BT;




class SetGoalPose : public SyncActionNode
{
    private:    
        geometry_msgs::Pose goalPose;
    public:

        geometry_msgs::Pose outputPose;


        SetGoalPose(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        // We want this method to be called ONCE and BEFORE the first tick()
        void init( geometry_msgs::Pose pose )
        {
            goalPose = pose;
        }

        static PortsList providedPorts()
        {
            return { OutputPort<geometry_msgs::Pose>("goalPose"), OutputPort<bool>("goalSet")  };
        }

        // This Action writes a value into the port "message"
        NodeStatus tick() override
        {
            // the output may change at each tick(). Here we keep it simple.
            setOutput("goalPose", goalPose);
            setOutput("goalSet", true);
            return NodeStatus::SUCCESS;
        }
};



class SafetyChecks : public SyncActionNode
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

        SafetyChecks(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{  };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("safetychecks, pointer to state = null");
            }
            
            return NodeStatus::SUCCESS;
        }
};




class IsFlying : public SyncActionNode
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

        IsFlying(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return { };
        }

        NodeStatus tick() override
        {
            if(state == nullptr)
            {
                ROS_INFO_STREAM("IsFlying, pointer to state = null");
            }

            if(state->isFlying == true)
            {
                return NodeStatus::SUCCESS;
            }
            else if(state->isFlying == false)
            {
                return NodeStatus::FAILURE;
            }
            //
        }
};

class HasCollisionFreeWP : public SyncActionNode
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

        HasCollisionFreeWP(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{  };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("HasCollisionFreeWP, pointer to state = null");
            }
            //allways return failure to get a new point from the potential field at each step
            return NodeStatus::FAILURE;

            //
        }
};


class CheckTaskFailUAVAtPoint : public SyncActionNode
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

        CheckTaskFailUAVAtPoint(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{  };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("CheckTaskFailUAVAtPoint, pointer to state = null");
            }
            
            // check if the task has failed, to be able to sell it again


                // test check heigh, to not break "noTaskTree", make sure that the task wont fail if UAV is already to high, 
            if(state->mavPose.position.z > 3 && state->taskIsActive && state->goalPose.position.z > 3) // remove
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
                    p.header.stamp = ros::Time::now();
                    p.pose.position = state->mavPose.position;
                    state->pub_uavWP.publish(p);

                    return NodeStatus::FAILURE;
                }
            }
            else
            {
                failLastTick = false;
                return NodeStatus::SUCCESS;
            }
        }
};


class UAVAtPoint : public SyncActionNode
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

        UAVAtPoint(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{ InputPort<geometry_msgs::Point>("point") };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("UAVAtPoint, pointer to state = null");
            }


            Optional<geometry_msgs::Point> msg = getInput<geometry_msgs::Point>("point");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error());
            }
            geometry_msgs::Point p;
            p = msg.value();

            //set goal pose f

            state->goalPose.position = p;
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
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::FAILURE;
            }

            //
        }
};



class UAVAtHomePoint : public SyncActionNode
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

        UAVAtHomePoint(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{ };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
            }



            double error2 = pow(state->mavPose.position.x - state->homePosition.position.x, 2) + 
                            pow(state->mavPose.position.y - state->homePosition.position.y, 2) + 
                            pow(state->mavPose.position.z - state->homePosition.position.z, 2);

            double tol = 0.1; // tolerance to accept current position
            if(error2 < pow(tol, 2))
            {
                this->state->isFlying = false;
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::FAILURE;
            }


            //
        }
};



class GetPointFromPotentialField : public SyncActionNode
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

        GetPointFromPotentialField(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{ OutputPort<geometry_msgs::Pose>("goalPose") };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
            }

            std::vector<test_cpp::obstacle_point> obs_action = state->obstacles_points.obstacles;//obs_action;//= state->obstacles_points.obstacles;

            test_cpp::obstacle_point goalPF;
            goalPF.pos.x = state->goalPose.position.x;
            goalPF.pos.y = state->goalPose.position.y;
            goalPF.pos.z = state->goalPose.position.z;


            goalPF.field_coeff = 1.5;
            goalPF.min_value = 0.1;
            goalPF.max_value = 5;
            goalPF.min_distance = 0;
            goalPF.max_distance = 1000;
            goalPF.type = "linear_decreasing";


            obs_action.push_back(goalPF);

            geometry_msgs::Pose goal;

            tf::Vector3 posUAV;
            posUAV.setX(state->mavPose.position.x);
            posUAV.setY(state->mavPose.position.y);
            posUAV.setZ(state->mavPose.position.z);

            tf::Vector3 field = obstacle_avoidance::getFieldFromPoints(obs_action, posUAV);
            tf::Vector3 goalVec = obstacle_avoidance::getGoalPointFromField(field);
            goalVec = goalVec + posUAV; // add current position, goalVec is only the direction to move in
            goal.position.x = goalVec.getX();
            goal.position.y = goalVec.getY();
            goal.position.z = goalVec.getZ();
            
            
            //std::cout << "x: " << goal.position.x << " y: " << goal.position.y << " z: " << goal.position.z << std::endl;
            setOutput("goalPose", goal );
            
            return NodeStatus::SUCCESS;
        }
};




// returns failure to stop the bt from quiting when a point is published
class PublishUAVWayPoint : public StatefulActionNode
{
    private: 
        bt_state::mav_state* state;

        BT::NodeStatus logic()
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

            return NodeStatus::RUNNING; // allways return running?
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

        PublishUAVWayPoint(const std::string& name, const NodeConfiguration& config) : StatefulActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{ InputPort<geometry_msgs::Pose>("goalPose") };
        }
        

        NodeStatus onStart()
        {
            return logic();
        }

        NodeStatus onRunning()
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




//simply publish uav pose (state goalPose)
class PublishHomePose : public SyncActionNode
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

        PublishHomePose(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{  };
        }

        NodeStatus tick() override
        {
            if(this->state == nullptr)
            {
                ROS_INFO_STREAM("PublishHomePose, pointer to state = null");
            }


            state->pub_uavWP.publish(state->homePosition);

            return NodeStatus::SUCCESS;
        }
};








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









// statefulactionnodes

class MoveToBTAction : public StatefulActionNode
{
  public:   

    actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
    test_action::MoveToGoal goal_;


    MoveToBTAction(const std::string& name, const NodeConfiguration& config)
      : StatefulActionNode(name, config)
      , actionClient_("mav_moveto_action", true)
    {
        ROS_INFO("Waiting for action server");
        // wait for the action server to start
        actionClient_.waitForServer();
        ROS_INFO("Server started");
        
    }

    static PortsList providedPorts()
    {
        return{ InputPort<geometry_msgs::Pose>("goalPose") };
    }

    NodeStatus onStart()
    {
        //goal_.pose_goal.position.x = 12;
        //goal_.pose_goal.position.y = 0;
        //goal_.pose_goal.position.z = 4;
        
        Optional<geometry_msgs::Pose> msg = getInput<geometry_msgs::Pose>("goalPose");
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

    NodeStatus onRunning()
    {
        Optional<geometry_msgs::Pose> msg = getInput<geometry_msgs::Pose>("goalPose");
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
class TakeOff : public StatefulActionNode
{
  public:   

    actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
    test_action::MoveToGoal goal_;

    bt_state::mav_state* state;
    geometry_msgs::Pose startPose;

    TakeOff(const std::string& name, const NodeConfiguration& config)
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

    static PortsList providedPorts()
    {
        return{};
    }

    NodeStatus onStart()
    {
        //goal_.pose_goal.position.x = 12;
        //goal_.pose_goal.position.y = 0;
        //goal_.pose_goal.position.z = 4;


        startPose = this->state->mavPose;

        goal_.pose_goal = startPose;
        //add one meter to z
        goal_.pose_goal.position.z += 1;
        
        
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

    NodeStatus onRunning()
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




class MoveToHomePoint : public StatefulActionNode
{
    private:
        bt_state::mav_state* state;

    public:

    actionlib::SimpleActionClient<test_action::MoveToAction> actionClient_;
    test_action::MoveToGoal goal_;


    MoveToHomePoint(const std::string& name, const NodeConfiguration& config)
      : StatefulActionNode(name, config)
      , actionClient_("mav_moveto_action", true)
    {
        ROS_INFO("Waiting for action server");
        // wait for the action server to start
        actionClient_.waitForServer();
        ROS_INFO("Server started");
    }

    void init(bt_state::mav_state* statePtr)//(bool* taskIsNewPtr)
    {
        if(statePtr == nullptr)
        {
            //error
        }
        this->state = statePtr;
    }


    static PortsList providedPorts()
    {
        return{ };
    }

    NodeStatus onStart()
    {
        goal_.pose_goal = state->homePosition;
        

        
        actionClient_.sendGoal(goal_);
        actionClient_.waitForResult(ros::Duration(0.01)); //wait for some response, otherwise the onRunning returns fail the first tick (weird?)

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

    NodeStatus onRunning()
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



class DelayOnce : public StatefulActionNode
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

            Optional<double> msg = getInput<double>("delayTime");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error());
            }

            //ROS_INFO_STREAM("Time left" << startTime.toSec() - ros::Time::now().toSec() + msg.value());
            if(ros::Time::now().toSec() > startTime.toSec() + msg.value() || alreadyDelayed)
            {
                alreadyDelayed = true;
                return NodeStatus::SUCCESS;
            }
            else
            {
                return NodeStatus::RUNNING;
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

        DelayOnce(const std::string& name, const NodeConfiguration& config) : StatefulActionNode(name, config)
        {
        }

        static PortsList providedPorts()
        {
            return{ InputPort<double>("delayTime") };
        }

        NodeStatus onStart()
        {
            startTime = ros::Time::now();
            return logic();
        }

        NodeStatus onRunning()
        {
            return logic();
        }

        void onHalted()
        {
            //
        }
};


class CheckHeight : public SyncActionNode
{
    private:    
        geometry_msgs::Pose* posePtr;
    public:



        CheckHeight(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
        {
        }

        // We want this method to be called ONCE and BEFORE the first tick()
        void init(geometry_msgs::Pose* pose )
        {
            posePtr = pose;
        }

        static PortsList providedPorts()
        {
            return {  };
        }

        NodeStatus tick() override
        {

            if(posePtr->position.z < 3.8)
            {
                ROS_INFO("Warning, height = %.2f", posePtr->position.z);
            }

            return NodeStatus::SUCCESS;
        }
};








#endif



