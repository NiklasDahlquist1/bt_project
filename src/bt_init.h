



#ifndef BT_INIT_H
#define BT_INIT_H


#include "ros/ros.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <vector>
#include <tuple>

#include "behaviors.h"


namespace bt
{
    void initFactory(BT::BehaviorTreeFactory& factory);
    void initNodes(BT::Tree& tree, bt_state::mav_state* statePtr);
    void loadNewTree();
    //std::vector<behavior_test> initBehaviorLibrary(std::string fileBehaviorsAvailable, std::string fileBehaviorDefinition);

} //



namespace bt
{
    void initFactory(BT::BehaviorTreeFactory& factory)
    {
        factory.registerNodeType<behaviors::MoveToBTAction>("MoveToBTAction");
        factory.registerNodeType<behaviors::IsFlying>("IsFlying");
        factory.registerNodeType<behaviors::TakeOff>("TakeOff");
        factory.registerNodeType<behaviors::UAVAtPoint>("UAVAtPoint");
        //factory.registerNodeType<behaviors::UAVAtHomePoint>("UAVAtHomePoint");
        //factory.registerNodeType<behaviors::MoveToHomePoint>("MoveToHomePoint");
        factory.registerNodeType<behaviors::GetPointFromPotentialField>("GetPointFromPotentialField");
        factory.registerNodeType<behaviors::PublishUAVWayPoint>("PublishUAVWayPoint");
        factory.registerNodeType<behaviors::HasCollisionFreeWP>("HasCollisionFreeWP");
        //factory.registerNodeType<behaviors::PublishHomePose>("PublishHomePose");
        factory.registerNodeType<behaviors::SafetyChecks>("SafetyChecks");
        factory.registerNodeType<behaviors::DelayOnce>("DelayOnce");

        factory.registerNodeType<behaviors::CheckTaskFailUAVAtPoint>("CheckTaskFailUAVAtPoint");
        factory.registerNodeType<behaviors::CheckTaskFailExplore>("CheckTaskFailExplore");

        factory.registerNodeType<behaviors::ExploredSomeTime>("ExploredSomeTime");
        factory.registerNodeType<behaviors::HasRandomPoint>("HasRandomPoint");
        factory.registerNodeType<behaviors::GetNewRandomPoint>("GetNewRandomPoint");
        factory.registerNodeType<behaviors::Explore>("Explore");



        factory.registerNodeType<behaviors::LandedAtPoint>("LandedAtPoint");
        factory.registerNodeType<behaviors::LandAtPoint>("LandAtPoint");
        factory.registerNodeType<behaviors::UAVAtPointOnce>("UAVAtPointOnce");


        return;
    }




    void initNodes(BT::Tree& tree, bt_state::mav_state* statePtr)//bool* taskIsNewBT, geometry_msgs::Pose* goalPose)
    {
        // Iterate through all the nodes and call init() if it is an Action_B
        for( auto& node: tree.nodes )
        {
            // Not a typo: it is "=", not "=="
            if(auto isFlying = dynamic_cast<behaviors::IsFlying*>( node.get()))
            {
                isFlying->init(statePtr);
            }
            else if(auto takeOff = dynamic_cast<behaviors::TakeOff*>( node.get()))
            {
                takeOff->init(statePtr);
            }
            else if(auto uavAtPoint = dynamic_cast<behaviors::UAVAtPoint*>( node.get()))
            {
                uavAtPoint->init(statePtr);
            }
            /*else if(auto uavAtHomePoint = dynamic_cast<behaviors::UAVAtHomePoint*>( node.get()))
            {
                uavAtHomePoint->init(statePtr);
            }
            else if(auto moveToHomePoint = dynamic_cast<behaviors::MoveToHomePoint*>( node.get()))
            {
                moveToHomePoint->init(statePtr);
            }
            else if(auto publishHomePose = dynamic_cast<behaviors::PublishHomePose*>( node.get()))
            {
                publishHomePose->init(statePtr);
            } */  
            else if(auto getPointFromPotentialField = dynamic_cast<behaviors::GetPointFromPotentialField*>( node.get()))
            {
                getPointFromPotentialField->init(statePtr);
            }
            else if(auto publishUAVWayPoint = dynamic_cast<behaviors::PublishUAVWayPoint*>( node.get()))
            {
                publishUAVWayPoint->init(statePtr);
            }        
            else if(auto hasCollisionFreeWP = dynamic_cast<behaviors::HasCollisionFreeWP*>( node.get()))
            {
                hasCollisionFreeWP->init(statePtr);
            }        

            else if(auto checkTaskFailUAVAtPoint = dynamic_cast<behaviors::CheckTaskFailUAVAtPoint*>( node.get()))
            {
                checkTaskFailUAVAtPoint->init(statePtr);
            }        
            else if(auto safetyChecks = dynamic_cast<behaviors::SafetyChecks*>( node.get()))
            {
                safetyChecks->init(statePtr);
            }        
            else if(auto delayOnce = dynamic_cast<behaviors::DelayOnce*>( node.get()))
            {
                delayOnce->init(statePtr);
            }            
            else if(auto checkTaskFailExplore = dynamic_cast<behaviors::CheckTaskFailExplore*>( node.get()))
            {
                checkTaskFailExplore->init(statePtr);
            }
            else if(auto exploredSomeTime = dynamic_cast<behaviors::ExploredSomeTime*>( node.get()))
            {
                exploredSomeTime->init(statePtr);
            }
            else if(auto hasRandomPoint = dynamic_cast<behaviors::HasRandomPoint*>( node.get()))
            {
                hasRandomPoint->init(statePtr);
            }
            else if(auto getNewRandomPoint = dynamic_cast<behaviors::GetNewRandomPoint*>( node.get()))
            {
                getNewRandomPoint->init(statePtr);
            }            
            else if(auto explore = dynamic_cast<behaviors::Explore*>( node.get()))
            {
                explore->init(statePtr);
            }           
            else if(auto landedAtPoint = dynamic_cast<behaviors::LandedAtPoint*>( node.get()))
            {
                landedAtPoint->init(statePtr);
            }            
            else if(auto landAtPoint = dynamic_cast<behaviors::LandAtPoint*>( node.get()))
            {
                landAtPoint->init(statePtr);
            }
            else if(auto uAVAtPointOnce = dynamic_cast<behaviors::UAVAtPointOnce*>( node.get()))
            {
                uAVAtPointOnce->init(statePtr);
            }
        }
    }

    


} // emd





#endif






