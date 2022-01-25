
#include "auction_client.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "bt_gen_new.h"


#include "actions.h"
//#include "bt_gen.h"
#include <ros/package.h>


#include "bt_init.h"


#include <fstream>
#include <iostream>
#include <string>
#include <sstream>




auction::Auction_client_mav* auction_client_ptr;

// callbacks        
void checkAvailableAuctionCB(const test_cpp::auction_auctionArray& msg)
{
    auction_client_ptr->checkAvailableAuctionCB(msg);
}
void AuctionWinnerCB(const test_cpp::auction_winner& msg)
{
    auction_client_ptr->auctionWinnerCB(msg);
}
void AuctionNotWonCB(const test_cpp::auction_winner& msg)
{
    //not used for now, should check if this client created the auction and nobody accepted the task
    auction_client_ptr->auctionNotWonCB(msg);
}

void mavPoseCB(const geometry_msgs::Pose& msg)
{
    auction_client_ptr->poseCB(msg);
}













int main(int argc, char **argv)
{
    ros::init(argc, argv, "auction_client");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(100);

    //make object available globally for callbacks (maybe not the nicest way)
    auction::Auction_client_mav auction_client_mav(ros::this_node::getName());
    auction_client_ptr = &auction_client_mav;

    ros::Subscriber sub_availableAuction = nodeHandle.subscribe("/auction_available", 1000, checkAvailableAuctionCB);
    ros::Subscriber sub_newClient = nodeHandle.subscribe("/auction_winner", 1000, AuctionWinnerCB);    
    ros::Subscriber sub_newAuction = nodeHandle.subscribe("/auction_return", 1000, AuctionNotWonCB);
    ros::Subscriber sub_pose = nodeHandle.subscribe("/pose", 1000, mavPoseCB);

    
    // just for plotting now
    ros::Publisher task_aborted = nodeHandle.advertise<test_cpp::auction_winner>("plot/task_aborted", 100);
    ros::Publisher task_succeeded = nodeHandle.advertise<test_cpp::auction_winner>("plot/task_succeeded", 100);


    ROS_INFO_STREAM("INIT CLIENT " << ros::this_node::getName());




    // get ros params

    std::string pathToFiles = ros::package::getPath("bt_project");
    pathToFiles.append("/files/");
    std::string behavior_available = pathToFiles + "behavior_available.csv"; //"standard" values
    std::string behavior_definition = pathToFiles + "behavior_definition.csv";
    std::string task_name = "moveTo";
    std::string task_data = "0;0;5;0;0;0;1";


    std::string param;
    //if(ros::param::get("behavior_available", param))
    if(nodeHandle.getParam(ros::this_node::getName() + "/behavior_available", param))
    {
        behavior_available = pathToFiles + param;
        std::cout << "Using behavior_available: "<< behavior_available << std::endl;
    }
    if(ros::param::get(ros::this_node::getName() + "/behavior_definition", param))
    {
        behavior_definition = pathToFiles + param;
        std::cout << "Using behavior_definition: "<< behavior_definition << std::endl;
    }
    if(ros::param::get(ros::this_node::getName() + "/task_name", param))
    {
        task_name = param;
        std::cout << "Using task name: "<< task_name << std::endl;
    }
    if(ros::param::get(ros::this_node::getName() + "/task_data", param))
    {
        task_data = param;
        std::cout << "Using task data: "<< task_data << std::endl;
    }




    // mav state setup
    bt_state::mav_state state;
    state.nodeHandle = nodeHandle;

    bt_state::initMAVStateCBPtr(state);
    ros::Subscriber sub_state_pose = nodeHandle.subscribe("/pose", 1000, bt_state::mavPoseCB);
    ros::Subscriber sub_state_obstacle_points = nodeHandle.subscribe("/obstacles_points", 1000, bt_state::obstaclePointsCB);



    // behavior library setup
    std::vector<bt::behavior_test> behaviorLibrary; 
    behaviorLibrary = bt::readBehaviorLibraryCSV(behavior_available, behavior_definition);



    //BT setup
    BT::BehaviorTreeFactory factory;
    bt::initFactory(factory);



    BT::Tree tree;


    // load tree
    test_cpp::auction_winner task;
    task.task_name = task_name;
    task.task_data = task_data;
    //gen_bt::loadNewBT(tree, factory, task);
/*
    // create root node for graph TODO, maybe move all this into a function such as GetNewBT()
    bt::Node_bt root = bt::Node_bt("BehaviorTree");
    root.addAttribute("ID", "BehaviorTree", "");

    //generate task dependent first nodes
    bt::Node_bt* startCondition = bt::getStartCondition(&root, task, behaviorLibrary); // get start condition based on specific task, and link to root node
    bt::generateTreeGraph(startCondition, behaviorLibrary); // expands tree from start condition
    std::string xmlBT = bt::getXMLFromGraph(&root);
    std::cout << xmlBT << std::endl;

    tree = factory.createTreeFromText(xmlBT);
*/

    bt::getNewTree(tree, factory, task, behaviorLibrary);



    //init all nodes
    bt::initNodes(tree, &state);



    //BT::printTreeRecursively(tree.rootNode());


    PublisherZMQ publisher_zmq(tree);



    //Variables for controlling BT
    //






    ros::Duration wait(1);
    wait.sleep();


    BT::NodeStatus status;

    while (ros::ok())
    {
        
        //ROS_LOG_STREAM(ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, "test loh");
        ros::spinOnce(); //check callbacks




        status = tree.tickRoot(); //tick behavior tree
        //std::cout << status << std::endl;

        std::cout << status << std::endl;


        loop_rate.sleep();
    }


    return 0;
}





