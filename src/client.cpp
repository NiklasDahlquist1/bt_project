
#include "auction_client.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "actions.h"
//#include "bt_gen.h"
#include "bt_gen_new.h"
#include "bt_init.h"

#include "nav_msgs/Odometry.h"



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

/*void mavPoseCB(const geometry_msgs::Pose& msg)
{
    auction_client_ptr->poseCB(msg);
}*/
  // use the nav_msgs::odometry?
void mavPoseCB(const nav_msgs::Odometry& msg)
{
    auction_client_ptr->poseCB(msg.pose.pose);
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
    ros::Subscriber sub_pose = nodeHandle.subscribe("/odometry", 1000, mavPoseCB);

    
    // just for plotting now
    ros::Publisher task_aborted = nodeHandle.advertise<test_cpp::auction_winner>("plot/task_aborted", 100);
    ros::Publisher task_succeeded = nodeHandle.advertise<test_cpp::auction_winner>("plot/task_succeeded", 100);


    ROS_INFO_STREAM("INIT CLIENT " << ros::this_node::getName());

    // get ros params

    std::string pathToFiles = ros::package::getPath("bt_project");
    pathToFiles.append("/files/");
    std::string behavior_available = pathToFiles + "behavior_available.csv"; //"standard" values
    std::string behavior_definition = pathToFiles + "behavior_definition.csv";


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







    // mav state setup
    bt_state::mav_state state;
    state.nodeHandle = nodeHandle;

    bt_state::initMAVStateCBPtr(state);
    ros::Subscriber sub_state_pose = nodeHandle.subscribe("/odometry", 1000, bt_state::mavPoseCB);
    ros::Subscriber sub_state_obstacle_points = nodeHandle.subscribe("/obstacles_points", 1000, bt_state::obstaclePointsCB);
    ros::Subscriber sub_motor_failure = nodeHandle.subscribe("/motor_fail", 100, bt_state::motorFailCB);



    // behavior library setup
    std::vector<bt::behavior_test> behaviorLibrary; 
    behaviorLibrary = bt::readBehaviorLibraryCSV(behavior_available, behavior_definition);



    //BT setup
    BT::BehaviorTreeFactory factory;
    bt::initFactory(factory);




    BT::Tree tree;
    BT::Tree tree_noTask;



    //bt::getNewTree(tree, factory, task, behaviorLibrary);
    //bt::getNewTree(tree_noTask, factory, task, behaviorLibrary);



    //init all nodes
    bt::initNodes(tree, &state);
    bt::initNodes(tree_noTask, &state);



    




    //Variables for controlling BT
    bool runningBT = false;

    bool runningNoTaskBT = false;
    bool hasNoTaskBT = false;


    ros::Duration wait(1);
    wait.sleep();


    ros::Time noTask = ros::Time::now();
    BT::NodeStatus status;
    while (ros::ok())
    {
        //ROS_LOG_STREAM(ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, "test loh");
        ros::spinOnce(); //check callbacks




        //auctioning logic
        if (state.canBidForNewTask == true) // not super beautiful, i dont really like this way of setting variables from anywhere. maybe make client bid with only negative values? makes sure to not affect the bid threshold
        {
            auction_client_mav.clientLogic();
        }
        if(auction_client_mav.switchTask) // new task is won
        {
            ROS_INFO_STREAM( "Win detected, switching goal task");
            auction_client_mav.switchTask = false;

            //state.goalPose = auction_client_mav.getGoalPose();
            state.taskIsNew = true;
            //goalPose = auction_client_mav.getGoalPose();
            //taskIsNewBT = true;

            //reload BT TODO should depend on what the task is
            tree.haltTree();
            //gen_bt::loadNewBT(tree, factory, auction_client_mav.getCurrentTask());
            //initActions(tree, &state);
            bt::getNewTree(tree, factory, auction_client_mav.getCurrentTask(), behaviorLibrary);
            bt::initNodes(tree, &state);



            runningBT = true;
            hasNoTaskBT = false; // BT is reset to task
            state.taskIsActive = true; // used for "detecting" failed tasks, remove?
        }



        //behavior tree logic

        if(runningBT)
        {
            status = tree.tickRoot(); //tick behavior tree
            //ROS_INFO_STREAM(status);

            if(status == BT::NodeStatus::SUCCESS)
            {
                ROS_INFO_STREAM("Task finnished, position: " << state.mavPose.position);
                task_succeeded.publish(auction_client_mav.getCurrentTask()); // plot

                runningBT = false;
                auction_client_mav.setTaskNotActive(); //allow for bidding on new auction
                //TODO reload "no-task" BT
                noTask = ros::Time::now();
                state.taskIsActive = false;
            }
            else if(status == BT::NodeStatus::FAILURE)
            {
                ROS_INFO_STREAM("Behavior tree fail, maybe sell task?");
                task_aborted.publish(auction_client_mav.getCurrentTask()); // plot


                runningBT = false;
                auction_client_mav.sellCurrentTask();
                auction_client_mav.setTaskNotActive(); //allow for bidding on new auction
               //TODO reload "no-task" BT
                noTask = ros::Time::now();// - ros::Duration(7);
                state.taskIsActive = false;
            }
        }
        else //if(ros::Time::now().toSec() >= noTask.toSec() + 8) // TODO, remove time, and make the default BT run directly. I guess the delay behavior should be included in the BT
        {
            if(hasNoTaskBT == false)
            {
                std::string goalHome = std::to_string(state.homePosition.position.x) += ";" + 
                                std::to_string(state.homePosition.position.y) += ";" + 
                                std::to_string(state.homePosition.position.z) ;//+= ";0;0;0;1";


                tree_noTask.haltTree();

                test_cpp::auction_winner t;
                //t.task_name = "noTaskBT";
                t.task_name = "land";
                t.task_data = goalHome;


                bt::getNewTree(tree_noTask, factory, t, behaviorLibrary);
                bt::initNodes(tree_noTask, &state);

                hasNoTaskBT = true;
                runningNoTaskBT = true;
            }
            if(runningNoTaskBT)
            {
                status = tree_noTask.tickRoot(); 

                if(status == NodeStatus::SUCCESS) // stop BT after completion??
                {
                    //runningNoTaskBT = false;
                }
            }
        }




        loop_rate.sleep();
    }


    return 0;
}





