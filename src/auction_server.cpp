
#include "auction_server.h"


#include "state.h"

#include <random>


auction::Auction_server* auction_server_ptr;
// callbacks        
void bidsCB(const test_cpp::auction_bidArray& msg)
{
    auction_server_ptr->bidsArrayCB(msg);
}
void newClientCB(const test_cpp::auction_client& msg)
{
    auction_server_ptr->newClientCB(msg);
}
void newAuctionCB(const test_cpp::auction_auction& msg)
{
    auction_server_ptr->newAuctionCB(msg);
}
void newAuctionArrayCB(const test_cpp::auction_auctionArray& msg)
{
    auction_server_ptr->newAuctionArrayCB(msg);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "auction_server");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(25);


    // get ros params

    double auctionTimeout = 6; //default values
    int bidsThreshold = 5;

    if(nodeHandle.getParam(ros::this_node::getName() + "/auction_timeout", auctionTimeout))
    {
        auctionTimeout = auctionTimeout;
        std::cout << "Auction server timeout: "<< auctionTimeout << std::endl;
    }
    if(nodeHandle.getParam(ros::this_node::getName() + "/bids_threshold", bidsThreshold))
    {
        auctionTimeout = bidsThreshold;
        std::cout << "Auction bids threshold: "<< bidsThreshold << std::endl;
    }


    //make object available globally for callbacks (maybe not the nicest way)
    auction::Auction_server auction_server(bidsThreshold, auctionTimeout);
    auction_server_ptr = &auction_server;


    ros::Subscriber sub_bid = nodeHandle.subscribe("/auction_bidArray", 1000, bidsCB);
    ros::Subscriber sub_newClient = nodeHandle.subscribe("/auction_newClient", 1000, newClientCB);    
    ros::Subscriber sub_newAuction = nodeHandle.subscribe("/auction_newAuction", 1000, newAuctionCB);

    ros::Subscriber sub_newAuctionArray = nodeHandle.subscribe("/auction_newAuctionArray", 1000, newAuctionArrayCB);




    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> d = t2-t1;

    
    while (ros::ok())
    {
        t1 = std::chrono::high_resolution_clock::now();
        ros::spinOnce(); //check callbacks

        //std::cout << std::fixed << "Current time: " << ros::Time::now().toSec() << std::endl;
        auction_server.updateAuctionsLogic();
        

        t2 = std::chrono::high_resolution_clock::now();
        d = t2-t1;
        if(d.count() > 1)
        {
            //std::cout << d.count() << "ms" << std::endl;
        }

        loop_rate.sleep();
    }


    return 0;
}





