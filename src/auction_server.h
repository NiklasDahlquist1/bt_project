
#ifndef AUCTION_SERVER_H
#define AUCTION_SERVER_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bt_project/auction_bid.h"
#include "bt_project/auction_auction.h"
#include "bt_project/auction_client.h"
#include "bt_project/auction_winner.h"
#include "bt_project/auction_auctionArray.h"
#include "bt_project/auction_bidArray.h"


#include <vector>
#include <tuple>


#include "optimization_ortools.h"


namespace auction
{
    
    class Auction_server
    {
    private:
        ros::NodeHandle nodeHandle;

        ros::Publisher auction_available_pub;
        ros::Publisher auction_winner_pub;
        ros::Publisher auction_return_pub;

        //multi auction
        //ros::Publisher auction_availableMulti_pub;


        std::vector<bt_project::auction_client> registeredClients; //name, ID
        
        std::vector<bt_project::auction_auction> auctionsQueue; // auction_type, creator_ID, creator_name, end_time, min_price, task_data 
        int auctionIDCounter = 0;

        //std::vector<std::tuple<std::string, float, int>> bidsCurrentAuction; // name, price, ID
        std::vector<bt_project::auction_bidArray> bidsCurrentAuction; // name, price, ID

        std::vector<bt_project::auction_auction> currentAuctions;
        bool currentAuctionRunning = false; //Used for single auction logic
        bool auctionRunning = false; //Used for auction logic

        ros::Time auctionStartTime; 


        //parameters for auctioning
        int bidsThreshold = 5;
        double auctionTimeout = 6;
    public:
        Auction_server(int bidsThreshold, double auctionTimeout);
        ~Auction_server();


        void bidsCB(const bt_project::auction_bidArray& msg); 
        void newClientCB(const bt_project::auction_client& msg); 
        void newAuctionCB(const bt_project::auction_auction& msg); // maybe remove this and only use the array version?
        void declareWinner();
        void publishNewAuction();

        void updateAuctionsLogic();
        void bidsArrayCB(const bt_project::auction_bidArray& msg); 

        void newAuctionArrayCB(const bt_project::auction_auctionArray& msg);
    };
    



    Auction_server::Auction_server(int bidsThreshold, double auctionTimeout)
    {
        this->auction_available_pub = nodeHandle.advertise<bt_project::auction_auctionArray>("auction_available", 1000);
        this->auction_winner_pub = nodeHandle.advertise<bt_project::auction_winner>("auction_winner", 1000);
        this->auction_return_pub = nodeHandle.advertise<bt_project::auction_winner>("auction_return", 1000);
        //ros::Publisher auction_bid_pub = nodeHandle.advertise<bt_project::auction_bid>("auction_bid", 1000);
        //ros::Publisher auction_newClient_pub = nodeHandle.advertise<bt_project::auction_client>("auction_newClient", 1000);

        //this->auction_availableMulti_pub = nodeHandle.advertise<bt_project::auction_auction>("auction_availableMulti", 1000);

        this->bidsThreshold = bidsThreshold;
        this->auctionTimeout = auctionTimeout;
    }
    
    Auction_server::~Auction_server()
    {
    }

    void Auction_server::updateAuctionsLogic()
    {
        if(auctionRunning)
        {
            //int bidsThreshold = 5;
            //double auctionMaxDuration = 6;

            ros::Time currentTime = ros::Time::now();
            if(bidsCurrentAuction.size() >= this->bidsThreshold || auctionStartTime.toSec() + this->auctionTimeout <= currentTime.toSec()) //TODO, make better, less "hardcoded"
            {
                declareWinner();
                auctionRunning = false;
            }
        }
        else if(auctionsQueue.size() >= 1 && auctionRunning != true) // start new auction if there is atleast one task available
        {
            currentAuctions = auctionsQueue;
            auctionsQueue.erase(auctionsQueue.begin(), auctionsQueue.end()); // remove all auctions and add them if they are not sold


            //publish the new active auction
            bt_project::auction_auctionArray auction_msg;
            auction_msg.auctions = currentAuctions;


            //add auction ID (for current auction)
            this->auctionIDCounter++;
            auction_msg.auction_ID = auctionIDCounter;
            //auction_availableMulti_pub.publish(auction_msg);
            auction_available_pub.publish(auction_msg);

            auctionRunning = true;            
            auctionStartTime = ros::Time::now(); //TODO, handle if 0 is returned (as mentioned in documentation)
        }
    }


    
    void Auction_server::bidsCB(const bt_project::auction_bidArray& msg)
    {
        //only accept bid if it has the ID of the current auction
        bool bidOk = false;
        for(bt_project::auction_auction a : currentAuctions)
        {
            if(msg.auction_ID == a.auction_ID)
            {
                bidOk = true;
                this->bidsCurrentAuction.push_back(msg);
                break;
            }
        }

        if(bidOk == false)
        {
            std::cout << "Did not accept that bid, auctionID is not matched" << std::endl;
        }
    }

    void Auction_server::bidsArrayCB(const bt_project::auction_bidArray& msg)
    {
        //only accept bid if it has the ID of any of the current auctions
        bool bidOK = false;

        if(msg.auction_ID == auctionIDCounter)
        {
            bidsCurrentAuction.push_back(msg);
        }
        //ROS_INFO_STREAM(msg.auction_ID << " " << auctionIDCounter);
    }

    void Auction_server::newAuctionCB(const bt_project::auction_auction& msg)
    {
        bt_project::auction_auction auction;
        auction = msg;
        auction.auction_ID = auctionIDCounter; //use "unique" ID for auction, not used, an ID is generated for each "bundle" of auctions available
        //this->auctionIDCounter++;
        

        this->auctionsQueue.push_back(auction);
    }


    void Auction_server::newAuctionArrayCB(const bt_project::auction_auctionArray& msg)
    {
        for(bt_project::auction_auction a : msg.auctions)
        {
            this->auctionsQueue.push_back(a);
        }

 
        std::cout << "Current auctions (size: " << currentAuctions.size() << "): " << std::endl;
        for(bt_project::auction_auction auction : this->currentAuctions)
        {
            std::cout << "Auction name: " << auction.task_name << ", data: " << auction.task_data << std::endl;
        }
        std::cout << "Current auctions queue:" << auctionsQueue.size() << "): " << std::endl;
        for(bt_project::auction_auction auction : this->auctionsQueue)
        {
            std::cout << "Auction name: " << auction.task_name << ", data: " << auction.task_data << std::endl;
        }
    }


    void Auction_server::newClientCB(const bt_project::auction_client& msg)
    {
        this->registeredClients.push_back(msg);


        std::cout << "Current clients:" << std::endl;
        for(bt_project::auction_client client : this->registeredClients)
        {
            std::cout << "name: " << client.name << " ID: " << client.ID << std::endl;
        }
    }


    bool sortByElement(const bt_project::auction_bid& a, const bt_project::auction_bid& b)
    {
        return (a.price < b.price);
        //return (std::get<1>(a) < std::get<1>(b));
    }


    void Auction_server::declareWinner()
    {
        ROS_INFO_STREAM("Auction finnished. Workers: " << bidsCurrentAuction.size() << " tasks: " << currentAuctions.size());


        if (bidsCurrentAuction.size() == 0)
        {
            for(bt_project::auction_auction a : currentAuctions) //return all auctions to the queue TODO ?
            {
                auctionsQueue.push_back(a);
            }
            return; //return if there are no bids, ie. no winners to be calculated
        }
        //create cost matrix
        int workers_num = bidsCurrentAuction.size();
        int tasks_num = currentAuctions.size();
        std::vector<std::vector<double>> costs(workers_num, std::vector<double>(tasks_num)); //TODO check if every bid is from a unique worker
        //TODO, dont assume every robot bids for every task. fixed, every robot should bid for every task, negative if unable to do a task

        

        //maybe not smartest loop, a bit brute force
        for(int i = 0; i < workers_num; ++i)
        {
            for(int j = 0; j < tasks_num; ++j)
            {
                costs[i][j] = bidsCurrentAuction[i].prices[j];
                //ROS_INFO_STREAM(costs[i][j]);
            }
        }

        std::vector<double> winners(workers_num);
        //winners = operations_research::optimizeBids(costs);
        winners = operations_research::taskMatching(costs); //use the matching optimization

        
        std::vector<int> tasksWon; //save tasks won, and delete them from the auctions queue
        for(int i = 0; i < winners.size(); ++i)
        {
            if(winners[i] != -1)
            {
                //publish task for winner
                bt_project::auction_winner winner; //TODO, change msg type of winner to {winner_name, winner_ID, etc, auction_auction auction}

                winner.auction_type = currentAuctions[winners[i]].auction_type;
                winner.task_name = currentAuctions[winners[i]].task_name;
                winner.task_data = currentAuctions[winners[i]].task_data;
                winner.creator_ID = currentAuctions[winners[i]].creator_ID;
                winner.creator_name = currentAuctions[winners[i]].creator_name;

                winner.auction_ID = currentAuctions[winners[i]].auction_ID;
                
                winner.winner_name = bidsCurrentAuction[i].name;
                winner.winner_ID = 0; //maybe use this later


                auction_winner_pub.publish(winner);

                tasksWon.push_back(winners[i]);

                std::cout << "Winner: " << winner.winner_name << ", task data: " << winner.task_data << std::endl;
            }
        }


        //remove sold tasks from queue
        std::sort(tasksWon.begin(), tasksWon.end(), std::greater<int>()); //sort in descending order, allows us to remove the higher indices without messing up the order
        for(int i : tasksWon)
        {
            //auctionsQueue.erase(auctionsQueue.begin() + i); //might be slow, maybe it is worth it to use a list
            //will remove the wrong task if other tasks are added during this auction...
            currentAuctions.erase(currentAuctions.begin() + i);
        }
        
        //add all remaining auctions back to the queue
        for(bt_project::auction_auction a : currentAuctions)
        {
            auctionsQueue.push_back(a);
        }
        //clear all bids
        bidsCurrentAuction.erase(bidsCurrentAuction.begin(), bidsCurrentAuction.end());





        return;
    }


} // namespace auction














#endif






