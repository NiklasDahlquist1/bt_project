
#ifndef AUCTION_CLIENT_H
#define AUCTION_CLIENT_H


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "test_cpp/auction_bid.h"
#include "test_cpp/auction_auction.h"
#include "test_cpp/auction_client.h"
#include "test_cpp/auction_winner.h"
#include "test_cpp/auction_bidArray.h"
#include "test_cpp/auction_auctionArray.h"

#include <vector>
#include <tuple>
#include <math.h>
#include <cmath>

namespace auction
{
    
    class Auction_client
    {
    private:
        bool checkIfIdNotIgnored(int ID);
        void ignoreCurrentTask(); // add current task ID to list of ignore IDs
    protected:
        ros::NodeHandle nodeHandle;

        ros::Publisher auction_bid_pub;
        ros::Publisher auction_taskAvailable_pub;
        


        std::vector<int> ignoreIDs;
    
        //std::vector<std::tuple<std::string, float, std::string>> tasksWon; //task_type, price payed, task_data 
        std::string name;

        test_cpp::auction_auctionArray availableAuction;

        test_cpp::auction_winner currentTask; // 
        bool currentTaskActive = false; //do not bid for new task while working on this
        bool auctionAvailableHasBeenSet = false;
        bool bidSent = false; //check to not spam bids

    public:
        bool switchTask = false;

        Auction_client(std::string name);
        ~Auction_client();

        //void registerAsClient();
        void winnerAuctionCB();
        //void returningAuctionCB();
        virtual double calculateBid(test_cpp::auction_auction auction);
        void sendBid(std::vector<double> prices);
        //void checkIfWinner(); // handled in the callback instead
        void clientLogic();


        void checkAvailableAuctionCB(const test_cpp::auction_auctionArray& msg);
        void auctionWinnerCB(const test_cpp::auction_winner& msg);
        void auctionNotWonCB(const test_cpp::auction_winner& msg);
        void setTaskNotActive(); //call when done with task to look for new task

        void sellCurrentTask(); // put current task up for auctioning again, remember to also sets taskActive to false

        test_cpp::auction_winner getCurrentTask();
    };

    class Auction_client_mav : public Auction_client
    {
        private:

        geometry_msgs::Pose currentPose; // current pose needed to calculate distance for cost function, could be implemented as a service i.e asked for when needed?


        public:
        using Auction_client::Auction_client;
        //Auction_client_mav(std::string name);
        //~Auction_client_mav();

        void poseCB(const geometry_msgs::Pose& msg);
        double calculateBid(test_cpp::auction_auction auction);

        //geometry_msgs::Pose getGoalPose();
    };

    //TODO, implement auction client for walker
    class Auction_client_walker : public Auction_client
    {
        private:

        geometry_msgs::Pose currentPose;


        public:
        using Auction_client::Auction_client;
        //Auction_client_mav(std::string name);
        //~Auction_client_mav();

        void poseCB(const geometry_msgs::Pose& msg);
        double calculateBid(test_cpp::auction_auction auction);
    };










// functions 

    void Auction_client::sellCurrentTask()
    {
        test_cpp::auction_auction a;
        test_cpp::auction_auctionArray ar; //TODO, how to know all original data from won task (min/max price, ??) maybe use same msg type for auction and winner?

        a.task_name = this->currentTask.task_name;
        a.task_data = this->currentTask.task_data;
        a.creator_name = this->name;
        a.auction_type = this->currentTask.auction_type;
        a.auction_ID = this->currentTask.auction_ID;
        ar.auctions.push_back(a);

        this->auction_taskAvailable_pub.publish(ar);
        //setTaskNotActive();
        ignoreCurrentTask();
    }

    void Auction_client::ignoreCurrentTask()
    {
        this->ignoreIDs.push_back(this->currentTask.auction_ID);
    }

    // return true if ID is OK
    bool Auction_client::checkIfIdNotIgnored(int ID)
    {
        bool b = true;
        for(int id : this->ignoreIDs)
        {
            if(id == ID)
            {
                b = false;
                break;
            }
        }
        return b;
    }



    test_cpp::auction_winner Auction_client::getCurrentTask()
    {
        return currentTask;
    }
 
    void Auction_client::setTaskNotActive()
    {
        this->currentTaskActive = false;
    }

    void Auction_client::sendBid(std::vector<double> prices)
    {
        test_cpp::auction_bidArray bids;

        bids.name = this->name;
        bids.prices = prices;
        bids.auction_ID = availableAuction.auction_ID;// make sure that the bids received by the auction server is connected to the tasks at this client

        this->auction_bid_pub.publish(bids);
    }

    double Auction_client::calculateBid(test_cpp::auction_auction auction)
    {
        return -1; 
    }
    
    void Auction_client::clientLogic()
    {
        //only bid if no current task is active
        if(currentTaskActive == false)
        {

            //make bid if auction available
            if(auctionAvailableHasBeenSet == true)
            {
                if(this->bidSent == false)
                {
                    bool hasCosts = false;
                    std::vector<double> costs;
                    for(test_cpp::auction_auction auction : availableAuction.auctions)
                    {
                        //TODO, add check to make sure that this task is not included in ignore lists
                        if(checkIfIdNotIgnored(auction.auction_ID))
                        {
                            double v = calculateBid(auction);
                            costs.push_back(v);

                            hasCosts = true; // dont send the bid if all tasks are in ignore list
                        }
                        else
                        {
                            costs.push_back(-1);
                        }
                    }

                    if(hasCosts)
                    {
                        // remove TODO
                        std::cout << "Cost: ";
                        for(double v : costs)
                        {
                            std::cout << v << " ";
                        }
                        std::cout << std::endl;

                        //send bid
                        sendBid(costs);
                        this->bidSent = true;
                        ROS_INFO_STREAM(this->name << " sent bid (" << costs.size() << ")");                     
                    }
                    else
                    {
                        this->bidSent = true; //to not spam bids
                        ROS_INFO_STREAM(this->name << " Did not bid, all tasks are ignored");
                    }
                }
            }
        }
    }

    void Auction_client::checkAvailableAuctionCB(const test_cpp::auction_auctionArray& msg)
    {
        this->availableAuction = msg;

        // a auction is now available
        this->auctionAvailableHasBeenSet = true;
        this->bidSent = false;
    }

    void Auction_client::auctionWinnerCB(const test_cpp::auction_winner& msg)
    {
        //check if the name of the winner is the same
        if(this->name == msg.winner_name && this->currentTaskActive == false)
        {
            this->currentTask = msg;
            //set this task as active
            this->currentTaskActive = true;


            //TODO (do nicer) add flag to allow change of task to be detected, could be polled from outside and handle task switching there
            this->switchTask = true;

            ROS_INFO_STREAM("Received new task of type: " << msg.auction_type);
        }
        else
        {
            // Wow, this is not good. winner of two tasks, maybe return it to the auctioning server? TODO, something
        }
    }

    void Auction_client::auctionNotWonCB(const test_cpp::auction_winner& msg)
    {
        //not used for now, should check if this client created the auction and nobody accepted the task
    }


    Auction_client::Auction_client(std::string name)
    {
        this->name = name;

        this->auction_bid_pub = nodeHandle.advertise<test_cpp::auction_bidArray>("auction_bidArray", 1000);
        this->auction_taskAvailable_pub = nodeHandle.advertise<test_cpp::auction_auctionArray>("auction_newAuctionArray", 1000);
    }
    
    Auction_client::~Auction_client()
    {
    }








    
    std::vector<std::string> splitString (std::string s, std::string delimiter) 
    {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
            token = s.substr (pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back (token);
        }

        res.push_back (s.substr (pos_start));
        return res;
    }











    void Auction_client_mav::poseCB(const geometry_msgs::Pose& msg)
    {
        this->currentPose = msg;
    }

    /*geometry_msgs::Pose Auction_client_mav::getGoalPose()
    {
        // data comes as "X;Y;Z;x;y;z;w"
        auto parts = splitString(currentTask.task_data, ";");
        if (parts.size() != 7)
        {
            ROS_INFO("Error, not correct auction data for setting goalPose"); //TODO handle that this auction was not correct
        }
        else
        {
            geometry_msgs::Pose goalPose;
            goalPose.position.x    = std::stod(parts[0]);
            goalPose.position.y    = std::stod(parts[1]);
            goalPose.position.z    = std::stod(parts[2]);
            goalPose.orientation.x = std::stod(parts[3]);
            goalPose.orientation.y = std::stod(parts[4]);
            goalPose.orientation.z = std::stod(parts[5]);
            goalPose.orientation.w = std::stod(parts[6]);


            ROS_INFO_STREAM("goal: x: " << goalPose.position.x << ", y: " << goalPose.position.y << ", z: " << goalPose.position.z);
            return goalPose;
        }
    }*/


    double Auction_client_mav::calculateBid(test_cpp::auction_auction auction) 
    {
        if(auction.task_name == "moveTo")
        {
            //calculate distance to goal and use that as bid

            // data comes as "X;Y;Z;x;y;z;w"
            auto parts = splitString(auction.task_data, ";");
            if (parts.size() != 7)
            {
                ROS_INFO("Error, not correct auction data"); //TODO handle that this auction was not correct
            }
            else
            {
                geometry_msgs::Pose goalPose;
                goalPose.position.x    = std::stod(parts[0]);
                goalPose.position.y    = std::stod(parts[1]);
                goalPose.position.z    = std::stod(parts[2]);
                goalPose.orientation.x = std::stod(parts[3]);
                goalPose.orientation.y = std::stod(parts[4]);
                goalPose.orientation.z = std::stod(parts[5]);
                goalPose.orientation.w = std::stod(parts[6]);

                //could return dist^2 but who needs that kind of speed (optimization)
                return sqrt( pow(goalPose.position.x - currentPose.position.x, 2) + pow(goalPose.position.y - currentPose.position.y, 2) + pow(goalPose.position.z - currentPose.position.z, 2) );
            }
        }
        if(auction.task_name == "explore")
        {
            //calculate distance to goal and use that as bid

            // data comes as "X;Y;Z;time"
            auto parts = splitString(auction.task_data, ";");
            if (parts.size() != 4)
            {
                ROS_INFO("Error, not correct auction data"); //TODO handle that this auction was not correct
            }
            else
            {
                geometry_msgs::Point goalPose;
                
                goalPose.x    = std::stod(parts[0]);
                goalPose.y    = std::stod(parts[1]);
                goalPose.z    = std::stod(parts[2]);
                double time = std::stod(parts[3]);

                double costPerSecond = 0.1;

                //could return dist^2 but who needs that kind of speed (optimization)
                return sqrt( pow(goalPose.x - currentPose.position.x, 2) + pow(goalPose.y - currentPose.position.y, 2) + pow(goalPose.z - currentPose.position.z, 2) ) + time * costPerSecond;
            }
        }
        else if(auction.task_name == "something")
        {
            return -1; // add more task cost functions
        }
        else
        {
            return -1;
                    
        }

    }








    double Auction_client_walker::calculateBid(test_cpp::auction_auction auction) 
    {
        if(auction.task_name == "walkTo") // TODO, implement all these cost functions
        {
            //calculate distance to goal and use that as bid

            // data comes as "X;Y;Z;x;y;z;w"
            auto parts = splitString(auction.task_data, ";");
            if (parts.size() != 7)
            {
                ROS_INFO("Error, not correct auction data"); //TODO handle that this auction was not correct
            }
            else
            {
                geometry_msgs::Pose goalPose;
                goalPose.position.x    = std::stod(parts[0]);
                goalPose.position.y    = std::stod(parts[1]);
                goalPose.position.z    = std::stod(parts[2]);
                goalPose.orientation.x = std::stod(parts[3]);
                goalPose.orientation.y = std::stod(parts[4]);
                goalPose.orientation.z = std::stod(parts[5]);
                goalPose.orientation.w = std::stod(parts[6]);

                //could return dist^2 but who needs that kind of speed (optimization)
                return sqrt( pow(goalPose.position.x - currentPose.position.x, 2) + pow(goalPose.position.y - currentPose.position.y, 2) + pow(goalPose.position.z - currentPose.position.z, 2) );
            }
        }
        else if(auction.task_name == "something")
        {
            return -1; // add more task cost functions
        }
        else
        {
            return -1;
                    
        }

    }

} // namespace auction



#endif





