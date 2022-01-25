

#include <ros/ros.h>

#include "bt_project/auction_auctionArray.h"
//#include "bt_project/auction_auctionArray.h"
#include <random>



void test_pub_array(int size)
{
    ros::NodeHandle nodeHandle;
    
    //test publish some random points
    ros::Publisher pub_moveTo = nodeHandle.advertise<bt_project::auction_auctionArray>("/auction_newAuctionArray", 1000, true);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);



/*
    std::cout << "subs: " << pub_moveTo.getNumSubscribers() << std::endl;
    ros::Rate delay(1.5);
    delay.sleep(); //hmm i dont know why this is neccesary
std::cout << "subs: " << pub_moveTo.getNumSubscribers() << std::endl;
*/

    std::vector<bt_project::auction_auction> arr(size);
    bt_project::auction_auctionArray auctionArr;
    for(int i = 0; i < size; ++i)
    {
        bt_project::auction_auction auction;
        auction.auction_type = "single";
        auction.task_name = "moveTo";
        auction.creator_name = "test";
        auction.auction_ID = (int) (dist(e2)*INT_MAX);
        double x = dist(e2) * 10 - 5;
        double y = dist(e2) * 10 - 5;
        double z = dist(e2) * 3.8 + 0.8;
        auction.task_data = std::to_string(x) += ";" + std::to_string(y) += ";" + std::to_string(z) += ";0;0;0;1";

        auctionArr.auctions.push_back(auction);
        //arr[i] = auction;
        //ros::Duration(0.01).sleep();
        std::cout << "Task data: " << auction.task_data << std::endl;
    }
    //auctionArr.auctions = arr;
    int maxWaitItt = 30;
    int itt = 0;
    while (pub_moveTo.getNumSubscribers() < 1)
    {
        if(itt > maxWaitItt)
        {
            continue;
        }
        ++itt;
        ros::WallDuration sleep_t(0.1);    
        sleep_t.sleep();
    }
    ros::Rate delay(1.5);
    delay.sleep();
    std::cout << "subs " << pub_moveTo.getNumSubscribers() << std::endl;
    pub_moveTo.publish(auctionArr);
    delay.sleep();
    delay.sleep();

}

void test_pub_Same_point(int size)
{
    ros::NodeHandle nodeHandle;
    
    //test publish some random points
    ros::Publisher pub_moveTo = nodeHandle.advertise<bt_project::auction_auctionArray>("/auction_newAuctionArray", 1000, true);


    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);


    std::vector<bt_project::auction_auction> arr(size);
    bt_project::auction_auctionArray auctionArr;
    for(int i = 0; i < size; ++i)
    {
        bt_project::auction_auction auction;
        auction.auction_type = "single";
        auction.task_name = "moveTo";
        auction.creator_name = "test";
        auction.auction_ID = (int) (dist(e2)*INT_MAX);
        double x = 4;
        double y = 4;
        double z = 1;
        auction.task_data = std::to_string(x) += ";" + std::to_string(y) += ";" + std::to_string(z) += ";0;0;0;1";

        auctionArr.auctions.push_back(auction);
        //arr[i] = auction;
        //ros::Duration(0.01).sleep();
        std::cout << "Task data: " << auction.task_data << std::endl;
    }
    //auctionArr.auctions = arr;

    
    ros::Rate delay(1.5);
    delay.sleep();
    std::cout << "subs " << pub_moveTo.getNumSubscribers() << std::endl;
    pub_moveTo.publish(auctionArr);
    delay.sleep();
    delay.sleep();    
}



void demo()
{
    ros::NodeHandle nodeHandle;
    ros::Publisher pub_moveTo = nodeHandle.advertise<bt_project::auction_auctionArray>("/auction_newAuctionArray", 1000, true);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);

    bt_project::auction_auctionArray auctionArr;
    bt_project::auction_auction auction;
    auction.auction_type = "single";
    auction.task_name = "moveTo";
    auction.creator_name = "test";

    auction.task_data = std::to_string(0) += ";" + std::to_string(-3) += ";" + std::to_string(2.5) += ";0;0;0;1";
    auction.auction_ID = (int) (dist(e2)*INT_MAX);
    auctionArr.auctions.push_back(auction);

    auction.task_data = std::to_string(1) += ";" + std::to_string(3) += ";" + std::to_string(2.5) += ";0;0;0;1";
    auction.auction_ID = (int) (dist(e2)*INT_MAX);
    auctionArr.auctions.push_back(auction);

    auction.task_data = std::to_string(0) += ";" + std::to_string(0) += ";" + std::to_string(4) += ";0;0;0;1";
    auction.auction_ID = (int) (dist(e2)*INT_MAX);
    auctionArr.auctions.push_back(auction);


    auction.task_name = "explore";
    auction.task_data = "0;0;5;30";
    auction.auction_ID = (int) (dist(e2)*INT_MAX);
    auctionArr.auctions.push_back(auction);


    for(const bt_project::auction_auction& a : auctionArr.auctions)
    {
        std::cout << a.task_data << std::endl;
        //std::cout << a.auction_ID << std::endl;
    }

    // publish
    ros::Rate delay(1.5);
    delay.sleep();
    std::cout << "subs " << pub_moveTo.getNumSubscribers() << std::endl;
    pub_moveTo.publish(auctionArr);
    delay.sleep();
    delay.sleep(); 
}

void publishGridOfPoints()
{
    ros::NodeHandle nodeHandle;
    ros::Publisher pub_moveTo = nodeHandle.advertise<bt_project::auction_auctionArray>("/auction_newAuctionArray", 1000, true);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);

    bt_project::auction_auctionArray auctionArr;
    bt_project::auction_auction auction;
    auction.auction_type = "single";
    auction.task_name = "moveTo";
    auction.creator_name = "test";


    // scatter points over the rectangle (-width, -height),(width, height)
    double rectWidth = 5;
    double RectHeight = 5;
    double height = 2;
    int pointsPerWidth = 9; //points + 1
    int PointsPerHeight = 9; //points + 1
    double x;
    double y;
    double z = height;

    for(int i = 0; i < pointsPerWidth; ++i)
    {
        for(int j = 0; j < PointsPerHeight ; ++j)
        {
            x = 2*rectWidth * (-1 + (2.0*i)/(pointsPerWidth - 1));
            y = 2*RectHeight * (-1 + (2.0*j)/(PointsPerHeight - 1));

            auction.task_data = std::to_string(x) += ";" + std::to_string(y) += ";" + std::to_string(z) += ";0;0;0;1";
            auction.auction_ID = (int) (dist(e2)*INT_MAX);
            auctionArr.auctions.push_back(auction);

            std::cout << "Point: (" << x << ", " << y << ", " << z << ")" << std::endl;
        }
    }
    std::cout << "number of points: " << auctionArr.auctions.size() << std::endl;







    // publish
    ros::Rate delay(1.5);
    delay.sleep();
    std::cout << "subs " << pub_moveTo.getNumSubscribers() << std::endl;
    pub_moveTo.publish(auctionArr);
    delay.sleep();
    delay.sleep(); 
}

void explore()
{
    ros::NodeHandle nodeHandle;
    ros::Publisher pub_moveTo = nodeHandle.advertise<bt_project::auction_auctionArray>("/auction_newAuctionArray", 1000, true);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(0, 1);

    bt_project::auction_auctionArray auctionArr;
    bt_project::auction_auction auction;
    auction.auction_type = "single";
    auction.task_name = "explore";
    auction.creator_name = "test";

    auction.task_data = "0;0;5;30";
    auction.auction_ID = (int) (dist(e2)*INT_MAX);
    auctionArr.auctions.push_back(auction);



    for(const bt_project::auction_auction& a : auctionArr.auctions)
    {
        std::cout << a.task_data << std::endl;
        //std::cout << a.auction_ID << std::endl;
    }

    // publish
    ros::Rate delay(1.5);
    delay.sleep();
    std::cout << "subs " << pub_moveTo.getNumSubscribers() << std::endl;
    pub_moveTo.publish(auctionArr);
    delay.sleep();
    delay.sleep(); 
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_auctions_test");
    std::cout << ros::ok() << std::endl;

    if(argc == 2)
    {

        std::string s = argv[1];
        if (s == "demo")
        {
            demo();
        }
        if (s == "grid")
        {
            publishGridOfPoints();
        }
        else if (s == "explore")
        {
            explore();
        }
        else
        {
            int size = std::stoi(argv[1], nullptr);
            test_pub_array(size);
        }
    }
    else if(argc == 1) //no input, standard size
    {
        //test_pub_auctions();
        test_pub_array(20);
    }
    else if(argc == 3)
    {
        std::string s = argv[2];
        if (s == "same")
        {
            int size = std::stoi(argv[1], nullptr);
            test_pub_Same_point(size);
        }


    }
    ros::spinOnce();
    //std::cout << argc << " " << argv[1] << std::endl; 

    return 0;
}