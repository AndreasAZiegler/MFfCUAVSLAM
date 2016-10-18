#include <macslam/server/ServerSystem.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "MAC-SLAM server node");

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun macslam clientnode path_to_vocabulary" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    boost::shared_ptr<macslam::ServerSystem> pSSys{new macslam::ServerSystem(Nh,NhPrivate,argv[1])};
    pSSys->InitializeClients();
//    pSSys->InitializeMapMatcher();


    ROS_INFO("started MCP-SLAM server node...");

//    ros::spin();

    int ServerRate;
    NhPrivate.param("ServerRate",ServerRate,1000);
    cout << "ServerRate: " << ServerRate << endl;

//    ros::AsyncSpinner ASpin(4);
//    ASpin.start();

    ros::MultiThreadedSpinner MSpin(2);

            MSpin.spin();

    ros::waitForShutdown();


//    ros::Rate r(ServerRate);
//    while(ros::ok())
//    {
//        ros::spinOnce();
//        r.sleep();
//    }


    return 0;
}
