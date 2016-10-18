#include <macslam/client/ClientSystem.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "MAC-SLAM client node");

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun macslam clientnode path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    boost::shared_ptr<macslam::ClientSystem> pCSys{new macslam::ClientSystem(Nh,NhPrivate,argv[1],argv[2])};
//    pCSys->InitializeThreads();

    ROS_INFO("started MCP-SLAM client node...");

    //ros::spin();

    int ClientRate;
    NhPrivate.param("ClientRate",ClientRate,1000);
    cout << "ClientRate: " << ClientRate << endl;

    ros::Rate r(ClientRate);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
