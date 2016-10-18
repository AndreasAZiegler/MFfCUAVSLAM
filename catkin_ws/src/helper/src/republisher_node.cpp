#include <helper/republisher.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "helper_node");

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    std::string OpMode;
    NhPrivate.param("OpMode",OpMode,std::string("nospec"));
    int sleeptime;
    NhPrivate.param("sleeptime",sleeptime,1000000);

    helper::republisher repub(Nh,NhPrivate,OpMode);
    helper::boundaries bounds(Nh,NhPrivate,sleeptime);

    std::thread tBounds(&helper::boundaries::run,bounds);

//    ros::spin();

    ros::Rate r(100);
    while(ros::ok())
    {
        repub.PubMesh();

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}

