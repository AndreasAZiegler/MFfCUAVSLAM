#include <helper/viewer.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "macslam_viewer_node");

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    mac_viewer::viewer View(Nh,NhPrivate);

//    ros::spin();

    ros::MultiThreadedSpinner MSpin;
    MSpin.spin();

//    ros::Rate r(1);
//    while(ros::ok())
//    {

//        ros::spinOnce();
//        r.sleep();
//    }


    return 0;
}
