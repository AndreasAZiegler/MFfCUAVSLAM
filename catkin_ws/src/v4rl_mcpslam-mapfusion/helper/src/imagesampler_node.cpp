#include <helper/imagesampler.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "imagesampler_node");

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    img2cloud::imagesampler Sampler(Nh,NhPrivate);

    Sampler.SampleFloorCloud();
    Sampler.SampleLongEdge();
    Sampler.SampleShortEdge();

//    helper::republisher repub(Nh,NhPrivate,OpMode);

//    ros::spin();

    ros::Rate r(1);
    while(ros::ok())
    {
        Sampler.PubFloor();

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
