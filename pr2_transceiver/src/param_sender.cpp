#include <ros/ros.h>
#include <std_msgs/Int32.h>

const int CNTRL_FREQ = 1;

class ParamsSender
{

  public:

    // Node Handlers
    ros::NodeHandle nh_;
    //Publishers
    ros::Publisher  scenario_number_pub;
    //Messages
    std_msgs::Int32 scenario_number_msg;

    //values
    int scenario_number;


    ParamsSender() : nh_("~") {}
    void init();
    void publishMsgs();

    ~ParamsSender()   { }
  void keyboardLoop();
};


void ParamsSender::init() {
    scenario_number_pub = nh_.advertise<std_msgs::Int32>( "/android/scenario_number/", 1);
    nh_.param("scenario_number", scenario_number, -1);
    scenario_number_msg.data=scenario_number;

    ROS_INFO_STREAM("scenario");
}

void ParamsSender::publishMsgs() {
    scenario_number_pub.publish(scenario_number_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "params_sender");
    ParamsSender paramsSender;
    paramsSender.init();
    ros::Rate pub_rate(CNTRL_FREQ);

    while (paramsSender.nh_.ok()) {
        ros::spinOnce();
        paramsSender.publishMsgs();
        pub_rate.sleep();
    }
    return 0;
}
