#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

const int CNTRL_FREQ = 1;

class ParamsSender
{

  public:
    // Node Handlers
    ros::NodeHandle nh_;
    //Publishers
    ros::Publisher  scenario_number_pub;
    ros::Publisher  goal_pose_pub;
    //Messages
    std_msgs::Int32 scenario_number_msg;
    geometry_msgs::Twist goal_pose_msg;

    //values
    int scenario_number;
    float target_px;
    float target_py;
    float target_pz;
    float target_rx;
    float target_ry;
    float target_rz;

    ParamsSender() : nh_("~") {}
    void init();
    void publishMsgs();

    ~ParamsSender()   { }
  void keyboardLoop();
};

void ParamsSender::init() {
    scenario_number_pub = nh_.advertise<std_msgs::Int32>( "/android/scenario_number/", 1);
    goal_pose_pub = nh_.advertise<geometry_msgs::Twist>( "/android/goal_pose/", 1);
    nh_.param<int>("scenario_number", scenario_number, -1);
    nh_.param<float>("target_px", target_px, 0.f);
    nh_.param<float>("target_py", target_py, 0.f);
    nh_.param<float>("target_pz", target_pz, 0.f);
    nh_.param<float>("target_rx", target_rx, 0.f);
    nh_.param<float>("target_ry", target_ry, 0.f);
    nh_.param<float>("target_rz", target_rz, 0.f);

    scenario_number_msg.data=scenario_number;
    goal_pose_msg.linear.x=target_px;
    goal_pose_msg.linear.y=target_py;
    goal_pose_msg.linear.z=target_pz;
    goal_pose_msg.angular.x=target_rx;
    goal_pose_msg.angular.y=target_ry;
    goal_pose_msg.angular.z=target_rz;
    ROS_INFO_STREAM("params loaded");
}

void ParamsSender::publishMsgs() {
    scenario_number_pub.publish(scenario_number_msg);
    goal_pose_pub.publish(goal_pose_msg);
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
