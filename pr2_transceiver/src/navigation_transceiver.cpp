#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <math.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Joy.h"

#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
const int CNTRL_FREQ = 100; // Frequency at which we will publish the final stream

const int INTERFACE01 = 11;
const int INTERFACE02 = 12;
const int INTERFACE03 = 13;

const int PORT = 58528;
const int BUFSIZE = 256;
const float D2R = 0.017453293;
const char velocity_str[] = "velocity";
const char ptz_str[] = "ptz";

//NavigationTransceiver Class
class NavigationTransceiver {
  public:

    PointHeadClient* point_head_client_;

    // Node Handlers
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher robot_navegation_pub;
    ros::Publisher r_arm_navegation_pub;
    ros::Publisher l_arm_navegation_pub;
    ros::Publisher robot_target_pub;

    //Subscribers
    ros::Subscriber android_robot_navegation_sub;
    ros::Subscriber interface_number_sub;
    ros::Subscriber user_measures_sub;
    ros::Subscriber robot_odometry_sub;
    ros::Subscriber head_target_sub;
    ros::Subscriber scenario_number_sub;
    ros::Subscriber goal_pose_sub;

    //Messages
    geometry_msgs::PoseStamped r_ps_msg;
    geometry_msgs::PoseStamped l_ps_msg;
    geometry_msgs::Twist robot_pose_msg;
    std_msgs::Float32 robot_target_msg;

    // Conversion to Quaternion
    tf::Matrix3x3 gripper_euler_mat;
    tf::Quaternion gripper_quat;

    float user_number;
    float user_time;
    float user_goal_time;
    float user_interface;

    // Base odometry data
    float base_yaw;
    float last_base_yaw;
    float base_x;
    float base_y;
    float base_z;

    float target_x;
    float target_y;
    float target_z;

    float gripper_yaw;
    float gripper_roll;
    float gripper_pitch;

    float gripper_pos_x;
    float gripper_pos_y;
    float gripper_pos_z;

    float head_ry;
    float head_rz;
    float last_head_rz;
    float sum_head_rz;

    int interface_number;
    int scenario_number;
    std::string username;

    float robot_target_x;
    float robot_target_y;
    float robot_target_rot;

    float target_distance;

    //udp socket
    bool udpReady;                    /* # bytes received */
    struct sockaddr_in myaddr;      /* our address */
    struct sockaddr_in remaddr;     /* remote address */
    socklen_t addrlen;              /* length of addresses */
    int recvlen;                    /* # bytes received */
    int fd;                         /* our socket */
    char buf[BUFSIZE];     /* receive buffer */

    // Name our nodehandle "wam" to preceed our messages/services
    NavigationTransceiver() : nh_("teleop_transceiver") {}
    void init();
    void lookAt(std::string frame_id, double x, double y, double z);
    void robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void interfaceNumberCallback(const std_msgs::Int32::ConstPtr& msg);
    void scenarioNumberCallback(const std_msgs::Int32::ConstPtr& msg);
    void goalPoseCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void userMeasures(const geometry_msgs::Twist::ConstPtr& msg);
    void headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiveUDPMsg();
    void publishMsgs();

    ~NavigationTransceiver() { close(fd); }
};

// WAM Teleoperation Initialization Function
void NavigationTransceiver::init() {

    udpReady = true;
    user_number=0.f;
    user_time=0.f;
    user_goal_time=0.f;
    user_interface=0.f;

    //scenario 1
    robot_target_x= 1.0;
    robot_target_y= 0.0;
    robot_target_rot = 0.0;

    last_base_yaw=0.0;
    base_yaw=0.0;
    head_ry=0.0;
    head_rz=0.0;

    //Yaw=Z;Pitch=Y;Roll=X
    gripper_yaw = 1.0*D2R;
    gripper_roll = 0.0*D2R;
    gripper_pitch = 90.0*D2R;

    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch);
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.header.frame_id = "/torso_lift_link";
    r_ps_msg.pose.position.x= 0.70;
    r_ps_msg.pose.position.y=-0.25;
    r_ps_msg.pose.position.z=-0.60;
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

    l_ps_msg.header.frame_id = "/torso_lift_link";
    l_ps_msg.pose.position.x= 0.70;
    l_ps_msg.pose.position.y= 0.25;
    l_ps_msg.pose.position.z=-0.60;
    l_ps_msg.pose.orientation.x=gripper_quat.getX();
    l_ps_msg.pose.orientation.y=gripper_quat.getY();
    l_ps_msg.pose.orientation.z=gripper_quat.getZ();
    l_ps_msg.pose.orientation.w=gripper_quat.getW();


    //Subscribers
    robot_odometry_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 1, &NavigationTransceiver::robotOdometry, this );
    interface_number_sub = nh_.subscribe<std_msgs::Int32> ( "/android/interface_number", 1, &NavigationTransceiver::interfaceNumberCallback, this);
    user_measures_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/user_measures", 1, &NavigationTransceiver::userMeasures, this );
    android_robot_navegation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/robot_navegation", 1, &NavigationTransceiver::transceiverRobotNavigation, this );
    head_target_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/head_target", 1, &NavigationTransceiver::headTargetCallback, this );
    scenario_number_sub = nh_.subscribe<std_msgs::Int32> ( "/android/scenario_number", 1, &NavigationTransceiver::scenarioNumberCallback, this);
    goal_pose_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/goal_pose", 1, &NavigationTransceiver::goalPoseCallback, this );

    //Publishers
    robot_navegation_pub = nh_.advertise<geometry_msgs::Twist>( "/base_controller/command", 1);
    r_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/r_cart/command_pose", 1);
    l_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/l_cart/command_pose", 1);
    robot_target_pub = nh_.advertise<std_msgs::Float32>( "/android/robot_base/target_distance", 1);

    //Head control
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
    nh_.param<std::string>("username", username, "default");

    //UDP socket config
    int fail;
    socklen_t addrlen = sizeof(remaddr);
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_INFO_STREAM("Can't create the socket :(");
	fail = fail+1;
    }
    setsockopt(fd,SOL_SOCKET,SO_RCVBUF, &BUFSIZE, sizeof(BUFSIZE) );

    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(PORT);
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
	ROS_INFO_STREAM("bind failed :(");
	fail = fail+1;
    }

    if( fail>0 )
        ROS_INFO_STREAM("UDP Command Socket NOT created :(");
    else
        ROS_INFO_STREAM("UDP Command Socket created!");

}

void NavigationTransceiver::interfaceNumberCallback(const std_msgs::Int32::ConstPtr& msg){
    interface_number = msg->data;
}

void NavigationTransceiver::scenarioNumberCallback(const std_msgs::Int32::ConstPtr& msg){
    scenario_number = msg->data;
}

void NavigationTransceiver::goalPoseCallback(const geometry_msgs::Twist::ConstPtr& msg){
    robot_target_x = msg->linear.x;
    robot_target_y = msg->linear.y;
    robot_target_rot = msg->angular.z;
}

void NavigationTransceiver::userMeasures(const geometry_msgs::Twist::ConstPtr& msg){
    user_number = msg->linear.x;
    user_interface = msg->linear.y;
    user_time = msg->angular.x;
    user_goal_time = msg->angular.y;
}

void NavigationTransceiver::transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg){

    robot_pose_msg.linear.x = 0.f; //frente
    robot_pose_msg.angular.z = 0.f; //rotar
    if(interface_number==INTERFACE01 || interface_number==INTERFACE02){
        robot_pose_msg.linear.x = msg->linear.x; //frente
        robot_pose_msg.angular.z = msg->angular.z; //rotar
    }else if (interface_number==INTERFACE03){
        robot_pose_msg.linear.x = msg->linear.x*target_x; //frente
        robot_pose_msg.angular.z = msg->linear.x*target_y; //rotar
    }
    if(robot_pose_msg.linear.x < 0)
        robot_pose_msg.angular.z = -robot_pose_msg.angular.z;
    robot_pose_msg.linear.y = msg->linear.y; //lado
    robot_pose_msg.linear.z = msg->linear.z;
    robot_pose_msg.angular.x = msg->angular.x;
    robot_pose_msg.angular.y = msg->angular.y;
}


void NavigationTransceiver::robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   //might be useful later...
    base_x = msg->pose.pose.position.x;
    base_y = msg->pose.pose.position.y;
    base_z = msg->pose.pose.position.z;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    base_yaw=(float)yaw;
}

void NavigationTransceiver::headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg){
    head_ry = msg->angular.y;
    head_rz = msg->angular.z;
}


void NavigationTransceiver::publishMsgs(){

    if(user_interface>0.f){
        if(user_number>0.f){
            std::string folder("/home/" + username + "/usability_test/user_" + boost::to_string(user_number));
            std::string name(folder + "/navigation_scenario_" + boost::to_string(scenario_number) + "_interface_" + boost::to_string(user_interface) + ".csv");
            boost::filesystem::path dir(folder);
            boost::filesystem::create_directories(dir);
            std::ofstream file(name.c_str());
            file << "total_time,goal_time,error_pos,error_rot,robot_x,robot_y,robot_yaw,goal_x,goal_y,goal_yaw" << std::endl;
            file << user_time << "," << user_goal_time << "," << target_distance << "," << (robot_target_rot-base_yaw);
            file << "," << base_x << "," << base_y << "," << base_yaw;
            file << "," << robot_target_x << "," << robot_target_y << "," << robot_target_rot;
            file.close();
            //user_interface=0.f;
            user_number=0.f;
            user_time=0.f;
        }
    }else{
        r_arm_navegation_pub.publish(r_ps_msg); //arm1
        l_arm_navegation_pub.publish(l_ps_msg); //arm2
        robot_navegation_pub.publish(robot_pose_msg); //robot pose
    
        target_distance = sqrt( (base_x-robot_target_x)*(base_x-robot_target_x) + (base_y-robot_target_y)*(base_y-robot_target_y) );
        robot_target_msg.data = target_distance;
        robot_target_pub.publish(robot_target_msg); //target distance
    
        //head
        float ref_head_ry = head_ry + 120*D2R;
        if(interface_number==INTERFACE01 || interface_number==INTERFACE02){
            target_x = 0.0 + 1.0*sin(ref_head_ry)*cos(head_rz);
            target_y = 0.0 + 1.0*sin(ref_head_ry)*sin(head_rz);
            target_z = 1.2 + 1.0*cos(ref_head_ry);
    
            lookAt("base_link", target_x, target_y, target_z);
    
        }else if (interface_number==INTERFACE03){
            float current_base_yaw = base_yaw;
            float current_head_rz = head_rz;
    
            float base_dRZ = current_base_yaw - last_base_yaw;
            float head_dRZ = current_head_rz - last_head_rz;
    
            sum_head_rz = sum_head_rz + head_dRZ - base_dRZ;
    
            target_x = 0.0 + 1.0*sin(ref_head_ry)*cos(sum_head_rz);
            target_y = 0.0 + 1.0*sin(ref_head_ry)*sin(sum_head_rz);
            target_z = 1.2 + 1.0*cos(ref_head_ry);
            //ROS_INFO_STREAM("XYZ: " << target_x << "|" << target_y << "|" << target_z);
            lookAt("base_link", target_x, target_y, target_z);
            last_base_yaw=current_base_yaw;
            last_head_rz = current_head_rz;
        }
    }
}


void NavigationTransceiver::transceiveUDPMsg(){

  while ( nh_.ok() ){

    geometry_msgs::Twist velocity_msg;
    std_msgs::Int32 ptz_msg;
    velocity_msg.linear.y = 0.0;
    velocity_msg.linear.z = 0.0;
    velocity_msg.angular.x = 0.0;
    velocity_msg.angular.y = 0.0;
    int command = 0;
    int ptz = 0;

    recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    if (recvlen > 0) {
        buf[recvlen] = 0;
	char* pch;
	pch = strtok (buf,";");
	while (pch != NULL)
	{
	    if( command==1 )
		velocity_msg.linear.x = (float)atof(pch);
	    if( command==2 ){
		velocity_msg.angular.z = (float)atof(pch);
	        robot_navegation_pub.publish(velocity_msg);
	    }

	    if( command > 0 )
	  	command=command+1;
	    if (strcmp (pch,velocity_str) == 0){
		command=1;
		ptz=0;
	    }

	    if( ptz==1 ){
		ptz_msg.data = (int)atoi(pch);
        	//r_arm_navegation_pub.publish(ptz_msg); change msg type
	    }

	    if( ptz > 0 )
	  	ptz=ptz+1;

	    if (strcmp (pch,ptz_str) == 0){
		command=0;
		ptz=1;
	    }

	    pch = strtok (NULL, ";");
	}
    }
  }
}

//! Points the high-def camera frame at a point in a given frame  
void NavigationTransceiver::lookAt(std::string frame_id, double x, double y, double z){

    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;
    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.1);

    //and go no faster than 1 rad/s
    goal.max_velocity = 10.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(0.1));

}


int main(int argc, char** argv)
{
    using namespace std;
    ros::init(argc, argv, "navigation_transceiver");
    NavigationTransceiver navigationTransceiver;
    navigationTransceiver.init();
    ros::Rate pub_rate(CNTRL_FREQ);

    //boost::thread t( &NavigationTransceiver::transceiveUDPMsg, &navigationTransceiver );
    //ros::spin();

    while (navigationTransceiver.nh_.ok()) {
        ros::spinOnce();
        navigationTransceiver.publishMsgs();
        pub_rate.sleep();
    }
    return 0;
}
