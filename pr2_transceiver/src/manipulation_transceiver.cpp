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

const int INTERFACE01 = 21;
const int INTERFACE02 = 22;
const int INTERFACE03 = 23;

const int PORT = 58528;
const int BUFSIZE = 256;
const float D2R = 0.017453293;
const char velocity_str[] = "velocity";
const char ptz_str[] = "ptz";

//ManipulationTransceiver Class
class ManipulationTransceiver {
  public:

    PointHeadClient* point_head_client_;

    // Node Handlers
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher r_arm_navegation_pub;
    ros::Publisher l_arm_navegation_pub;
    ros::Publisher r_arm_grasp_pub;
    ros::Publisher l_arm_grasp_pub;

    //Subscribers
    ros::Subscriber android_gripper_pose_sub;
    ros::Subscriber android_r_arm_navigation_sub;
    ros::Subscriber android_r_arm_grasp_sub;
    ros::Subscriber robot_odometry_sub;
    ros::Subscriber interface_number_sub;
    //Messages
    pr2_controllers_msgs::Pr2GripperCommand gripper_msg;
    geometry_msgs::PoseStamped r_ps_msg;
    geometry_msgs::PoseStamped l_ps_msg;


    // Conversion to Quaternion
    tf::Matrix3x3 gripper_euler_mat;
    tf::Quaternion gripper_quat;

    // Base odometry data
    float base_x;
    float base_y;
    float base_z;

    float gripper_yaw;
    float gripper_roll;
    float gripper_pitch;

    float gripper_pos_x;
    float gripper_pos_y;
    float gripper_pos_z;

    float target_pos_x;
    float target_pos_y;
    float target_pos_z;
    float target_radius;
    float depth_ang;
    float height_ang;
    float sphere_dX;
    float sphere_dY;

    int interface_number;

    //udp socket
    bool udpReady;                  /* # bytes received */
    struct sockaddr_in myaddr;      /* our address */
    struct sockaddr_in remaddr;     /* remote address */
    socklen_t addrlen;              /* length of addresses */
    int recvlen;                    /* # bytes received */
    int fd;                         /* our socket */
    char buf[BUFSIZE];     /* receive buffer */


    // Name our nodehandle "wam" to preceed our messages/services
    ManipulationTransceiver() : nh_("teleop_transceiver") {}
    void init();
    void lookAt(std::string frame_id, double x, double y, double z);
    void transceiveUDPMsg();
    void transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverGrasp(const std_msgs::Float32::ConstPtr& msg);
    void transceiverGripperPose(const std_msgs::Int32::ConstPtr& msg);
    void robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void interfaceNumberCallback(const std_msgs::Int32::ConstPtr& msg);
    void publishMsgs();

    ~ManipulationTransceiver() { close(fd); }
};

// WAM Teleoperation Initialization Function
void ManipulationTransceiver::init() {

    int fail=0;

    depth_ang=0.0;
    //height_ang=90.0*D2R;
    height_ang=0.0;
    sphere_dX=0.0;
    sphere_dY=0.0;

    target_pos_x=0.719612;
    target_pos_y=0.0753333;
    target_pos_z=-0.172468;
    target_radius=0.20;
    //target_pos_x=0.650141;
    //target_pos_y=0.092925;
    //target_pos_z=0.632065; //offset of the gripper


    gripper_yaw = 90.0*D2R;
    gripper_roll = 0.0*D2R;
    gripper_pitch = 0.0*D2R;
    udpReady = true;

    gripper_pos_x=target_pos_x - 0.001;
    gripper_pos_y=target_pos_y - 0.001;
    gripper_pos_z=target_pos_z + target_radius;


    //recalculate here the angles
    gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;
    gripper_roll = atan( (target_pos_z - gripper_pos_z) / (- target_pos_y + gripper_pos_y ) );
    gripper_pitch = 0.0;

    //Yaw=Z;Pitch=Y;Roll=X
    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch); //30° in Z
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.header.frame_id = "/torso_lift_link";
    r_ps_msg.pose.position.x= 0.75;
    r_ps_msg.pose.position.y=-0.2;
    r_ps_msg.pose.position.z= 0.1;
    r_ps_msg.pose.position.x= gripper_pos_x;
    r_ps_msg.pose.position.y= gripper_pos_y;
    r_ps_msg.pose.position.z= gripper_pos_z;
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

    //up
    gripper_euler_mat.getRotation(gripper_quat);
    l_ps_msg.header.frame_id = "/torso_lift_link";
    l_ps_msg.pose.position.x= 0.00;
    l_ps_msg.pose.position.y= 0.4;
    l_ps_msg.pose.position.z= 0.6;
    l_ps_msg.pose.orientation.x=0.0;
    l_ps_msg.pose.orientation.y=-0.707;
    l_ps_msg.pose.orientation.z=0.0;
    l_ps_msg.pose.orientation.w=0.707;


    //Subscribers
    interface_number_sub = nh_.subscribe<std_msgs::Int32> ( "/android/interface_number", 1, &ManipulationTransceiver::interfaceNumberCallback, this);
    android_r_arm_navigation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/r_arm_navigation", 1, &ManipulationTransceiver::transceiverRArmNavigation, this );
    android_r_arm_grasp_sub = nh_.subscribe<std_msgs::Float32> ("/android/r_arm_grasp", 1, &ManipulationTransceiver::transceiverGrasp, this );
    android_gripper_pose_sub = nh_.subscribe<std_msgs::Int32> ("/android/gripper_pose/preset", 1, &ManipulationTransceiver::transceiverGripperPose, this );
    robot_odometry_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 1, &ManipulationTransceiver::robotOdometry, this );

    //Publishers
    r_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/r_cart/command_pose", 1);
    l_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/l_cart/command_pose", 1);
    r_arm_grasp_pub = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>( "/r_gripper_controller/command", 1);
    l_arm_grasp_pub = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>( "/l_gripper_controller/command", 1);

    //Head control
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }

    //UDP socket config
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

void ManipulationTransceiver::interfaceNumberCallback(const std_msgs::Int32::ConstPtr& msg){
    interface_number = msg->data;
}

void ManipulationTransceiver::transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg){

    sphere_dX = sphere_dX + 0.25*msg->linear.x/CNTRL_FREQ;
    sphere_dY = sphere_dY + 0.25*msg->linear.y/CNTRL_FREQ;
    target_radius = target_radius + 0.5*msg->linear.z/CNTRL_FREQ;



    gripper_pos_x = target_pos_x + sphere_dX;
    gripper_pos_y = target_pos_y + sphere_dY;
    gripper_pos_z = target_pos_z + sqrt(target_radius*target_radius - sphere_dX*sphere_dX - sphere_dY*sphere_dY);
    r_ps_msg.pose.position.x = gripper_pos_x;
    r_ps_msg.pose.position.y = gripper_pos_y;
    r_ps_msg.pose.position.z = gripper_pos_z;


//TODO test with atan2, not to do all 4...
    if(sphere_dY < 0.f){
        gripper_roll = atan( (target_pos_z - gripper_pos_z) / (- target_pos_y + gripper_pos_y ) );
        gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
        gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;

    }else{
        gripper_roll = 3.141592654 - atan( (target_pos_z - gripper_pos_z) / ( target_pos_y - gripper_pos_y ) );
        gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
        gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;
    }
    //Yaw=Z;Pitch=Y;Roll=X
    ROS_INFO_STREAM("x: " << sphere_dX << " y: " << sphere_dY);
    ROS_INFO_STREAM("r: " << gripper_roll << " y: " << gripper_yaw << " p: " << gripper_pitch );

    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch);
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

    if(r_ps_msg.pose.position.x < 0.1)
        r_ps_msg.pose.position.x = 0.1;
    if(r_ps_msg.pose.position.x > 1.0)
        r_ps_msg.pose.position.x = 1.0;

    if(r_ps_msg.pose.position.y < -0.8)
        r_ps_msg.pose.position.y = -0.8;
    if(r_ps_msg.pose.position.y > 0.8)
        r_ps_msg.pose.position.y = 0.8;

    if(r_ps_msg.pose.position.z < -0.5)
        r_ps_msg.pose.position.z = -0.5;
    if(r_ps_msg.pose.position.z > 0.5)
        r_ps_msg.pose.position.z = 0.5;

}

/*
void ManipulationTransceiver::transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg){

    depth_ang = depth_ang + 2.0*msg->linear.x/CNTRL_FREQ;
    height_ang = height_ang - 2.0*msg->linear.y/CNTRL_FREQ;
    target_radius = target_radius + 0.5*msg->linear.z/CNTRL_FREQ;

    if(depth_ang < -1.57)
        depth_ang = -1.57;
    if(depth_ang > 1.57)
        depth_ang = 1.57;
    if(height_ang < 0)
        depth_ang = 0;
    if(height_ang > 3.1415)
        depth_ang = 3.1415;

//    gripper_pos_x = target_pos_x + target_radius*sin(depth_ang);
//    gripper_pos_y = target_pos_y + target_radius*cos(height_ang);
//    gripper_pos_z = target_pos_z + target_radius*sin(height_ang);

    ROS_INFO_STREAM("d: " << depth_ang << " h: " << height_ang);

    gripper_pos_x = target_pos_x + target_radius*sin(depth_ang)*cos(height_ang);
    gripper_pos_y = target_pos_y + target_radius*sin(depth_ang)*sin(height_ang);
    gripper_pos_z = target_pos_z + target_radius*cos(depth_ang);
    r_ps_msg.pose.position.x = gripper_pos_x;
    r_ps_msg.pose.position.y = gripper_pos_y;
    r_ps_msg.pose.position.z = gripper_pos_z;

    ROS_INFO_STREAM("xr: " << gripper_pos_x << " yr: " << gripper_pos_y << " zr: " << gripper_pos_z );
    //Yaw=Z;Pitch=Y;Roll=X

    if(depth_ang<0){
        gripper_roll = atan( (target_pos_z - gripper_pos_z) / (- target_pos_y + gripper_pos_y ) );
        gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
        gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;
    }else{
        gripper_roll = 3.141592654 - atan( (target_pos_z - gripper_pos_z) / ( target_pos_y - gripper_pos_y ) );
        gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
        gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;
    }

    ROS_INFO_STREAM("r: " << gripper_roll << " y: " << gripper_yaw << " p: " << gripper_pitch );
    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch);
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

    if(r_ps_msg.pose.position.x < 0.1)
        r_ps_msg.pose.position.x = 0.1;
    if(r_ps_msg.pose.position.x > 1.0)
        r_ps_msg.pose.position.x = 1.0;

    if(r_ps_msg.pose.position.y < -0.8)
        r_ps_msg.pose.position.y = -0.8;
    if(r_ps_msg.pose.position.y > 0.8)
        r_ps_msg.pose.position.y = 0.8;

    if(r_ps_msg.pose.position.z < -0.5)
        r_ps_msg.pose.position.z = -0.5;
    if(r_ps_msg.pose.position.z > 0.5)
        r_ps_msg.pose.position.z = 0.5;

}*/

void ManipulationTransceiver::transceiverGripperPose(const std_msgs::Int32::ConstPtr& msg){

    int pose = msg->data;
    if (pose==1){ //right2left
       depth_ang=-0.192491;
       height_ang=2.63017;
    } else if (pose==2) { //front2back
       depth_ang=-2.4102;
       height_ang=2.13221;
    } else if (pose==3) { //top2down
       depth_ang=0*D2R;
       height_ang=90.0*D2R;
    }

}

void ManipulationTransceiver::transceiverGrasp(const std_msgs::Float32::ConstPtr& msg){
    gripper_msg.position = msg->data;
    gripper_msg.max_effort = 100.0;
}

void ManipulationTransceiver::robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   //might be useful later...
   base_x = msg->pose.pose.position.x;
   base_y = msg->pose.pose.position.y;
   base_z = msg->pose.pose.position.z;

   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);

   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
}
//TODO
void ManipulationTransceiver::publishMsgs(){



    r_arm_navegation_pub.publish(r_ps_msg);  //arm1
    l_arm_navegation_pub.publish(l_ps_msg);  //arm2
    r_arm_grasp_pub.publish(gripper_msg);    //grasp1
    l_arm_grasp_pub.publish(gripper_msg);    //grasp2
    lookAt("r_gripper_tool_frame", 0, 0, 0); //head

}


//TODO
void ManipulationTransceiver::transceiveUDPMsg(){

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
	        //robot_navegation_pub.publish(velocity_msg);
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
void ManipulationTransceiver::lookAt(std::string frame_id, double x, double y, double z){

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
    ros::init(argc, argv, "manipulation_transceiver");
    ManipulationTransceiver manipulationTransceiver;
    manipulationTransceiver.init();
    ros::Rate pub_rate(CNTRL_FREQ);

    //boost::thread t( &ManipulationTransceiver::transceiveUDPMsg, &manipulationTransceiver );
    //ros::spin();

    while (manipulationTransceiver.nh_.ok()) {
        ros::spinOnce();
        manipulationTransceiver.publishMsgs();
        pub_rate.sleep();
    }
    return 0;
}
