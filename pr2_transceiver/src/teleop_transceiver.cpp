#include <math.h>
#include <ros/ros.h>
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

const int CNTRL_FREQ = 100; // Frequency at which we will publish the final stream
const int PORT = 58528;
const int BUFSIZE = 256;
const float D2R = 0.017453293;
const char velocity_str[] = "velocity";
const char ptz_str[] = "ptz";

//TeleopTransceiver Class
class TeleopTransceiver {
  public:

    // Node Handlers
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher robot_navegation_pub;
    ros::Publisher r_arm_navegation_pub;
    ros::Publisher l_arm_navegation_pub;
    ros::Publisher r_arm_grasp_pub;
    ros::Publisher l_arm_grasp_pub;

    //Subscribers
    ros::Subscriber android_robot_navegation_sub;
    ros::Subscriber android_r_arm_navigation_sub;
    ros::Subscriber android_gripper_pose_sub;
    ros::Subscriber android_r_arm_grasp_sub;
    ros::Subscriber robot_odometry_sub;
    ros::Subscriber target_sub;

    //Messages
    geometry_msgs::PoseStamped r_ps_msg;
    geometry_msgs::PoseStamped l_ps_msg;


    // Conversion to Quaternion
    tf::Matrix3x3 gripper_euler_mat;
    tf::Quaternion gripper_quat;

    // Base odometry data
    double base_yaw;
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

    float head_ry;
    float head_rz;


    //udp socket
    bool udpReady;                    /* # bytes received */
    struct sockaddr_in myaddr;      /* our address */
    struct sockaddr_in remaddr;     /* remote address */
    socklen_t addrlen;              /* length of addresses */
    int recvlen;                    /* # bytes received */
    int fd;                         /* our socket */
    char buf[BUFSIZE];     /* receive buffer */

    // Name our nodehandle "wam" to preceed our messages/services
    TeleopTransceiver() : nh_("teleop_transceiver") {}
    void init();
    void transceiveUDPMsg();
    void transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverGrasp(const std_msgs::Float32::ConstPtr& msg);
    void transceiverGripperPose(const std_msgs::Int32::ConstPtr& msg);
    void robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ~TeleopTransceiver() { close(fd); }
};

// WAM Teleoperation Initialization Function
void TeleopTransceiver::init() {

    int fail=0;
//    target_pos_x=0.650141;
//    target_pos_y=0.092925;
//    target_pos_z=0.632065; //offset of the gripper

    head_ry=0.0;
    head_rz=0.0;

    depth_ang=0.0;
    height_ang=90.0*D2R;


    target_pos_x=0.719612;
    target_pos_y=0.0753333;
    target_pos_z=-0.172468;
    target_radius=0.20;


    gripper_yaw = 90.0*D2R;
//    gripper_yaw = 00.0*D2R;
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
//    r_ps_msg.pose.position.x= 0.75;
//    r_ps_msg.pose.position.y=-0.2;
//    r_ps_msg.pose.position.z= 0.1;
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

    //rewrite

//r:hidden
    gripper_yaw = 1.0*D2R;
    gripper_roll = 0.0*D2R;
    gripper_pitch = 90.0*D2R;
    //Yaw=Z;Pitch=Y;Roll=X
    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch);
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.header.frame_id = "/torso_lift_link";
    r_ps_msg.pose.position.x= 0.70;
    r_ps_msg.pose.position.y=-0.05;
    r_ps_msg.pose.position.z=-0.60;
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

//l:hidden
    l_ps_msg.header.frame_id = "/torso_lift_link";
    l_ps_msg.pose.position.x= 0.70;
    l_ps_msg.pose.position.y= 0.05;
    l_ps_msg.pose.position.z=-0.60;
    l_ps_msg.pose.orientation.x=gripper_quat.getX();
    l_ps_msg.pose.orientation.y=gripper_quat.getY();
    l_ps_msg.pose.orientation.z=gripper_quat.getZ();
    l_ps_msg.pose.orientation.w=gripper_quat.getW();

    //gripper_euler_mat.setEulerYPR(90.0*D2R,0.0,180.0*D2R); //luego revisar angulos
    
//for right
/*
 x: 0.3
    y: 0.25
    z: -0.4
  orientation:
    x: 0.707
    y: 0.707
    z: 0.0
    w: 0.0"

x: 0.3
    y: -0.1
    z: -0.4
  orientation:
    x: 0.707
    y: 0.0
    z: 0.0
    w: 0.707"

position:
    x: 0.1
    y: -0.1
    z: -0.4
  orientation:
    x: 0.707
    y: 0.0
    z: 0.0
    w: 0.707"


*/


    //Subscribers
    android_robot_navegation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/robot_navegation", 1, &TeleopTransceiver::transceiverRobotNavigation, this );
    android_r_arm_navigation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/r_arm_navigation", 1, &TeleopTransceiver::transceiverRArmNavigation, this );
    android_r_arm_grasp_sub = nh_.subscribe<std_msgs::Float32> ("/android/r_arm_grasp", 1, &TeleopTransceiver::transceiverGrasp, this );
    android_gripper_pose_sub = nh_.subscribe<std_msgs::Int32> ("/android/gripper_pose/selection", 1, &TeleopTransceiver::transceiverGripperPose, this );
    robot_odometry_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 1, &TeleopTransceiver::robotOdometry, this );
    //target_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/head_target", 1, &TeleopTransceiver::targetCallback, this );

    //Publishers
    robot_navegation_pub = nh_.advertise<geometry_msgs::Twist>( "/base_controller/command", 1);
    r_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/r_cart/command_pose", 1);
    l_arm_navegation_pub = nh_.advertise<geometry_msgs::PoseStamped>( "/l_cart/command_pose", 1);
    r_arm_grasp_pub = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>( "/r_gripper_controller/command", 1);
    l_arm_grasp_pub = nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>( "/l_gripper_controller/command", 1);

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


void TeleopTransceiver::transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg){
    geometry_msgs::Twist twist_msg;
/*
    float x = msg->linear.x;
    float y = msg->linear.y;
    float mag = sqrt(x*x + y*y);
    float ang = atan2(y,x) - (float)base_yaw;

    twist_msg.linear.x = mag*cos(ang);
    twist_msg.linear.y = mag*sin(ang);

    twist_msg.linear.z = msg->linear.z;
    twist_msg.angular.x = msg->angular.x;
    twist_msg.angular.y = msg->angular.y;
    twist_msg.angular.z = msg->angular.z;
    robot_navegation_pub.publish(twist_msg);*/

    float x = msg->linear.x;
    float y = msg->linear.y;
    float mag = sqrt(x*x + y*y);
    float ang = atan2(y,x) - (float)base_yaw + msg->angular.z;

    twist_msg.linear.x = mag*cos(ang); //frente
    twist_msg.linear.y = msg->linear.z;
    twist_msg.linear.z = msg->linear.z; //vacio por ahora....
    twist_msg.angular.x = msg->angular.x;
    twist_msg.angular.y = msg->angular.y;
    twist_msg.angular.z = mag*sin(ang); //lado
    robot_navegation_pub.publish(twist_msg);

}

void TeleopTransceiver::transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg){
    // msg->linear.z or msg->angular.z



//    ROS_INFO_STREAM("depth_ang: " << depth_ang << " height_ang :" << height_ang);

    gripper_pos_x = gripper_pos_x + msg->linear.x/CNTRL_FREQ;
    gripper_pos_y = gripper_pos_y + msg->linear.y/CNTRL_FREQ;
    gripper_pos_z = gripper_pos_z + msg->linear.z/CNTRL_FREQ;

/*
    depth_ang=depth_ang + 2.0*msg->linear.x/CNTRL_FREQ;
    height_ang=height_ang - 2.0*msg->linear.y/CNTRL_FREQ;
    target_radius = target_radius + 0.5*msg->linear.z/CNTRL_FREQ;


    gripper_pos_x = target_pos_x + target_radius*sin(depth_ang);
    gripper_pos_y = target_pos_y + target_radius*cos(height_ang);
//FIX    gripper_pos_x = target_pos_x + target_radius*cos(height_ang)*sin(depth_ang);
//FIX    gripper_pos_y = target_pos_y + target_radius*cos(height_ang)*cos(depth_ang);
    gripper_pos_z = target_pos_z + target_radius*sin(height_ang);

    //Yaw=Z;Pitch=Y;Roll=X
    gripper_roll = gripper_roll + msg->angular.x/CNTRL_FREQ;
    gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
    gripper_yaw = gripper_yaw + msg->angular.z/CNTRL_FREQ;

    r_ps_msg.pose.position.x = gripper_pos_x;
    r_ps_msg.pose.position.y = gripper_pos_y;
    r_ps_msg.pose.position.z = gripper_pos_z;

    gripper_roll = atan( (target_pos_z - gripper_pos_z) / (- target_pos_y + gripper_pos_y ) );
    gripper_pitch = gripper_pitch + msg->angular.y/CNTRL_FREQ;
    gripper_yaw = atan( (target_pos_x - gripper_pos_x) / (- target_pos_y + gripper_pos_y ) )  + 1.570796327;
*/
    gripper_euler_mat.setEulerYPR(gripper_yaw,gripper_roll,gripper_pitch);
    gripper_euler_mat.getRotation(gripper_quat);
    r_ps_msg.pose.orientation.x=gripper_quat.getX();
    r_ps_msg.pose.orientation.y=gripper_quat.getY();
    r_ps_msg.pose.orientation.z=gripper_quat.getZ();
    r_ps_msg.pose.orientation.w=gripper_quat.getW();

    r_arm_navegation_pub.publish(r_ps_msg);
    l_arm_navegation_pub.publish(l_ps_msg);

//    ROS_INFO_STREAM("x: " << r_ps_msg.pose.position.x << " y :" << r_ps_msg.pose.position.y << " z: " << r_ps_msg.pose.position.z);
    if(r_ps_msg.pose.position.x < 0.1)
        r_ps_msg.pose.position.x = 0.1;
    if(r_ps_msg.pose.position.x > 1.0)
        r_ps_msg.pose.position.x = 1.0;

    if(r_ps_msg.pose.position.y < -0.8)
        r_ps_msg.pose.position.y = -0.8;
    if(r_ps_msg.pose.position.y > 0.8)
        r_ps_msg.pose.position.y = 0.8;
/*
    if(r_ps_msg.pose.position.z < -0.25)
        r_ps_msg.pose.position.z = -0.25;
    if(r_ps_msg.pose.position.z > 0.25)
        r_ps_msg.pose.position.z = 0.25;
*/
}

void TeleopTransceiver::transceiverGripperPose(const std_msgs::Int32::ConstPtr& msg){

    int pose = msg->data;
/*    if (pose==1){ //right2left
        gripper_roll = 0.0;
        gripper_pitch = 0.0;
        gripper_yaw = 1.570796327;
    } else if (pose==2) { //front2back
        gripper_roll = 0.0;
        gripper_pitch = 1.570796327;
        gripper_yaw = 0.0;
    } else if (pose==3) { //top2down
        gripper_roll = 1.570796327;
        gripper_pitch = 0.0;
        gripper_yaw = 0.0;
    }*/

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

void TeleopTransceiver::transceiverGrasp(const std_msgs::Float32::ConstPtr& msg){
    pr2_controllers_msgs::Pr2GripperCommand gripper_msg;
    gripper_msg.position = msg->data;
    gripper_msg.max_effort = 100.0;
    r_arm_grasp_pub.publish(gripper_msg);
    l_arm_grasp_pub.publish(gripper_msg);
}

void TeleopTransceiver::robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   //might be useful later...
   base_x = msg->pose.pose.position.x;
   base_y = msg->pose.pose.position.y;
   base_z = msg->pose.pose.position.z;

   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);

   double roll, pitch;
   m.getRPY(roll, pitch, base_yaw);
}

void TeleopTransceiver::headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg){
  head_ry = msg->angular.y + 1.57;
  head_rz = msg->angular.z;
}


//TODO
void TeleopTransceiver::transceiveUDPMsg(){

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


int main(int argc, char** argv)
{
    using namespace std;
    ros::init(argc, argv, "teleop_transceiver");
    TeleopTransceiver teleopTransceiver;
    teleopTransceiver.init();
    ros::Rate pub_rate(CNTRL_FREQ);
    boost::thread t( &TeleopTransceiver::transceiveUDPMsg, &teleopTransceiver );
    ros::spin();

    return 0;
}
