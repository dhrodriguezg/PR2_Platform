#include <ctime>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <tf/transform_datatypes.h>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <cstdio>
#include <stdio.h>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace std;

const int CNTRL_FREQ = 100;

class ROSBagExtractor {
  public:

    // Node Handlers
    ros::NodeHandle nh_;

    //Subscribers
    ros::Subscriber robot_odometry_sub;
    ros::Subscriber arm_command_sub;
    ros::Subscriber user_measures_sub;
    ros::Subscriber camera_sub;
    ros::Subscriber head_target_sub;
    ros::Subscriber navegation_sub;
    ros::Subscriber arm_navigation_sub;
    ros::Subscriber arm_grasp_sub;

    //Messages
    geometry_msgs::PoseStamped r_ps_msg;
    geometry_msgs::PoseStamped l_ps_msg;

    string path;
    string user;
    string interface_number;
    string scenario_number;

    bool robot_data;
    bool user_data;
    bool navigation_task;
    bool manipulation_task;
    bool task_nav_started;
    bool task_man_started;
    bool task_nav_finished;
    bool task_man_finished;

    ofstream* robot_global_file;
    ofstream* user_global_file;

    clock_t init_time;
    int measurement_number;
    //external data used when recording
    int camera_selection;
    float head_ry;
    float head_rz;
    float grasp;


    ROSBagExtractor() : nh_("rosbag_extractor") {}
    void init();
    void begin();
    void playROSBag(string rosbag);
    void extractMetadata(string rosbag_file);

    void robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void armPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void userMeasures(const geometry_msgs::Twist::ConstPtr& msg);

    void selectionCallback(const std_msgs::Int32::ConstPtr& selection_msg);
    void headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg);
    void transceiverGrasp(const std_msgs::Float32::ConstPtr& msg);

    ~ROSBagExtractor() {}
};

void ROSBagExtractor::init(){

    robot_data=true;
    user_data=true;
    task_nav_finished=false;
    task_nav_started=false;
    task_man_finished=false;
    task_man_started=false;

    measurement_number=0;
    camera_selection=0;
    head_ry=0.f;
    head_rz=0.f;
    grasp=0.f;

    path="/home/dhrodriguezg/usability_test/Rosbags/";
    
    //Subscribers
    if(robot_data){
        robot_odometry_sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 1, &ROSBagExtractor::robotOdometry, this );
        arm_command_sub = nh_.subscribe<geometry_msgs::PoseStamped> ("/r_cart/command_pose", 1, &ROSBagExtractor::armPose, this );
    }
    if(user_data){
        camera_sub = nh_.subscribe <std_msgs::Int32> ( "/android/camera/selection", 1, &ROSBagExtractor::selectionCallback, this);
        //navigation
        head_target_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/head_target", 1, &ROSBagExtractor::headTargetCallback, this );
        navegation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/robot_navegation", 1, &ROSBagExtractor::transceiverRobotNavigation, this );
        //manipulation
        arm_navigation_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/r_arm_navigation", 1, &ROSBagExtractor::transceiverRArmNavigation, this );
        arm_grasp_sub = nh_.subscribe<std_msgs::Float32> ("/android/r_arm_grasp", 1, &ROSBagExtractor::transceiverGrasp, this );
    }
    user_measures_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/user_measures", 1, &ROSBagExtractor::userMeasures, this );

}

void ROSBagExtractor::begin(){

    vector<string> rosbag_list;
    rosbag_list.clear();
    char buff[512];
    FILE *in;

    //get list of rosbag files
    string command("ls " + path);
    if(!(in = popen(command.c_str(), "r"))){
        ROS_INFO_STREAM("ERROR GETTING THE LIST OF ROSBAG");
        return;
    }

    while( fgets(buff, sizeof(buff), in) != NULL ){
        rosbag_list.push_back( buff );
    }
    pclose(in);

    ROS_INFO_STREAM("NUMBER OF FILES FOUND: " << rosbag_list.size() );

    for(int i=0; i < rosbag_list.size() && nh_.ok(); i++){
        playROSBag(rosbag_list.at(i).substr(0, rosbag_list.at(i).size()-1) );
    }

    ROS_INFO_STREAM("(OK)ALL COMPATIBLE FILES WERE EXTRACTED");

}

void ROSBagExtractor::playROSBag(string rosbag_file){

    task_nav_finished=false;
    task_man_finished=false;
    ofstream robot_file;
    ofstream user_file;

    if (rosbag_file.substr(rosbag_file.size()-4, rosbag_file.size()).compare(".bag") != 0) //check extension
        return;
    //Metadata
    extractMetadata(rosbag_file);

    if(robot_data){
        string task("");
        task = navigation_task ? "robot_navigation" : "robot_manipulation" ;
        string folder( path + "output/robot/");
        string name(folder + task + "_u" + user + "_s" + scenario_number + "_i" + interface_number + ".csv");
        boost::filesystem::path dir(folder);
        boost::filesystem::create_directories(dir);
        robot_file.open(name.c_str());
        robot_global_file = &robot_file;
        if(navigation_task)
            robot_file << "secs,nsecs,user,scenario,interface,robot_x,robot_y,robot_yaw" << std::endl;
        else if(manipulation_task)
            robot_file << "seq,user,scenario,interface,arm_x,arm_y,arm_z,arm_roll,arm_pitch,arm_yaw" << std::endl;
    }
    if(user_data){
        string task("");
        task = navigation_task ? "user_navigation" : "user_manipulation" ;
        string folder( path + "output/client/");
        string name(folder + task + "_u" + user + "_s" + scenario_number + "_i" + interface_number + ".csv");
        boost::filesystem::path dir(folder);
        boost::filesystem::create_directories(dir);
        user_file.open(name.c_str());
        user_global_file = &user_file;
        if(navigation_task)
            user_file << "secs,camera,head_ry,head_rz,pos_x,ang_z" << std::endl;
        else if(manipulation_task)
            user_file << "secs,camera,grasp,radius,sphere_x,sphere_y,pitch" << std::endl;
    }
    //playback
    string command("rosbag play " + path + rosbag_file);
    char buff[512];
    FILE *in;

    //get list of rosbag files
    ROS_INFO_STREAM("PLAYING ROSBAG FILE: " << rosbag_file);
    init_time = clock();
    measurement_number=0;
    if(!(in = popen(command.c_str(), "r"))){
        ROS_INFO_STREAM("(ERR)ERROR PLAYING ROSBAG FILE" << rosbag_file);
        return;
    }

    while( fgets(buff, sizeof(buff), in) != NULL ){
        //ignore msg output
        if(!nh_.ok()) //if ros is closing -> stop
             return;
        
    }
    task_nav_finished=true;
    task_man_finished=true;

    pclose(in);
    if(robot_data)
        robot_file.close();
    if(user_data)
        user_file.close();

    ROS_INFO_STREAM("(OK)FINISHED PLAYING ROSBAG FILE: " << rosbag_file);
}


void ROSBagExtractor::extractMetadata(string rosbag_file){

    string delimiter_underscore = "_";
    string delimiter_dash = "-";
    string s = rosbag_file.substr(0, rosbag_file.find(delimiter_underscore));

    size_t pos = 0;
    int i=0;
    string token;
    while ((pos = s.find(delimiter_dash)) != std::string::npos) {
        token = s.substr(0, pos);
        if(i==0){
            user=token.substr(1, token.size());
        }else if(i==1){
            scenario_number=token.substr(2, token.size());
            if( token.substr(0, 1).compare("n") == 0 ){
                navigation_task=true;
                manipulation_task=false;
            }else{
                navigation_task=false;
                manipulation_task=true;
            }
        }
        s.erase(0, pos + delimiter_dash.length());
        i++;
    }
    //last token
    interface_number=s.substr(2, s.size());

    ROS_INFO_STREAM("User: " << user);
    ROS_INFO_STREAM("Scenario: " << scenario_number);
    ROS_INFO_STREAM("Interface: " << interface_number);
    ROS_INFO_STREAM("Nav|Man: " << navigation_task << "|" << manipulation_task);
}



void ROSBagExtractor::robotOdometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
   if(!navigation_task)
       return;

   int  secs = msg->header.stamp.sec;
   long nsecs = msg->header.stamp.nsec;
   float base_x = msg->pose.pose.position.x;
   float base_y = msg->pose.pose.position.y;

   if(!task_nav_started){
      if(sqrt(base_x*base_x+base_y*base_y) > 0.0005)
         task_nav_started=true;
   }
   if ( !task_nav_started || task_nav_finished)
      return;

   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   if(robot_data)
      *robot_global_file << secs << "," << nsecs << "," << user << "," << scenario_number << "," << interface_number << "," << base_x << "," << base_y << "," << yaw << std::endl;
}



void ROSBagExtractor::armPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
   if(!manipulation_task)
       return;

   long seq = msg->header.seq;
   float arm_x = msg->pose.position.x;
   float arm_y = msg->pose.position.y;
   float arm_z = msg->pose.position.z;

   tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
   tf::Matrix3x3 m(q);
   double arm_roll, arm_pitch, arm_yaw;
   m.getRPY(arm_roll, arm_pitch, arm_yaw);

   if(!task_man_started){
      if(arm_x > 0.52)
         task_man_started=true;
   }
   if (!task_man_started || task_man_finished)
      return;

   if(robot_data)
      *robot_global_file << seq << "," << user << "," << scenario_number << "," << interface_number << "," << arm_x << "," << arm_y << "," << arm_z << "," << arm_roll << "," << arm_pitch << "," << arm_yaw << std::endl;
}



void ROSBagExtractor::selectionCallback(const std_msgs::Int32::ConstPtr& msg){
    camera_selection = msg->data;
}

void ROSBagExtractor::headTargetCallback(const geometry_msgs::Twist::ConstPtr& msg){
    head_ry = msg->angular.y;
    head_rz = msg->angular.z;
}
void ROSBagExtractor::transceiverRobotNavigation(const geometry_msgs::Twist::ConstPtr& msg){
    float pos_x = msg->linear.x; //frente
    float pos_y = msg->linear.y; //lado
    float ang_z = msg->angular.z; //rotar

    if ( !task_nav_started || task_nav_finished)
      return;

    double secs = double(clock() - init_time) / CLOCKS_PER_SEC;
    measurement_number = measurement_number + 1;

    if(user_data)
        *user_global_file << measurement_number << "," << camera_selection << "," << head_ry << "," << head_rz << "," << pos_x << "," << ang_z << std::endl;

}

void ROSBagExtractor::transceiverGrasp(const std_msgs::Float32::ConstPtr& msg){
    grasp = msg->data;//grasp
}

void ROSBagExtractor::transceiverRArmNavigation(const geometry_msgs::Twist::ConstPtr& msg){
    float sphere_x = msg->linear.x;//x
    float sphere_y = msg->linear.y;//y
    float radius = msg->linear.z;//radius
    float pitch = msg->angular.y;//pitch

    if (!task_man_started || task_man_finished)
        return;

    double secs = double(clock() - init_time) / CLOCKS_PER_SEC;
    measurement_number = measurement_number + 1;
    if(user_data)
        *user_global_file << measurement_number << "," << camera_selection << "," << grasp << "," << radius << "," << sphere_x << "," << sphere_y << "," << pitch << std::endl;
}

void ROSBagExtractor::userMeasures(const geometry_msgs::Twist::ConstPtr& msg){
    task_nav_finished=true;
    task_man_finished=true;
}


int main(int argc, char** argv)
{
    using namespace std;
    ros::init(argc, argv, "rosbag_extractor");
    ROSBagExtractor rosbagExtractor;
    rosbagExtractor.init();
    ros::Rate pub_rate(CNTRL_FREQ);
    boost::thread t( &ROSBagExtractor::begin, &rosbagExtractor );
    ros::spin();

    return 0;
}
