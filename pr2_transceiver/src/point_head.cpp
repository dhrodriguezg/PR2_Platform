#include <math.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include "geometry_msgs/Twist.h"

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
const int CNTRL_FREQ = 100; // Frequency at which we will publish the final stream


class RobotHead {
private:
  PointHeadClient* point_head_client_;
  ros::Subscriber target_sub;
  float ry;
  float rz;

public:
  ros::NodeHandle nh_;
  //! Action client initialization 
  RobotHead() : nh_("robot_head") {
    //Initialize the client for the Action interface to the head controller
    ry = 0.0f;
    rz = 0.0f;
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    target_sub = nh_.subscribe<geometry_msgs::Twist> ("/android/head_target", 1, &RobotHead::targetCallback, this );

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  //define methods
  void targetCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void lookAt(std::string frame_id, double x, double y, double z);
  void doTracking();

  ~RobotHead(){
    delete point_head_client_;
  }

};


//! Points the high-def camera frame at a point in a given frame  
void RobotHead::lookAt(std::string frame_id, double x, double y, double z)
{
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


void RobotHead::targetCallback(const geometry_msgs::Twist::ConstPtr& msg){
  ry = msg->angular.y + 1.57;
  rz = msg->angular.z;
}

//! Shake the head from left to right n times  
void RobotHead::doTracking(){

    lookAt("r_gripper_tool_frame", 0, 0, 0);
/*  float x = 0.0 + 2.0*sin(ry)*cos(rz);
  float y = 0.0 + 2.0*sin(ry)*sin(rz);
  float z = 1.2 + 2.0*cos(ry);

  lookAt("base_link", x, y, z);*/

  /*
  usleep(50000);
  //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
  lookAt("base_link", 5.0, 1.0, 1.2);
  usleep(50000);
  //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
  lookAt("base_link", 5.0, -1.0, 1.2);
  */

}




int main(int argc, char** argv){

  ros::init(argc, argv, "robot_head");
  RobotHead head;
  ros::Rate pub_rate(CNTRL_FREQ);

  while (head.nh_.ok()) {
        ros::spinOnce();
        head.doTracking();
        pub_rate.sleep();
  }
  return 0;
}
