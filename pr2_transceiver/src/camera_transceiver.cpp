#include <math.h>
#include <ros/ros.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/CompressedImage.h"

const int CNTRL_FREQ = 20; // Frequency at which we will publish the final stream

//CameraTransceiver Class
class CameraTransceiver {
  public:

    // Node Handlers
    ros::NodeHandle nh_;

    // Android variables
    int selection;
    bool isCamera01;
    bool isCamera02;
    bool isCamera03;
    bool isCamera04;
    bool isCamera05;
    bool isCameraReady;
    //Messages
    sensor_msgs::CompressedImage camera01;
    sensor_msgs::CompressedImage camera02;
    sensor_msgs::CompressedImage camera03;
    sensor_msgs::CompressedImage camera04;
    sensor_msgs::CompressedImage camera05;
    sensor_msgs::CompressedImage cameraToSend;

    //Subscribers
    ros::Subscriber camera01_sub;
    ros::Subscriber camera02_sub;
    ros::Subscriber camera03_sub;
    ros::Subscriber camera04_sub;
    ros::Subscriber camera05_sub;
    ros::Subscriber cameraTS_sub;

    //Publishers
    ros::Publisher android_view_pub;

    // Name our nodehandle "wam" to preceed our messages/services
    CameraTransceiver() : nh_("camera_transceiver") {}
    void init();

    void selectionCallback(const std_msgs::Int32::ConstPtr& selection_msg);
    void camera01Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);
    void camera02Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);
    void camera03Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);
    void camera04Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);
    void camera05Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);
    void sendMsgs();
    ~CameraTransceiver() {}
};

// WAM Teleoperation Initialization Function
void CameraTransceiver::init() {

     // initializing Android variables
    selection=1; //wide_stereo is default.
    isCamera01 = false;
    isCamera02 = false;
    isCamera03 = false;
    isCamera04 = false;
    isCamera05 = false;
    isCameraReady = false;

    //Subscribers
    camera01_sub = nh_.subscribe <sensor_msgs::CompressedImage> ( "/wide_stereo/left/image_rect_color/compressed", 1, &CameraTransceiver::camera01Callback, this);
    camera02_sub = nh_.subscribe <sensor_msgs::CompressedImage> ( "/narrow_stereo/left/image_raw/compressed", 1, &CameraTransceiver::camera02Callback, this);
    camera03_sub = nh_.subscribe <sensor_msgs::CompressedImage> ( "/r_forearm_cam/image_raw/compressed", 1, &CameraTransceiver::camera03Callback, this);
    camera04_sub = nh_.subscribe <sensor_msgs::CompressedImage> ( "/camera_topdown/image_raw/compressed", 1, &CameraTransceiver::camera04Callback, this);
    camera05_sub = nh_.subscribe <sensor_msgs::CompressedImage> ( "/rviz/lateral_view/image_raw/compressed", 1, &CameraTransceiver::camera05Callback, this);
    cameraTS_sub = nh_.subscribe <std_msgs::Int32> ( "/android/camera/selection", 1, &CameraTransceiver::selectionCallback, this);

    //Publishers
    android_view_pub = nh_.advertise<sensor_msgs::CompressedImage>( "/android/image_raw/compressed", 1);

}

void CameraTransceiver::selectionCallback(const std_msgs::Int32::ConstPtr& selection_msg){
    selection = selection_msg->data;
}


void CameraTransceiver::camera01Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg){
    camera01.header = image_msg->header;
    camera01.format = image_msg->format;
    camera01.data = image_msg->data;
    isCamera01 = true;
}

void CameraTransceiver::camera02Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg){
    camera02.header = image_msg->header;
    camera02.format = image_msg->format;
    camera02.data = image_msg->data;
    isCamera02 = true;
}

void CameraTransceiver::camera03Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg){
    camera03.header = image_msg->header;
    camera03.format = image_msg->format;
    camera03.data = image_msg->data;
    isCamera03 = true;
}

void CameraTransceiver::camera04Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg){
    camera04.header = image_msg->header;
    camera04.format = image_msg->format;
    camera04.data = image_msg->data;
    isCamera04 = true;
}

void CameraTransceiver::camera05Callback(const sensor_msgs::CompressedImage::ConstPtr& image_msg){
    camera05.header = image_msg->header;
    camera05.format = image_msg->format;
    camera05.data = image_msg->data;
    isCamera05 = true;
}

void CameraTransceiver::sendMsgs(){

    if(selection==1 && isCamera01){
	cameraToSend=camera01;
        isCameraReady = true;
    }else if(selection==2 && isCamera02){
	cameraToSend=camera02;
        isCameraReady = true;
    }else if(selection==3 && isCamera03){
	cameraToSend=camera03;
        isCameraReady = true;
    }else if(selection==4 && isCamera04){
	cameraToSend=camera04;
        isCameraReady = true;
    }else if(selection==5 && isCamera05){
	cameraToSend=camera05;
        isCameraReady = true;
    }

    //checks if there are data to send.
    if (isCameraReady)
	android_view_pub.publish(cameraToSend);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_transceiver");
    CameraTransceiver cameraTransceiver;
    cameraTransceiver.init();

    ros::Rate pub_rate(CNTRL_FREQ);

    while (cameraTransceiver.nh_.ok()) {
        ros::spinOnce();
        cameraTransceiver.sendMsgs();
        pub_rate.sleep();
    }
    return 0;
}
