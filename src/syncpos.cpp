#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include<cv_bridge/cv_bridge.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;

ofstream file;
int cnt=0;

void callback(const geometry_msgs::PoseStampedConstPtr& realpos,const geometry_msgs::PoseStampedConstPtr& ledpos)
{
    file<<cnt++<<"\t"<<
        realpos->pose.position.x<<"\t"<<
        realpos->pose.position.y<<"\t"<<
        ledpos->pose.position.x<<"\t"<<
        ledpos->pose.position.y<<"\t"<<endl;

}


int main(int argc, char *argv[])
{
    file.open("/home/kk/dataset/cashe/posdata.txt",ios::out);


    //-------ros初始化
    
    ros::init(argc, argv, "syncpos");
    ros::start();
    ros::NodeHandle nh;

    Subscriber<PoseStamped> realsub(nh, "/vrpn_client_node/RigidBody01/pose", 100);
    Subscriber<PoseStamped> ledsub(nh, "/zbot/ledPose", 100);
    typedef sync_policies::ApproximateTime<PoseStamped, PoseStamped> sync_pol;
    Synchronizer<sync_pol> sync(sync_pol(100), realsub,ledsub);

    sync.registerCallback(boost::bind(&callback,_1,_2));


    
    ros::spin();
    file.close();

    // Stop all threads



    return 0;
}


