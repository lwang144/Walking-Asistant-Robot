#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

using namespace std;

#define  ASR_CMD  1
string wakeup_word="可乐";  //default wakeup word

ros::Publisher pub;

void wakeupWordCheck(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Now will check wakeup world...");
    if(msg->data.find(wakeup_word) != string::npos)
    {
        system("play ~/Music/wakeup_alert.mp3");
        std_msgs::Int32 msg;
        msg.data = ASR_CMD;
        pub.publish(msg);
    }
    else
    {
        ROS_WARN_STREAM("Wakeup word don't match!");
    }
}

int main(int argc, char* argv[])
{
    string sphinx_topic = "/pocketSphinx_recognizer_node/output";
    string asr_topic = "/voice/xf_asr_topic";

    ros::init(argc, argv, "sphinx_wakeup_node");
    ros::NodeHandle ndHandle;

    ros::param::get("~sphinx_topic", sphinx_topic);
    ros::param::get("~asr_topic",    asr_topic);
    ros::param::get("~wakeup_word",  wakeup_word);

    ros::Subscriber sub = ndHandle.subscribe(sphinx_topic, 1, wakeupWordCheck);
    pub = ndHandle.advertise<std_msgs::Int32>(asr_topic, 1);
    ros::spin();

    return 0;
}
