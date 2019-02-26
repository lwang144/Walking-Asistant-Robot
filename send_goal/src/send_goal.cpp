#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

#define DINNINGROOM  "DINNINGROOM"
#define BEDROOM 	 "BEDROOM"
#define STUDY   	 "STUDY"
#define BATHROOM 	 "BATHROOM"

static int flag = 0;
static string result = "到达目的地";

bool Callback_flag = false;
string msg_str = "";

typedef struct _POSE
{
	double X;
	double Y;
	double Z;
	double or_x;
    double or_y;
    double or_z;
	double or_w;
} POSE;

POSE pose1 = {5.60237, -1.8963, 0.0, 0.0, 0.0, 0.0, 1.0};  //Bedroom  0.0, 0.0, 0.0, -0.740479961141, 0.672078438241
POSE pose2 = {-3.0, -2.5, 0.0,  0.0, 0.0, 1.0}; //STUDY
POSE pose3 = {-3.0, 1.0, 0.0,  0.0, 0.0, 0.0, 1.0};  //DInningRroom
POSE pose4 = {2.2, -2.2, 0.0, 0.0, 0.0, 0.0, 1.0}; //BathRoom

void setHome(ros::Publisher pub)
{
	geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
	msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = 0.0;
    msg_poseinit.pose.pose.position.y = 0.0;
    msg_poseinit.pose.pose.position.z = 0;
    msg_poseinit.pose.pose.orientation.x = 0.0;
    msg_poseinit.pose.pose.orientation.y = 0.0;
    msg_poseinit.pose.pose.orientation.z = 0.0;
    msg_poseinit.pose.pose.orientation.w = 1.0;

    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
}

void setGoal(POSE pose)
{
	 //tell the action client that we want to spin a thread by default 
    MoveBaseClient ac("move_base", true); 
       
    //wait for the action server to come up 
    while(!ac.waitForServer(ros::Duration(5.0))){ 
        ROS_WARN("Waiting for the move_base action server to come up"); 
    } 
       
    move_base_msgs::MoveBaseGoal goal; 

    //we'll send a goal to the robot to move 1 meter forward 
    goal.target_pose.header.frame_id = "map"; 
    goal.target_pose.header.stamp = ros::Time::now(); 
    
    goal.target_pose.pose.position.x = pose.X;
    goal.target_pose.pose.position.y = pose.Y; 
    goal.target_pose.pose.position.z = pose.Z;  
    goal.target_pose.pose.orientation.x = pose.or_x;
    goal.target_pose.pose.orientation.y = pose.or_y;
    goal.target_pose.pose.orientation.z = pose.or_z;
    goal.target_pose.pose.orientation.w = pose.or_w;  

    ROS_INFO("Sending goal"); 
     
    ac.sendGoal(goal); 
       
    ac.waitForResult(); 
       
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      ROS_INFO("it is successful"); 
      flag = 1;
    }
     else 
       ROS_ERROR("The base failed  move to goal!!!");  
}

void subCallBack(const std_msgs::String::ConstPtr &msg)
{
     ROS_INFO_STREAM("Topic is Subscriber ");
     std::cout<<"get topic text: " << msg->data << std::endl;
      
     Callback_flag = true;
     msg_str = msg->data;
}  

int main(int argc, char** argv) {
	ros::init(argc, argv, "send_goals_node");
	ros::NodeHandle ndHandle;

	std::string sub_move_topic = "/voice/send_goal_topic";
	ros::Subscriber sub = ndHandle.subscribe(sub_move_topic, 10, subCallBack);  
	ros::Publisher pub_initialpose = ndHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
	ros::Publisher pub = ndHandle.advertise<std_msgs::String>("/voice/xf_tts_topic", 10);   //TTS Node
	//ros::Rate rate_loop(10);
 	//setHome(pub_initialpose);

 	// setGoal(pose1);
  	while(ros::ok())
  	{
      if(Callback_flag == true)
      {
        Callback_flag = false;
 
        if(msg_str == BEDROOM)
        {
            msg_str = "";
            setGoal(pose1);
        }
        else  if(msg_str == STUDY)
        {
            msg_str = "";
            setGoal(pose2);
        }
        else if(msg_str == DINNINGROOM)
        {
 			      msg_str = "";
            setGoal(pose3);
        }
        else if(msg_str == BATHROOM)
        {
 			      msg_str = "";
            setGoal(pose4);
        }
      }
      if(flag)
      {
        std_msgs::String msg;
        msg.data = result;
        pub.publish(msg);
        flag = 0;
      }

      ros::spinOnce();
     // rate_loop.sleep();
  	}


	return 0;
}
