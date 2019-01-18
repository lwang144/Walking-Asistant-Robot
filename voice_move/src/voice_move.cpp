/***********************************************************/
/* Author: www.corvin.cn                                   */
/***********************************************************/
/* Description:该源码为接收语音解析后控制小车移动的代码，  */
/*   包括小车的前后左右移动。                              */
/*                                                         */
/***********************************************************/
/* History:                                                */
/*  20180117:init this source code.                        */
/*                                                         */
/***********************************************************/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#define  MOVE_FORWARD_CMD  1
#define  MOVE_BACK_CMD     2
#define  MOVE_LEFT_CMD     3
#define  MOVE_RIGHT_CMD    4
#define  TURN_LEFT_CMD     5
#define  TURN_RIGHT_CMD    6

#define  STOP_MOVE_CMD     7
#define  BEDROOM_CMD       8
#define  DINNINGROOM_CMD   9
#define  STUDY_CMD         10
#define  BATHROOM_CMD      11

ros::Publisher pub;
float speed_x = 0.2;
float speed_y = 0.2;
float turn_speed = 0.5;
int pub_flag = 0;

geometry_msgs::Twist cmd_msg;


void subCallBack(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_WARN("Get Move CMD:%d",msg->data); 

    switch(msg->data)
    {
        case MOVE_FORWARD_CMD: //move forward
            cmd_msg.linear.x = speed_x;
            cmd_msg.linear.y = 0;
            cmd_msg.angular.z = 0;
            break;

        case MOVE_BACK_CMD: //move back 
            cmd_msg.linear.x = -speed_x; 
            cmd_msg.linear.y = 0;
            cmd_msg.angular.z = 0;
            break;

        case MOVE_LEFT_CMD: //move left 
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = speed_y; 
            cmd_msg.angular.z = 0;
            break;

        case MOVE_RIGHT_CMD: //move right 
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = -speed_y; 
            cmd_msg.angular.z = 0;
            break;

        case TURN_LEFT_CMD: //turn left 
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = 0; 
            cmd_msg.angular.z = turn_speed; 
            break;

        case TURN_RIGHT_CMD: //turn right 
            cmd_msg.linear.x = 0; 
            cmd_msg.linear.y = 0; 
            cmd_msg.angular.z = -turn_speed; 
            break;

        case STOP_MOVE_CMD: //stop move 
            cmd_msg.linear.x = 0; 
            cmd_msg.linear.y = 0; 
            cmd_msg.angular.z = 0; 
            ROS_WARN("Get stop move cmd:%d",msg->data); 
            break;

        default:
        	cmd_msg.linear.x = 0; 
            cmd_msg.linear.y = 0; 
            cmd_msg.angular.z = 0; 
            ROS_WARN("Get Unknown Move CMD:%d",msg->data); 
            break;
    }

    if((msg->data >= MOVE_FORWARD_CMD)&&(msg->data <= STOP_MOVE_CMD))
    {
        pub_flag = 1;
    }
    else
    {
        ROS_WARN("set pub_flag = 0"); 
        pub_flag = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "voice_move_node");
    ros::NodeHandle ndHandle;

    std::string sub_move_topic = "/voice/voice_move_topic";
    std::string pub_move_topic = "/cmd_vel";

    ros::param::get("~sub_move_topic",     sub_move_topic);
    ros::param::get("~pub_move_topic",     pub_move_topic);
    ros::param::get("~default_speed_x",    speed_x);
    ros::param::get("~default_speed_y",    speed_y);
    ros::param::get("~default_turn_speed", turn_speed);

    ros::Subscriber sub = ndHandle.subscribe(sub_move_topic, 1, subCallBack);    
    pub = ndHandle.advertise<geometry_msgs::Twist>(pub_move_topic, 1);

    ros::Rate loop_rate(5);
    while(ros::ok())
    {

        if(pub_flag)
        {
            pub.publish(cmd_msg);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

