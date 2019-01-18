/*
* NLU node
* author: leo Wang
* date: 2019.01
* 中文自然语言处理, 语音控制小车移动和导航
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>
#include <string>
#include <exception>
#include <codecvt>

using namespace std;
 
static int flag = 0;
static string goal_string = "";
static string result;

static string move_forward_str;
static string move_back_str;
static string move_left_str;
static string move_right_str;
static string turn_left_str;
static string turn_right_str;
static string stop_move_str;
static string bedroom_str;
static string dinningroom_str;
static string study_str;
static string bathroom_str;



#define MOVE_FORWARD_CMD 1
#define MOVE_BACK_CMD    2
#define MOVE_LEFT_CMD    3
#define MOVE_RIGHT_CMD   4
#define TURN_LEFT_CMD    5
#define TURN_RIGHT_CMD   6
#define STOP_MOVE_CMD    7

#define BEDROOM_CMD      8
#define DINNINGROOM_CMD  9
#define STUDY_CMD        10
#define BATHROOM_CMD     11

ros::Publisher cmd_vel_pub;
ros::Publisher send_goal_pub;

 int writer(char *data, size_t size, size_t nmemb, string *writerData)
{
    unsigned long sizes = size * nmemb;
    if (writerData == NULL)
        return -1;
 
    writerData->append(data, sizes);
 
    return sizes;
}

 /**
 * parse tuling server response json string
 */
int parseJsonResonse(string input)
{
    Json::Value root;
    Json::Reader reader;
    cout << "tuling server response origin json str:" << input << endl;
    bool parsingSuccessful = reader.parse(input, root);

    if(!parsingSuccessful)
    {
        cout << "!!! Failed to parse the response data" <<endl;
        return 1;
    }
    const Json::Value code = root["code"];
    const Json::Value text = root["text"];
    result = text.asString();
    flag = 1;
    cout << "response code:" << code << endl;
    cout << "response text:" << result <<endl;

    return 0;
}

/**
 * send tuling server http pose requeset
 */
int HttpPostRequest(string input)
{
    string buffer;
 
   std::string strJson = "{";
    strJson += "\"key\" : ";
    strJson += "\"";
    strJson += "e9f34edac06c455886c9e0c851f86bce";
    strJson += "\",";
    strJson += "\"info\" : ";
    strJson += "\"";
    strJson += input;
    strJson += "\"";
    strJson += "}";
 
    std::cout<<"post json string: " << strJson << std::endl;
 
     try
    {
        CURL *pCurl = NULL;
        CURLcode res;
        // In windows, this will init the winsock stuff
        curl_global_init(CURL_GLOBAL_ALL);
 
        // get a curl handle
        pCurl = curl_easy_init();
        if (NULL != pCurl)
        {
            // 设置超时时间为10秒
            curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 10);
 
            // First set the URL that is about to receive our POST.
            // This URL can just as well be a
            // https:// URL if that is what should receive the data.
            curl_easy_setopt(pCurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");
            //curl_easy_setopt(pCurl, CURLOPT_URL, "http://192.168.0.2/posttest.cgi");
 
            // 设置http发送的内容类型为JSON
            curl_slist *plist = curl_slist_append(NULL,"Content-Type:application/json;charset=UTF-8");
            curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);
 
            // 设置要POST的JSON数据
            curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, strJson.c_str());
 
            curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, writer);
 
            curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);
 
            // Perform the request, res will get the return code
            res = curl_easy_perform(pCurl);
            // Check for errors
            if (res != CURLE_OK)
            {
                printf("curl_easy_perform() failed:%s\n", curl_easy_strerror(res));
            }
            // always cleanup
            curl_easy_cleanup(pCurl);
        }
        curl_global_cleanup();
    }
    catch (std::exception &ex)
    {
        printf("curl exception %s.\n", ex.what());
    }
    if(buffer.empty())
    {
      std::cout<< "!!! ERROR The Tuling sever response NULL" << std::endl;
    }
    else
    {
        parseJsonResonse(buffer);
    }
 
    return 0;
 
}

wstring str2wstr(const std::string& str)
{
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.from_bytes(str);
}

int parseInputString(string input)
{
    int ret = 0;
    wstring convertStr = str2wstr(input); 

    wstring forwardStr   = str2wstr(move_forward_str);
    wstring backStr      = str2wstr(move_back_str);
    wstring moveLeftStr  = str2wstr(move_left_str);
    wstring moveRightStr = str2wstr(move_right_str);
    wstring turnLeftStr  = str2wstr(turn_left_str);
    wstring turnRightStr = str2wstr(turn_right_str);
    wstring stopMoveStr  = str2wstr(stop_move_str);
    wstring BedroomStr   = str2wstr(bedroom_str);
    wstring DinningroomStr    = str2wstr(dinningroom_str);
    wstring StudyStr     = str2wstr(study_str);
    wstring BathroomStr  = str2wstr(bathroom_str);

    if(convertStr.find(forwardStr) != string::npos)
    {
        ret = MOVE_FORWARD_CMD;
    }
    else if(convertStr.find(backStr) != string::npos) 
    {
        ret = MOVE_BACK_CMD; 
    }
    else if(convertStr.find(moveLeftStr) != string::npos) 
    {
        ret = MOVE_LEFT_CMD; 
    }
    else if(convertStr.find(moveRightStr) != string::npos) 
    {
        ret = MOVE_RIGHT_CMD; 
    }
    else if(convertStr.find(turnLeftStr) != string::npos) 
    {
        ret = TURN_LEFT_CMD; 
    }
    else if(convertStr.find(turnRightStr) != string::npos) 
    {
        ret = TURN_RIGHT_CMD; 
    }
    else if(convertStr.find(stopMoveStr) != string::npos) 
    {
        ret = STOP_MOVE_CMD; 
    }
    else if(convertStr.find(BedroomStr) != string::npos) 
    {
        goal_string = "BEDROOM"; 
        ret = BEDROOM_CMD;
    }
    else if(convertStr.find(DinningroomStr) != string::npos) 
    {
        goal_string = "DINNINGROOM"; 
        ret = DINNINGROOM_CMD;
    }
    else if(convertStr.find(StudyStr) != string::npos) 
    {
        goal_string = "STUDY"; 
        ret = STUDY_CMD;
    }
    else if(convertStr.find(BathroomStr) != string::npos) 
    {
        goal_string = "BATHROOM"; 
        ret = BATHROOM_CMD;
    }

    return ret;
}

/**
*   when nlp node get input,will auto send http post request to tuling server
**/
void arvCallBack(const std_msgs::String::ConstPtr &msg)
{
    int ret = 0;
    std::cout<<"your quesion is: " << msg->data << std::endl;

    ret = parseInputString(msg->data);
    ROS_INFO("tuling_nlu_node get parseInputString return: %d", ret);
    if(ret == 0) //send tuling nlu server to process
    {
        HttpPostRequest(msg->data);
    }
    if((ret >= MOVE_FORWARD_CMD)&&(ret <= BATHROOM_CMD))
    {
        std_msgs::Int32 move_msg;
        move_msg.data = ret;
        cmd_vel_pub.publish(move_msg);
    }
	if((ret >= BEDROOM_CMD)&&(ret <= BATHROOM_CMD))
    {
        std_msgs::String msg1;
        msg1.data = goal_string;
        send_goal_pub.publish(msg1);
    }
//    else //send nav msg
//    {
//        std_msgs::Int32 nav_msg;
//        nav_msg.data = ret;
//        nav_move_pub.publish(nav_msg);
//    }
}

/**
 * main function
 */ 
int main(int argc, char **argv)
{
    ros::init(argc, argv,"tuling_nlu_node");
    ros::NodeHandle nd;
    
    ros::param::get("~move_forward", move_forward_str);
    ros::param::get("~move_back",  move_back_str);
    ros::param::get("~move_left",  move_left_str);
    ros::param::get("~move_right", move_right_str);
    ros::param::get("~turn_left",  turn_left_str);
    ros::param::get("~turn_right", turn_right_str);
    ros::param::get("~stop_move",  stop_move_str);
    
    ros::param::get("~bedroom", bedroom_str);    
    ros::param::get("~dinningroom", dinningroom_str);
    ros::param::get("~study", study_str);
    ros::param::get("~bathroom", bathroom_str);

    ros::Subscriber sub = nd.subscribe("voice/tuling_nlu_topic", 10, arvCallBack);
    ros::Publisher pub = nd.advertise<std_msgs::String>("/voice/xf_tts_topic", 10);
    cmd_vel_pub = nd.advertise<std_msgs::Int32>("/voice/voice_move_topic", 10);
    send_goal_pub = nd.advertise<std_msgs::String>("/voice/send_goal_topic", 10);

    ros::Rate loop_rate(10);
 
    while(ros::ok())
    {
        if(flag)
        {
            std_msgs::String msg;
            msg.data = result;
            pub.publish(msg);
            
            flag = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
 
 
}
