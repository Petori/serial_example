#include <ros/ros.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <unistd.h>

using namespace std;

int main (int argc, char **argv){
    ros::init(argc, argv, "move_control");
    ros::NodeHandle n;

    ros::Publisher chatter = n.advertise<std_msgs::Char>("command_pub", 10);
    sleep(1.0);

     cout<<"请确保Serial port is Opened"<<endl;
    while(ros::ok())
    {
        cout<<"-----------------------------------------"<<endl;
        cout<<"左导轨靠近按'1'，远离按'2'；右导轨靠近按'3'，远离按'4'；按'5'停止运动；按's'退出"<<endl;

        char a;
        cin>>a;
        cout<<endl<<endl;
        std_msgs::Char b ;
        b.data = a;
        if(a!='s')
            {
            chatter.publish(b);
            }
        else
            {
            chatter.publish(b);
            usleep(200000);
            break;
        }
        }
    return 0;
}
