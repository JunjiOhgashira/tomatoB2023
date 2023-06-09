#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <command_msgs/Joyope.h>
#include <cmath>
#include "joy_config.h"

#define MAX_TIME 20
#define KOBUKI_SPEED 0.5
#define KOBUKI_ACC KOBUKI_SPEED / MAX_TIME
#define KOBUKI_TURN 0.5
#define KOBUKI_ANG_ACC KOBUKI_TURN / MAX_TIME

#define CRANE_MOVE_X 0.001
#define CRANE_MOVE_Y 0.001
#define CRANE_MOVE_Z 0.001

int btn[BUTTONS_NUM]; // Buttons
float ax[AXES_NUM];   // Axes

float sgn(float x)
{
    return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0);
}

void joyCallBack(const sensor_msgs::Joy &joy_msg)
{
    for (int i = 0; i < BUTTONS_NUM; i++)
    {
        btn[i] = joy_msg.buttons[i];
    }

    for (int i = 0; i < AXES_NUM; i++)
    {
        ax[i] = joy_msg.axes[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_operation");
    ros::NodeHandle nh;
    ros::Publisher joy_ope_pub = nh.advertise<command_msgs::Joyope>("ope_command", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallBack);
    bool is_hand_open = false;

    int time_speed = 0;
    int time_turn = 0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        command_msgs::Joyope cmd_msgs;

        // Operation using joystick
        if (ax[AXIS_LSTICK_LR] != 0.0 || ax[AXIS_LSTICK_UD] != 0.0)
        {
            cmd_msgs.command = "/crane/move_yz";
            cmd_msgs.value2 = CRANE_MOVE_Z * ax[AXIS_LSTICK_UD];
            cmd_msgs.value = CRANE_MOVE_Y * ax[AXIS_LSTICK_LR];
        }
        else if (ax[AXIS_RSTICK_UD] != 0.0)
        {
            cmd_msgs.command = "/crane/move_x";
            cmd_msgs.value = CRANE_MOVE_X * ax[AXIS_RSTICK_UD];
        }
        else if (btn[BUTTON_B] == 1)
        {
            cmd_msgs.command = "/crane/hand";
            cmd_msgs.value = 1.5;
            cmd_msgs.value2 = 0.01;
            is_hand_open = true;
        }
        else if (btn[BUTTON_B] == 0 && is_hand_open)
        {
            cmd_msgs.command = "/crane/hand";
            cmd_msgs.value = 0.05;
            cmd_msgs.value2 = -0.01;
            is_hand_open = false;
        }

        else if (btn[BUTTON_Y] == 1)
        {
            cmd_msgs.command = "/elevator/move";
            cmd_msgs.value = 1;
        }
        else if (btn[BUTTON_A] == 1)
        {
            cmd_msgs.command = "/elevator/move";
            cmd_msgs.value = -1;
            // Other input
        }

        else if (ax[AXIS_CROSS_UD] != 0.0)
        {
            cmd_msgs.command = "/kobuki/straight";
            cmd_msgs.value = ax[AXIS_CROSS_UD];
        }
        else if (ax[AXIS_CROSS_LR] != 0.0)
        {
            cmd_msgs.command = "/kobuki/turn";
            cmd_msgs.value = ax[AXIS_CROSS_LR];
        }

        else
        {
            cmd_msgs.command = "/none";
            cmd_msgs.value = 0.0;
            cmd_msgs.value2 = 0.0;
        }

        ROS_INFO("%s", cmd_msgs.command.c_str());

        // Publish command to crane+, kobuki and elevator
        joy_ope_pub.publish(cmd_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
}