#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <command_msgs/Joyope.h>

#define BTN_ELEVATOR_UP (0)
#define BTN_ELEVATOR_DOWN (1)

#define DEFAULT_ELEVATOR_SPEED (285)

std::string cmd; // ccommand
float val, val2; // command value

float elevator_direction = 0;

void cmdCallBack(const command_msgs::Joyope &joy_ope)
{
    cmd = joy_ope.command.c_str();
    val = joy_ope.value;
    val2 = joy_ope.value2;

    if (cmd == "/elevator/move")
    {
        elevator_direction = val;
    }
    else
    {
        elevator_direction = 0;
    }
}

int main(int argc, char **argv)
{
    int elevator_speed = DEFAULT_ELEVATOR_SPEED;

    ros::init(argc, argv, "elevator_commander");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");
    _nh.getParam("elevator_speed", elevator_speed);
    ros::Publisher elevator_state_pub = nh.advertise<sensor_msgs::JointState>("elevator_states", 1);
    ros::Subscriber joy_sub = nh.subscribe("ope_command", 10, cmdCallBack);

    sensor_msgs::JointState elevator_states;
    elevator_states.name.push_back("elevator");
    elevator_states.velocity.push_back(0);

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        elevator_states.velocity[0] = elevator_direction * elevator_speed;
        elevator_states.header.stamp = ros::Time::now();
        elevator_state_pub.publish(elevator_states);

        loop_rate.sleep();
    }

    return 0;
}
