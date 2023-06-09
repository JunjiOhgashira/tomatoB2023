#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <stdexcept>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "dynamixel_interface.hpp"

#define DEFAULT_BAUDRATE (1'000'000)
#define DEFAULT_PORTNAME ("/dev/ttyUSB0")
#define DEFAULT_LOOPRATE (1'000)
#define KEY_ELEVATOR_SPEED (80)

constexpr int arm_motor_ids[5] = {1, 2, 3, 4, 5}; // protocol version 1.0

// 6にしたらうごく
constexpr int elevator_motor_id = 6;             // protocol version 2.0
constexpr double motor_direction[5] = {1., 1., -1., -1., 1.};

int target_joint_position[5] = {};
int target_elevator_velocity = 0;

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void positionCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // int position = 0;

    for (int i = 0; i < 5; ++i)
    {
        // https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-position-36
        // position = (int)fmax(fmin(((motor_direction[i]*msg->position[i]/((5./6.)*M_PI)+1.)*512.),1023.),0.);
        // controller.setPosition(i+1, position);
        // target_joint_position[i] = position;
        target_joint_position[i] = (int)fmax(fmin(((motor_direction[i] * msg->position[i] / ((5. / 6.) * M_PI) + 1.) * 512.), 1023.), 0.);
    }
}

void elevatorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    int velocity = (int)fmax(fmin(msg->velocity[0], 1023.), -1023.);
    target_elevator_velocity = velocity;

    ROS_INFO("%d", velocity);
}

int main(int argc, char **argv)
{
    int baudrate = DEFAULT_BAUDRATE;
    std::string port_name = DEFAULT_PORTNAME;
    int node_freq = DEFAULT_LOOPRATE;
    DynamixelInterface controller;

    sensor_msgs::JointState present_js;
    int16_t position_raw = 0;
    sensor_msgs::JointState present_elevator;

    ros::init(argc, argv, "dynamixel_controller");
    ros::NodeHandle nh;
    nh.getParam("baudrate", baudrate);
    nh.getParam("port_name", port_name);
    nh.getParam("freq", node_freq);
    ros::Subscriber set_position_sub = nh.subscribe("/arm_joint_states", 10, positionCallback);
    ros::Publisher get_position_pub = nh.advertise<sensor_msgs::JointState>("/present_arm_joint_states", 1);
    ros::Subscriber set_elevator_sub = nh.subscribe("/elevator_states", 10, elevatorCallback);
    ros::Publisher get_elevator_pub = nh.advertise<sensor_msgs::JointState>("/present_elevator_states", 1);

    present_js.name.resize(5);
    present_js.position.resize(5);
    for (int i = 0; i < 5; ++i)
    {
        present_js.name[i] = "joint" + std::to_string(i + 1);
        present_js.position[i] = 0;
    }
    present_elevator.name.push_back("elevator");
    present_elevator.position.push_back(0);
    present_elevator.velocity.push_back(0);
    present_elevator.effort.push_back(0);

    for (int i = 0; i < 5; ++i)
    {
        controller.addMotor(arm_motor_ids[i], DynamixelInterface::PROTOCOL::VERSION_1);
    }
    controller.addMotor(elevator_motor_id, DynamixelInterface::PROTOCOL::VERSION_2);

    try
    {
        controller.open(port_name, baudrate);
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR("Dynamixel Initialization Error!!: %s", e.what());
        return -1;
    }

    controller.setOperationgMode(elevator_motor_id, DynamixelInterface::OPERATING_MODE::VELOCITY);
    controller.setTorqueEnableAll();
    // ros::Duration(10).sleep();

    for (int i = 0; i < 5; ++i)
    {
        target_joint_position[i] = controller.getPosition(arm_motor_ids[i]);
    }

    ros::Rate loop_rate(node_freq);
    int dxl_error = 0;
    int dxl_comm_result = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        if (kbhit())
        {
            switch (getchar())
            {
            case 'i':
                target_elevator_velocity = KEY_ELEVATOR_SPEED;
                break;
            case 'k':
                target_elevator_velocity = -KEY_ELEVATOR_SPEED;
                break;
            default:
                break;
            }
        }

        for (int i = 0; i < 5; ++i)
        {
            controller.setPosition(arm_motor_ids[i], target_joint_position[i]);
        }
        controller.setVelocity(elevator_motor_id, target_elevator_velocity);

        dxl_error = 0;
        dxl_comm_result = COMM_TX_FAIL;

        for (int i = 0; i < 5; ++i)
        {
            position_raw = controller.getPosition(arm_motor_ids[i]);
            present_js.position[i] = motor_direction[i] * ((double)position_raw / 512. - 1.) * ((5. / 6.) * M_PI);
        }
        position_raw = controller.getPosition(6);
        present_elevator.position[0] = ((double)position_raw / 2048. - 1.) * ((5. / 6.) * M_PI);
        present_elevator.velocity[0] = controller.getVelocity(6);
        present_elevator.effort[0] = controller.getCurrent(6);

        present_elevator.header.stamp = present_js.header.stamp = ros::Time::now();
        get_position_pub.publish(present_js);
        get_elevator_pub.publish(present_elevator);
        loop_rate.sleep();
    }

    controller.close();
    return 0;
}
