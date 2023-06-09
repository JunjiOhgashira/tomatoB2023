#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "arm_ik.h"
#include <cmath>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <command_msgs/Joyope.h>

std::string cmd; // ccommand
float val, val2; // command value

bool communication_flag = false;

Angle4D current_angles = {};

void cmdCallBack(const command_msgs::Joyope &joy_ope)
{
  cmd = joy_ope.command.c_str();
  val = joy_ope.value;
  val2 = joy_ope.value2;
}

void motorStateCallback(const sensor_msgs::JointState &msg)
{
  communication_flag = true;
  current_angles.angle1 = msg.position[0];
  current_angles.angle2 = msg.position[1];
  current_angles.angle3 = msg.position[2];
  current_angles.angle4 = msg.position[3];
  current_angles.angle5 = msg.position[4];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_ik_node");
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/arm_joint_states", 1);
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("/target_point", 1);
  ros::Subscriber joy_sub = nh.subscribe("ope_command", 10, cmdCallBack);
  ros::Subscriber arm_sub = nh.subscribe("/present_arm_joint_states", 1, motorStateCallback);

  ros::Rate loop_rate(1000);
  ros::Rate loop_rate_slow(10);

  ArmMock arm_mock(0.05, 0.05, 0.15, 0.15, 0.1);
  ArmSolver arm_solver(0.05, 0.05, 0.15, 0.15, 0.1);
  ArmSmooth arm_smooth;

  Angle4D init_angles;
  init_angles.angle1 = 0;
  init_angles.angle2 = -0.782437;
  init_angles.angle3 = 2.395841;
  init_angles.angle4 = -0.042704;
  init_angles.angle5 = 0.0;

  while (!communication_flag)
  {
    ros::spinOnce();
    loop_rate_slow.sleep();
  }

  geometry_msgs::Point current_point;
  geometry_msgs::PointStamped target_point;

  arm_smooth.setCurrentAngles(current_angles);
  arm_smooth.setTargetAngles(init_angles);
  current_point = arm_mock.getTargetPoint();
  for (int i = 0; i < 3000; ++i)
  {
    ros::spinOnce();
    arm_mock.setAngle(arm_smooth.output(i / 3000.));
    joint_state_pub.publish(arm_mock.getJointState());

    current_point = arm_mock.getTargetPoint();
    target_point.point.x = current_point.x;
    target_point.point.y = current_point.y;
    target_point.point.z = current_point.z;
    point_pub.publish(target_point);
    loop_rate.sleep();
  }

  joint_state_pub.publish(arm_mock.getJointState());

  float pitch = 1.57;
  float t = 0.0;
  float dt = 0.001;
  while (ros::ok())
  {
    target_point.header.stamp = ros::Time::now();
    target_point.header.frame_id = "base_link";

    Angle4D angles;

    ros::spinOnce();

    current_point = arm_mock.getTargetPoint();

    if (cmd == "/crane/move_x")
    {
      target_point.point.x = current_point.x + val;
    }
    else if (cmd == "/crane/move_yz")
    {
      target_point.point.y = current_point.y + val;
      target_point.point.z = current_point.z + val2;
    }

    point_pub.publish(target_point);
    if (!arm_solver.solve(target_point.point, pitch, angles))
    {
      target_point.point.x = current_point.x;
      target_point.point.y = current_point.y;
      target_point.point.z = current_point.z;
      arm_solver.solve(target_point.point, pitch, angles);
    }
    arm_smooth.setTargetAngles(angles);
    for (int i = 0; i < 20; i++)
    {
      arm_mock.setAngle(arm_smooth.output((float)i / 20));
      joint_state_pub.publish(arm_mock.getJointState());
      ros::spinOnce();
      loop_rate.sleep();
    }
    current_point = arm_mock.getTargetPoint();
  }
  return 0;
}
