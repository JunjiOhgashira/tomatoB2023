#include "../include/kobuki_operation/kobuki_operation.hpp"

KobukiOperation::KobukiOperation(double freq) : _nh(),
                                                _freq(freq),
                                                _update_rate(freq),
                                                _control{0},
                                                _speed_acc(1),
                                                _turn_acc(0.6)
{
    _cmd_sub = _nh.subscribe("ope_command", 1, &KobukiOperation::cmdCallBack, this);

    _kobuki_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void KobukiOperation::spin()
{
    ///< awaken timer
    ros::Duration(0.1).sleep();

    ///< change terminal setting
    struct termios old_terminal, new_terminal;
    int old_fcntl;
    tcgetattr(STDIN_FILENO, &old_terminal);
    new_terminal = old_terminal;
    new_terminal.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    old_fcntl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl | O_NONBLOCK);

    while (ros::ok())
    {
        ros::spinOnce();
        normalOperation();
    }
    kobukiStop();

    ///< restore terminal setting
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl);
}

void KobukiOperation::kobukiMove(double speed, double turn)
{
    _control.target_speed = speed;
    _control.target_turn = turn;
    kobukiInterpolate();
}

void KobukiOperation::kobukiStop()
{
    _control.target_speed = 0.0;
    _control.target_turn = 0.0;
    kobukiInterpolate();
}

void KobukiOperation::normalOperation()
{
    double motion_v_tmp;
    double rotation_v_tmp;

    if (_cmd == "/kobuki/straight")
    {
        motion_v_tmp = _val * 0.1;
        kobukiMove(motion_v_tmp, 0);
        ROS_INFO(_cmd.c_str());
    }
    else if (_cmd == "/kobuki/turn")
    {
        rotation_v_tmp = _val * 1.0;
        kobukiMove(0, rotation_v_tmp);
    }
    else
    {
        kobukiStop();
    }

    kobukiInterpolate();
    _update_rate.sleep();
}

void KobukiOperation::kobukiInterpolate()
{
    if (_control.target_speed > _control.control_speed)
        _control.control_speed = fmin(_control.target_speed, _control.control_speed + _speed_acc / _freq);
    else if (_control.target_speed < _control.control_speed)
        _control.control_speed = fmax(_control.target_speed, _control.control_speed - _speed_acc / _freq);
    else
        _control.control_speed = _control.target_speed;

    if (_control.target_turn > _control.control_turn)
        _control.control_turn = fmin(_control.target_turn, _control.control_turn + _turn_acc / _freq);
    else if (_control.target_turn < _control.control_turn)
        _control.control_turn = fmax(_control.target_turn, _control.control_turn - _turn_acc / _freq);
    else
        _control.control_turn = _control.target_turn;

    geometry_msgs::Twist command;
    command.linear.x = _control.control_speed;
    command.angular.z = _control.control_turn;
    _kobuki_pub.publish(command);
}

void KobukiOperation::cmdCallBack(const command_msgs::Joyope &cmd_msg)
{
    _cmd = cmd_msg.command.c_str();
    _val = cmd_msg.value;
    _val2 = cmd_msg.value2;
}