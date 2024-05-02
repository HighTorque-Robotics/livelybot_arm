#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>
#include "canboard.h"
#include "ros/ros.h"
#include <thread>
namespace lively_robot{
class robot
{
private:
    std::string robot_name, Serial_Type, CAN_type, CANboard_type, Serial_allocate;
    int arm_dof, leg_dof, CANboard_num, Seial_baudrate;
    ros::NodeHandle n;
    std::vector<canboard> CANboards;
    std::vector<std::string> str;
    std::vector<lively_serial *> ser;

public:
    std::vector<motor*> Motors;
    // std::vector<std::shared_ptr<canport>> CANPorts;
    std::vector<canport *> CANPorts;
    std::vector<std::thread> ser_recv_threads, send_threads;

    robot()
    {
        if (n.getParam("robot/Seial_baudrate", Seial_baudrate))
        {
            // ROS_INFO("Got params seial_baudrate: %s",seial_baudrate.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params seial_baudrate");
        }
        if (n.getParam("robot/robot_name", robot_name))
        {
            // ROS_INFO("Got params robot_name: %s",robot_name.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params robot_name");
        }
        if (n.getParam("robot/CANboard_num", CANboard_num))
        {
            // ROS_INFO("Got params CANboard_num: %d",CANboard_num);
        }
        else
        {
            ROS_ERROR("Faile to get params CANboard_num");
        }
        if (n.getParam("robot/CANboard_type", CANboard_type))
        {
            // ROS_INFO("Got params CANboard_type: %s",CANboard_type.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params CANboard_type");
        }
        if (n.getParam("robot/CANboard_type", CANboard_type))
        {
            // ROS_INFO("Got params CANboard_type: %s",CANboard_type.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params CANboard_type");
        }
        if (n.getParam("robot/Serial_Type", Serial_Type))
        {
            // ROS_INFO("Got params Serial_Type: %s",Serial_Type.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params Serial_Type");
        }
        if (n.getParam("robot/Serial_allocate", Serial_allocate))
        {
            // ROS_INFO("Got params Serial_Type: %s",Serial_Type.c_str());
        }
        else
        {
            ROS_ERROR("Faile to get params Serial_allocate");
        }
        

        ROS_INFO("\033[1;32mThe robot name is %s\033[0m", robot_name.c_str());
        ROS_INFO("\033[1;32mThe robot has %d CANboards\033[0m", CANboard_num);
        ROS_INFO("\033[1;32mThe CANboard type is %s\033[0m", CANboard_type.c_str());
        ROS_INFO("\033[1;32mThe Serial type is %s\033[0m", Serial_Type.c_str());
        ROS_INFO("\033[1;32mThe Serial allocate type is %s\033[0m", Serial_allocate.c_str());
        init_ser();
        if (Serial_allocate == "1for2")
        {
            for (size_t i = 1; i <= CANboard_num; i++) // 一个CANboard使用两个串口
            {
                CANboards.push_back(canboard(i, &ser));
            }
        }

        for (canboard &cb : CANboards)
        {
            cb.push_CANport(&CANPorts);
        }
        for (canport *cp : CANPorts)
        {
            // std::thread(&canport::send, &cp);
            cp->puch_motor(&Motors);
        }
        
        // test_ser_motor();
        ROS_INFO("\033[1;32mThe robot has %ld motors\033[0m", Motors.size());
        ROS_INFO("robot init");
        // for (motor m:Motors)
        // {
        //     std::cout<<m.get_motor_belong_canboard()<<" "<<m.get_motor_belong_canport()<<" "<<m.get_motor_id()<<std::endl;
        // }
    }
    void motor_send()
    {
        for (canboard &cb : CANboards)
        {
            cb.motor_send();
            // ROS_INFO("ok");
        }
    }
    void init_ser()
    {

        for (size_t i = 0; i < 4; i++)
        {
            str.push_back("/dev/ttyUSB" + std::to_string(i));
        }
        for (size_t i = 0; i < str.size(); i++)
        {
            // lively_serial *s = new lively_serial(&str[i], 2000000, 1);
            lively_serial *s = new lively_serial(&str[i], Seial_baudrate, 1);
            ser.push_back(s);
            ser_recv_threads.push_back(std::thread(&lively_serial::recv, s));
        }
    }
    void test_ser_motor()
    {
        for(lively_serial * s:ser)
        {
            s->test_ser_motor();
        }
    }
    ~robot() {}
};
}
#endif