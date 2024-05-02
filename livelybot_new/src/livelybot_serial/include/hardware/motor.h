#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "../serial_struct.h"
#include <stdint.h>
#include "ros/ros.h"
#include "livelybot_msg/MotorState.h"
#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

enum motor_type{
    null,
    _5046,// 黑色 圆形 20
    _4538,// 银色 圆形 
    _5047_36,//5047 双级 36减速比 黑色 方形 
    _5047_9,//5047 单级 9减速比 黑色 方形
    _4438_32,//black and white little 32
    _4438_8,//black and white little 8
    _7136,//black and white big  7 
};

class motor
{
private:
    int type, id, num,CANport_num, CANboard_num;
    ros::NodeHandle n;
    // lively_serial *ser;
    motor_back_t data;
    ros::Publisher _motor_pub;
    livelybot_msg::MotorState p_msg;
    std::string motor_name;
    motor_type type_=motor_type::null;
public:

    
    
    cdc_acm_rx_message_t cmd;
    motor(int _motor_num, int _CANport_num, int _CANboard_num) : CANport_num(_CANport_num), CANboard_num(_CANboard_num)
    {
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/name", motor_name))
        {
            // ROS_INFO("Got params name: %s",motor_name);
        }
        else
        {
            ROS_ERROR("Faile to get params name");
        }
        _motor_pub = n.advertise<livelybot_msg::MotorState>("/livelybot_real_real/" + motor_name + "_controller/state", 1);

        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/id", id))
        {
            // ROS_INFO("Got params id: %d",id);
        }
        else
        {
            ROS_ERROR("Faile to get params id");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/type", type))
        {
            // ROS_INFO("Got params type: %d",type);
        }
        else
        {
            ROS_ERROR("Faile to get params type");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/num", num))
        {
            // ROS_INFO("Got params num: %d",num);
        }
        else
        {
            ROS_ERROR("Faile to get params num");
        }
        set_motor_type(type);
        memset(&cmd, 0, sizeof(cmd));
        memset(&data, 0, sizeof(data));
        cmd.motor_cmd.ID = id;
        cmd.head[0] = 0xFE;
        cmd.head[1] = 0xFD;
        data.ID = id;
    }
    ~motor() {}
    template <typename T>
    inline T float2int(float in_data, uint8_t type);
    inline float int2float(int32_t in_data, uint8_t type);
    void fresh_cmd(float position, float velocity, float torque, float Kp, float Kd);
    void fresh_data(int32_t position, int32_t velocity, int32_t torque);
    int get_motor_id() { return id; }
    int get_motor_type() { return type; }
    motor_type get_motor_enum_type() {return type_;}
    int get_motor_num() { return num; }
    /***
     * @brief setting motor type 
     * @param type correspond to  different motor type 0~null 1~5046 2~5047_36减速比 3~5047_9减速比
    */
    void set_motor_type(size_t type)
    {
        type_=static_cast<motor_type>(type);
    }
    void set_motor_type(motor_type type)
    {
        type_=type;
    }
    int get_motor_belong_canport() { return CANport_num; }
    int get_motor_belong_canboard() { return CANboard_num; }
    cdc_acm_rx_message_t *return_cmd_p() { return &cmd; }
    motor_back_t *get_current_motor_state() { return &data; }
};
#endif