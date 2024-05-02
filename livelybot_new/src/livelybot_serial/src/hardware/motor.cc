#include "../../include/hardware/motor.h"
template <typename T>
inline T motor::float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case 0: // radian float pos/vel to int32
        return (int32_t)(in_data / my_2pi * 100000.0);
    case 1: // angle float pos/vel to int32
        return (int32_t)(in_data / 360.0 * 100000.0);
    case 2: // float torque to int32 5046*********************
        return (int32_t)((in_data )/ 0.000528);
    case 3: //float torque to int32 4538
        return (int32_t)((in_data )/ 0.000445);
    case 4: //float torque to int32 5047 36 减速比*************
        return (int32_t)((in_data )/ 0.000462);
    case 5: //float torque to int32 5047 9 减速比
        return (int32_t)((in_data )/ 0.000533);

    case 6: // float kp to int16 5046 
        return (int16_t)(in_data * 0x7FF);//kp~[0,15)************************************
    case 7:// float kd to int16 5046 
        return (int16_t)(in_data * 0x7FF);//kd~[0,15)************************************

    case 8://float kp to int16 4538 
        return (int16_t)((in_data*0x7FF/(my_2pi*0.5)));//kp~[0,50) max_tau 3Nm
    case 9://float kd to int16 4538 
        return (int16_t)((in_data*0x7FF/(my_2pi*0.005)));//kd~[0,0.5)

    case 10://float kp to int16 5047 36 减速比
        return (int16_t)((in_data*0x7FF));//kp~[0,15)********************************
    case 11://float kd to int16 5047 36 减速比
        return (int16_t)((in_data*0x7FF));//kd~[0,15)*********************************

    case 12://float kp to int16 5047 9 减速比
        return (int16_t)((in_data*0x7FF/(my_2pi*0.165)));//kp~[0,16)  max_tau 4Nm
    case 13://float kd to int16 5047 9 减速比
        return (int16_t)((in_data*0x7FF/(my_2pi*0.0033)));//kd~[0,0.33)

    case 14://float torque 4438 32 faker
        return (int32_t)((in_data )*2000);
    case 15://float kp to int16 4438 32
        return (int16_t)((in_data*0x7FF));//kp~[0,) max_tau 4Nm
    case 16://float kd to int16 4438 32
        return (int16_t)((in_data*0x7FF));//kd~[0,)


    case 17://float torque 4438 8 faker
        return (int32_t)((in_data )*2000);
    case 18://float kp to int16 4438 8
        return (int16_t)((in_data*0x7FF));//kp~[0,) max_tau 4Nm
    case 19://float kd to int16 4438 8
        return (int16_t)((in_data*0x7FF));//kd~[0,)

    
    case 20://float torque 7136 7 faker
        return (int32_t)((in_data )*2000);
    case 21://float kp to int16 7136 7 
        return (int16_t)((in_data*0x7FF));//kp~[0,) max_tau 4Nm
    case 22://float kd to int16 7136 7 
        return (int16_t)((in_data*0x7FF));//kd~[0,)
    default:
        return T();
    }
}

inline float motor::int2float(int32_t in_data, uint8_t type)
{
    switch (type)
    {
    case 0: // radian float pos/vel to int32
        return (float)(in_data * my_2pi / 100000.0);
    case 1: // angle float pos/vel to int32
        return (float)(in_data * 360.0 / 100000.0);


    case 2: // float torque to int32 5046******************************
        return (float)(in_data *0.000528);
    case 3: // float torque to int32 4538
        return (float)(in_data *0.000445);
    case 4: // float torque to int32 5047 36减速比***********************
        return (float)(in_data *0.000462);
    case 5: // float torque to int32 5047 9减速比
        return (float)(in_data *0.000533);
    case 6:// float torque to int32 4438 32 fake 
        return (float)(in_data/2000);
    case 7:// float torque to int32 4438 8 fake 
        return (float)(in_data/2000);
    case 8:// float torque to int32 7136 7 fake 
        return (float)(in_data/2000);
    default:
        return float();

    }
}

void motor::fresh_cmd(float position, float velocity, float torque, float Kp, float Kd)
{
    switch(type_)
    {
        case motor_type::null:
            ROS_INFO("error,motor type not set,fresh command fault");
            break;
        case motor_type::_5046:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 2);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 6);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 7);
            break;
        case motor_type::_4538:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 3);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 8);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 9);

            break;
        case motor_type::_5047_36:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 4);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 10);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 11);
            break;
        case motor_type::_5047_9:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 5);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 12);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 13);

            break;
        case motor_type::_4438_32:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 14);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 15);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 16);

            break;
        case motor_type::_4438_8:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 17);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 18);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 19);

            break;
        case motor_type::_7136:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 20);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 21);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 22);
            
            break;
        default:
            ROS_INFO("motor type setting error");
            return;
    }
    if(cmd.motor_cmd.Kp<0||cmd.motor_cmd.Kd<0)return;
    
    cmd.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cmd, sizeof(cdc_acm_rx_message_t) - 2);
    // ROS_INFO("CRC: 0x%x",cmd.crc16);
}

void motor::fresh_data(int32_t position, int32_t velocity, int32_t torque)
{
    switch(type_)
    {
        case motor_type::null:
            ROS_INFO("error,motor type not set,fresh data fault");
            break;
        case motor_type::_5046:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 2);
            break;
        case motor_type::_4538:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 3);
            break;
        case motor_type::_5047_36:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 4);
            break;
        case motor_type::_5047_9:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 5);
            break;
        case motor_type::_4438_32:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 6);
            break;
        case motor_type::_4438_8:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 7);
            break;
        case motor_type::_7136:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 8);
            break;
        default:
            ROS_INFO("motor type setting error");
            return;
    }

    _motor_pub.publish(p_msg);
}