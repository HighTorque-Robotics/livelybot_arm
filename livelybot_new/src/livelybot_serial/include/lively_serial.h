#ifndef _LIVELY_SERIAL_H_
#define _LIVELY_SERIAL_H_

#include "serial_struct.h"
#include "serial/serial.h"
#include "ros/ros.h"
#include "hardware/motor.h"
// #include "livelybot_serial.h"
class lively_serial
{
private:
    serial::Serial _ser;
    std::string *_port;
    std::string _result;
    uint32_t _baudrate;
    uint8_t _debug_level;
    bool init_flag, send_flag, recv_flag;
    cdc_acm_tx_message_t cdc_acm_tx_message; // 串口发送的信息，来自于Motor，反馈给PC
    cdc_acm_rx_message_t cdc_acm_rx_message; // 串口收到的信息，来自于PC，用于控制Motor
    std::vector<motor_back_t *> Motor_data;
    std::vector<motor *> Motors;
    std::map<int, motor *> Map_Motors_p;
    ros::Rate *r;
    int *id;
    uint16_t crc_head;
    // livelybot_serial lv_ser;
public:
    lively_serial(std::string *port, uint32_t baudrate, uint8_t debug_level)
        : _port(port), _baudrate(baudrate), _debug_level(debug_level)
    {
        init_flag = false;
        send_flag = false;
        recv_flag = false;
        _ser.setPort(*_port); // 设置打开的串口名称

        _ser.setBaudrate(_baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
        _ser.setTimeout(to);                                       // 设置串口的timeout
        // 打开串口
        try
        {
            _ser.open(); // 打开串口
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Unable to open  motor port "+*_port); // 打开串口失败，打印信息
        }
        if (_ser.isOpen())
        {
            ROS_INFO_STREAM("IMU Serial Port initialized."); // 成功打开串口，打印信息
        }
        else
        {
        }
        cdc_acm_tx_message.head[1] = cdc_acm_rx_message.head[0] = 0xFE;
        cdc_acm_tx_message.head[0] = cdc_acm_rx_message.head[1] = 0xFD;
        init_flag = true;
        // start receive thread
    }
    ~lively_serial();
    void send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd);
    void send(cdc_acm_rx_message_t *_cdc_acm_rx_message);

    void recv();

    void init_map_motor(std::map<int, motor *> *_Map_Motors_p)
    {
        Map_Motors_p = *_Map_Motors_p;
        r = new ros::Rate(Map_Motors_p.size()*1100);
    }
    void test_ser_motor()
    {
        for (const auto &pair : Map_Motors_p)
        {
            std::cout << "Key: " << pair.first << ", Value: " << pair.second->get_motor_id() << std::endl;
        }
        int keyToFind = 4;
        auto it = Map_Motors_p.find(keyToFind);

        if (it != Map_Motors_p.end())
        {
            // 找到了键为 keyToFind 的元素
            std::cout << "Found: " << it->first << " -> " << it->second->get_motor_id() << std::endl;
        }
        else
        {
            // 未找到键为 keyToFind 的元素
            std::cout << "Key " << keyToFind << " not found in the map." << std::endl;
        }
    }
    lively_serial(const lively_serial &) = delete;
    lively_serial &operator=(const lively_serial &) = delete;
};

/* variable */

/* function */
#endif