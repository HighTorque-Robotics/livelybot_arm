#include "../include/lively_serial.h"
lively_serial::~lively_serial()
{
}
#define read_by_Byte 1
void lively_serial::recv()
{
    ROS_INFO_STREAM("start thread");
    while (ros::ok() && init_flag)
    {
#ifdef read_by_Byte
        _result = _ser.read(2);
        // ROS_INFO("==================================START");
        // ROS_INFO(_ser.getPort().c_str());
        if (*(uint8_t *)&_result[0] == 0xFD && *(uint8_t *)&_result[1] == 0xFE)
        {
            _result = _ser.read(sizeof(cdc_acm_tx_message_t)-2);
            // for (size_t i = 0; i < _result.size(); i++)
            // {
            //     printf("0x%02X ",*(uint8_t *)&_result[i]);
            // }
            // std::cout<<std::endl;
            memcpy(&cdc_acm_tx_message.motor_back_raw,(const void *)&_result[0],sizeof(cdc_acm_tx_message_t) - 2);
            // std::cout<< (int)cdc_acm_tx_message.motor_back.ID<<std::endl;

            if (cdc_acm_tx_message.crc16 == crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_tx_message, sizeof(cdc_acm_tx_message_t) - 2))
            {
                auto it = Map_Motors_p.find(cdc_acm_tx_message.motor_back_raw.ID);
                if (it != Map_Motors_p.end())
                {

                    it->second->fresh_data(cdc_acm_tx_message.motor_back_raw.position,
                                           cdc_acm_tx_message.motor_back_raw.velocity,
                                           cdc_acm_tx_message.motor_back_raw.torque);

                    // ROS_INFO("%d", it->second->get_motor_id());
                    // ROS_INFO("END");
                }
                else
                {
                    ROS_ERROR("OUT RANGE");
                }
            }
            else
            {
                // ROS_INFO("%X %X",cdc_acm_tx_message.crc16,crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_tx_message, sizeof(cdc_acm_tx_message_t) - 2));
                memset(&cdc_acm_tx_message.motor_back_raw,0,sizeof(cdc_acm_tx_message_t) - 2);
                ROS_ERROR("CRC ERROR");
            }
        }
        else
        {
            // ROS_ERROR("FRAME HEAD ERROR");
        }

#else // read avalible
      // this method is read the avaliable from the buffer
        _result = _ser.read(_ser.available());
        if (_result.length() > 0)
        {
            if (*(uint8_t *)&_result[0] == 0xFD && *(uint8_t *)&_result[1] == 0xFE)
            {

                // printf("0x%04X\n ", *(uint16_t *)&_result[15]);
                // printf("0x%04X\n ", crc_ccitt(0x0000, (const uint8_t *)&_result[0], sizeof(cdc_acm_tx_message_t) - 2));

                if (*(uint16_t *)&_result[15] == crc_ccitt(0x0000, (const uint8_t *)&_result[0], sizeof(cdc_acm_tx_message_t) - 2))
                {
                    // for (size_t i = 0; i < sizeof(cdc_acm_tx_message_t); i++)
                    // {
                    //     printf("0x%02X ", *(uint8_t *)&_result[i]);
                    // }
                    // std::cout << std::endl;
                    // ROS_INFO_STREAM("");
                    auto it = Map_Motors_p.find(*(uint8_t *)&_result[2]);
                    if (it != Map_Motors_p.end())
                    {
                        it->second->fresh_data(*(int32_t *)&_result[3], *(int32_t *)&_result[7], *(int32_t *)&_result[11]);
                    }
                    else
                    {
                        // ROS_ERROR("OUT RANGE");
                    }
                }
                else
                {
                    // ROS_ERROR("CRC ERROR");
                }
            }
            else
            {
                // ROS_INFO_STREAM("else 3");
            }
        }
        else
        {
            // ROS_INFO_STREAM("else 2");
        }
        // r->sleep();
#endif
        // TODO : this method is read one bytes
    }
}

void lively_serial::send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd)
{
    // ROS_INFO_STREAM("START");
    cdc_acm_rx_message.motor_cmd.position = position;
    cdc_acm_rx_message.motor_cmd.velocity = velocity;
    cdc_acm_rx_message.motor_cmd.torque = torque;
    cdc_acm_rx_message.motor_cmd.Kp = Kp;
    cdc_acm_rx_message.motor_cmd.Kd = Kd;
    cdc_acm_rx_message.motor_cmd.ID = ID;
    cdc_acm_rx_message.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t) - 2);
    uint8_t *byte_ptr = (uint8_t *)&cdc_acm_rx_message;
    // ROS_INFO_STREAM("STEP1");
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    // ROS_INFO_STREAM("STEP2");
    _ser.write((const uint8_t *)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message));
    // ROS_INFO_STREAM("END"); // STEP2 -> END 1.7ms  START -> END 1.71
}
void lively_serial::send(cdc_acm_rx_message_t *_cdc_acm_rx_message)
{
    uint8_t *byte_ptr = (uint8_t *)_cdc_acm_rx_message;
    // ROS_INFO("STEP1 %x",_cdc_acm_rx_message);
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    _ser.write((const uint8_t *)_cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t));
}
