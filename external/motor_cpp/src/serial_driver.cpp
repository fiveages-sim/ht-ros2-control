#include "serial_driver.hpp"
#include <cstring>
#include <iostream>

namespace
{
// 与原 serial::Timeout::simpleTimeout(1000) 语义一致：读/写超时 1000 ms
constexpr int kSerialTimeoutMs = 1000;

// libserialport 返回负值表示错误；错误时抛出异常以保持原有 try/catch 语义
void check_sp(sp_return r, const char *what)
{
    if (r < SP_OK)
    {
        throw std::runtime_error(std::string(what) + ": " + sp_last_error_message());
    }
}
}  // namespace

serial_driver::serial_driver(std::string *port, uint32_t baudrate, bool _canport_error_output_flag): canport_error_output_flag(_canport_error_output_flag)
{
    init_flag = false;
    error_flag = false;

    try
    {
        // 按端口名创建并打开串口（读写模式）
        if (sp_get_port_by_name(port->c_str(), &_ser) != SP_OK || _ser == nullptr)
        {
            throw std::runtime_error("sp_get_port_by_name failed");
        }
        check_sp(sp_open(_ser, SP_MODE_READ_WRITE), "sp_open");
        check_sp(sp_set_baudrate(_ser, static_cast<int>(baudrate)), "sp_set_baudrate");
        init_flag = true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\033[1;31m" << "Motor Unable to open port" << "\033[0m" << std::endl;
        this->error_flag = true;
        close();
    }
}


serial_driver::~serial_driver()
{
    close();
}


void serial_driver::recv_1for6_42()
{
    uint16_t CRC16 = 0;
    cdc_tr_message_data_s cdc_rx_message_data{};
    while (init_flag)
    {
        cdc_tr_message_head_data_s SOF{};
        try
        {
            read_bytes(&(SOF.head), 1);
            if (SOF.head == 0xF7)      //  head
            {
                read_bytes(&(SOF.cmd), 4);
                if (SOF.crc8 == Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF)) // cmd_id
                {
                    read_bytes((uint8_t *)&CRC16, 2);
                    // 防栈溢出：帧头声明的载荷长度不能超过接收缓冲区上限。
                    // 非法/错帧（如失步、噪声把 0xF7 误判为帧头）可能带来超大 len，
                    // 无校验直接 read_bytes() 会把栈写穿导致 SIGSEGV。
                    // 这里不读取载荷、丢弃本帧，等下一个 0xF7 重新同步；
                    // 残留字节会在后续循环里逐个被消费，不会造成阻塞。
                    if (SOF.len > sizeof(cdc_rx_message_data))
                    {
                        std::cerr << "\033[1;31m" << "motor_cpp/src/serial_driver.cpp:recv_1for6_42: invalid frame len="
                                  << SOF.len << " (max " << sizeof(cdc_rx_message_data)
                                  << "), discarding & resyncing" << "\033[0m" << std::endl;
                        continue;
                    }
                    read_bytes((uint8_t *)&cdc_rx_message_data, SOF.len);
                    if (CRC16 != crc_ccitt(0xFFFF, (const uint8_t *)&cdc_rx_message_data, SOF.len))
                    {
                        memset(&cdc_rx_message_data, 0, sizeof(cdc_rx_message_data));
                    }
                    else
                    {
                        // printf("cmd %02X  ", SOF.cmd);
                        // for (int i = 0; i < SOF.len; i++)
                        // {
                        //     printf("0x%02X ", cdc_rx_message_data.data[i]);
                        // }
                        // printf("\n");

                        switch (SOF.cmd)
                        {
                        case (MODE_RESET_ZERO):
                        case (MODE_CONF_WRITE):
                            *p_mode_flag = SOF.cmd;
                            for (int i = 0; i < SOF.len; i++)
                            {
                                p_motor_id->insert(cdc_rx_message_data.data[i]);
                            }
                            break;

                        case(MODE_SET_NUM):
                            {
                                const uint8_t v_major = cdc_rx_message_data.data[2];
                                const uint8_t v_minor = cdc_rx_message_data.data[3];
                                const uint8_t v_patch = SOF.len == 4 ? 0 : cdc_rx_message_data.data[4];
                                *p_port_version = COMBINE_VERSION(v_major, v_minor, v_patch);
                            }
                            break;
                        case(MODE_FUN_V):
                            *p_fun_v = (fun_version)cdc_rx_message_data.data[0];
                            break;
                        case(MODE_MOTOR_VERSION):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_version_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_version[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->set_version(cdc_rx_message_data.motor_version[i]);
                                }
                            }
                            break;

                        case(MODE_MOTOR_STATE):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(0, 0, 
                                                    cdc_rx_message_data.motor_state[i].pos,
                                                    cdc_rx_message_data.motor_state[i].vel,
                                                    cdc_rx_message_data.motor_state[i].tqe);
                                }
                            }
                            break;
                        case(MODE_MOTOR_STATE2):
                            for (size_t i = 0; i < SOF.len / sizeof(cdc_rx_motor_state2_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state2[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(
                                                    cdc_rx_message_data.motor_state2[i].mode,
                                                    cdc_rx_message_data.motor_state2[i].fault,
                                                    cdc_rx_message_data.motor_state2[i].pos,
                                                    cdc_rx_message_data.motor_state2[i].vel,
                                                    cdc_rx_message_data.motor_state2[i].tqe);
                                }
                            }
                            break;
                        case(MODE_FDCAN_MOTOR_STATE2):
                            {
                                cdc_rx_fdcan_state_s &_p_fdcan_state = cdc_rx_message_data.fdcan_motor_state.fdcan_state;
                                
                                if (canport_error_output_flag)
                                {
                                    if (_p_fdcan_state.fault > FDCAN_STATUS_ERROR_WARNING || _p_fdcan_state.fault == FDCAN_STATUS_UNKNOWN)
                                    {
                                        ROS_ERROR("canport[%d] flaut = %d, rx = %d, tx = %d", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                    else if (_p_fdcan_state.fault == FDCAN_STATUS_ERROR_WARNING)
                                    {
                                        ROS_INFO("\033[1;32mcanport[%d] flaut = %d, rx = %d, tx = %d\033[0m", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                }
                                
                                p_fdcan_state->fault = _p_fdcan_state.fault;
                                p_fdcan_state->rx_err_num = _p_fdcan_state.rx_err_num;
                                p_fdcan_state->tx_err_num = _p_fdcan_state.tx_err_num;
                                

                                for (size_t i = 0; i < (SOF.len - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state2_s); i++)
                                {
                                    auto it = Map_Motors_p.find(cdc_rx_message_data.fdcan_motor_state.motor_state2[i].id);
                                    if (it != Map_Motors_p.end())
                                    {
                                        it->second->fresh_data(
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].mode,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].fault,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].pos,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].vel,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state2[i].tqe);
                                    }
                                }
                            }
                            break;
                        case(MODE_FDCAN_MOTOR_STATE):
                            {
                                cdc_rx_fdcan_state_s &_p_fdcan_state = cdc_rx_message_data.fdcan_motor_state.fdcan_state;
                                
                                if (canport_error_output_flag)
                                {
                                    if (_p_fdcan_state.fault > FDCAN_STATUS_ERROR_WARNING || _p_fdcan_state.fault == FDCAN_STATUS_UNKNOWN)
                                    {
                                        ROS_ERROR("canport[%d] flaut = %d, rx = %d, tx = %d", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                    else if (_p_fdcan_state.fault == FDCAN_STATUS_ERROR_WARNING)
                                    {
                                        ROS_INFO("\033[1;32mcanport[%d] flaut = %d, rx = %d, tx = %d\033[0m", Map_Motors_p.begin()->second->get_motor_belong_canport(), _p_fdcan_state.fault, _p_fdcan_state.rx_err_num, _p_fdcan_state.tx_err_num);
                                    }
                                }
                                
                                p_fdcan_state->fault = _p_fdcan_state.fault;
                                p_fdcan_state->rx_err_num = _p_fdcan_state.rx_err_num;
                                p_fdcan_state->tx_err_num = _p_fdcan_state.tx_err_num;
                                

                                for (size_t i = 0; i < (SOF.len - sizeof(cdc_rx_fdcan_state_s)) / sizeof(cdc_rx_motor_state_s); i++)
                                {
                                    auto it = Map_Motors_p.find(cdc_rx_message_data.fdcan_motor_state.motor_state[i].id);
                                    if (it != Map_Motors_p.end())
                                    {
                                        it->second->fresh_data(0,0,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].pos,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].vel,
                                                        cdc_rx_message_data.fdcan_motor_state.motor_state[i].tqe);
                                    }
                                }
                            }
                            break;
                        case(MODE_TQE_ADJS_FLAG):
                            for (size_t i = 0; i < (SOF.len / sizeof(cdc_rx_motor_flag_s)); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_flag[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->set_tqe_adjust_flag(cdc_rx_message_data.motor_flag[i].flag);
                                }
                            }
                            break;
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    // ROS_ERROR("clcl");
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << '\n';
            close();
            error_flag = true;
            break;
        }
    }
}

bool serial_driver::is_serial_error(void)
{
    return this->error_flag;
}

void serial_driver::set_run_flag(bool flag)
{
    this->init_flag = flag;
}

bool serial_driver::get_run_flag(void)
{
    return this->init_flag;
}

void serial_driver::close(void)
{
    if (_ser != nullptr)
    {
        sp_flush(_ser, SP_BUF_BOTH);
        sp_close(_ser);
        sp_free_port(_ser);
        _ser = nullptr;
    }
    init_flag = false;
}

void serial_driver::send_2(cdc_tr_message_s *cdc_tr_message)
{
    cdc_tr_message->head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message->head.data[1]), 3, 0xFF);
    cdc_tr_message->head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message->data.data[0]), cdc_tr_message->head.s.len);

    try
    {
        if (_ser != nullptr)
        {
            check_sp(sp_blocking_write(_ser,
                        reinterpret_cast<const uint8_t *>(&cdc_tr_message->head.s.head),
                        cdc_tr_message->head.s.len + sizeof(cdc_tr_message_head_s),
                        kSerialTimeoutMs),
                    "sp_blocking_write");
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        close();
        error_flag = true;
    }
}


void serial_driver::port_version_init(uint16_t *p)
{
    p_port_version = p;
}


void serial_driver::port_motors_id_init(std::unordered_set<int> *_p_motor_id, int *_p_mode_flag)
{
    p_motor_id = _p_motor_id;
    p_mode_flag = _p_mode_flag;
}


void serial_driver::init_map_motor(std::map<int, motor *> *_Map_Motors_p)
{
    Map_Motors_p = *_Map_Motors_p;
}


void serial_driver::port_fun_v_init(fun_version *_p_fun_v)
{
    p_fun_v = _p_fun_v;
}


void serial_driver::port_fdcan_state_init(cdc_rx_fdcan_state_s *_p_fdcan_state)
{
    p_fdcan_state = _p_fdcan_state;
    p_fdcan_state->fault = FDCAN_STATUS_OK;
    p_fdcan_state->rx_err_num = 0;
    p_fdcan_state->tx_err_num = 0;
}

void serial_driver::read_bytes(uint8_t *buffer, size_t size)
{
    if (_ser == nullptr || size == 0)
    {
        return;
    }
    size_t bytes_read = 0;
    while (bytes_read < size)
    {
        const sp_return r = sp_blocking_read(_ser, buffer + bytes_read, size - bytes_read, kSerialTimeoutMs);
        if (r <= 0)
        {
            throw std::runtime_error(std::string("sp_blocking_read: ") + sp_last_error_message());
        }
        bytes_read += static_cast<size_t>(r);
    }
}
