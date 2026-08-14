#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>
#include "canboard.hpp"
#include <thread>
#include <initializer_list>
#include <fstream>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>

namespace hightorque_robot
{
    class robot
    {
    private:
        std::string robot_name, Serial_Type;
        /// 控制盒 USB 选择：auto=自动检测（只允许一个控制盒）；或指定 USB 路径
        /// （支持 1-1.2 / usb-0:1.2 / pci-...-usb-0:1.2 任一种，子串匹配）
        std::string usb_select_ = "auto";
        int CANboard_num, Seial_baudrate;
        std::vector<canboard> CANboards;
        std::vector<std::string> str;
        std::string SDK_version2 = "4.6.0"; // SDK版本
        std::condition_variable error_check_cv;
        std::mutex error_check_mutex;
        bool error_check_flag = false;
        std::thread error_check_thread_;
        fun_version fun_v = fun_v1;
        uint16_t slave_v = COMBINE_VERSION(3, 0, 0);
        bool canport_error_output_flag = false;
        bool board_special_flag = false;
    public:
        std::vector<serial_driver *> ser;
        std::vector<motor *> Motors;
        std::vector<canport *> CANPorts;
        std::vector<std::thread> ser_recv_threads;
        int motor_position_limit_flag = 0;
        int motor_torque_limit_flag = 0;

        RobotParams robot_params;

        int motor_timeout_ms = 0;

        robot();
        /// @param config_path 电机 YAML 路径
        /// @param usb_select  控制盒选择："auto" 或 USB 路径（如 "1-1.2" / "usb-0:1.2"）
        robot(const std::string& config_path, const std::string& usb_select = "auto");
        ~robot();

    private:
        void init_robot(const std::string& config_path);
        int serial_pid_vid(const char *name, int *pid, int *vid);
        int serial_pid_vid(const char *name);
        std::vector<std::string> list_serial_ports(const std::string& full_prefix);
        void init_ser();

        // ==================== 控制盒 USB 选择（usb_select）====================
        /// 一个 tty 口对应的 USB 标识（多种表示，用于子串匹配与报错提示）
        struct UsbId
        {
            std::string kernels;  // 设备 KERNELS，如 "1-1.2"
            std::string id_path;  // udev ID_PATH 风格（不含接口段），如 "usb-0:1.2"
            std::string syspath;  // /sys/class/tty 的 realpath 全串
        };
        /// 通过 /sys/class/tty 解析 tty 设备在 USB 总线上的标识
        UsbId usb_id_of(const std::string& dev) const;
        /// usb_select 与端口是否匹配（子串匹配）
        bool usb_select_matches(const UsbId& id, const std::string& select) const;
        /// 按 usb_select 过滤候选端口；auto 时检测控制盒数量（>1 报错退出）
        std::vector<std::string> select_usb_ports(const std::vector<std::string>& candidates);
        /// 打印当前所有控制盒（报错提示用）
        void print_usb_boxes(const std::vector<std::string>& candidates) const;
        void check_error();
        int check_serial_dev_exist(int);        
        void set_port_motor_num();
    public:

        void detect_motor_limit();
        void motor_send_cmd();       
        void send_get_motor_state_cmd();
        void send_get_motor_version_cmd();
        void check_motor_connection_position();
        void check_motor_connection_version();
        void set_brake();
        void set_stop();
        void set_reset();
        void set_reset_zero();
        void set_reset_zero(std::initializer_list<int> motors);
        void set_motor_runzero();
        void set_timeout(int16_t t_ms);
        void set_timeout(uint8_t portx, int16_t t_ms);
        void motor_version_detection();
        void set_data_reset();
        void canboard_bootloader();
        void canboard_fdcan_reset();
        void get_motor_tqe_adjs_flag();
        void send_get_tqe_adjust_flag_cmd();
        void check_tqe_adjust_flag();
    };
}
#endif
