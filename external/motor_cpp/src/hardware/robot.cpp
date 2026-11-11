#include "robot.hpp"
#include "parse_robot_params.hpp"
#include <unistd.h>
#include <limits.h>
#include <cctype>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <iomanip>

namespace hightorque_robot
{
    robot::robot()
    {
        init_robot("../robot_param/robot_config.yaml");
    }

    robot::robot(const std::string& config_path, const std::string& usb_select)
        : usb_select_(usb_select)
    {
        init_robot(config_path);
    }

    void robot::init_robot(const std::string& config_path)
    {
        auto config = YAML::LoadFile(config_path);
        std::string config_robot_name = "unknown";
        if (config["robot"]) {
            if (config["robot"]["robot_name"]) {
                config_robot_name = config["robot"]["robot_name"].as<std::string>();
            } else if (config["robot"]["name"]) {
                config_robot_name = config["robot"]["name"].as<std::string>();
            }
        }
        std::cout << "robot_config: " << config_robot_name << std::endl;
        auto param_file = config_path;
        if (config["robot"]["param_file"])
        {
            param_file = config["robot"]["param_file"].as<std::string>();
        }
        
        // 如果param_file是相对路径，则相对于config_path的目录
        if (param_file[0] != '/') {
            size_t last_slash = config_path.find_last_of('/');
            if (last_slash != std::string::npos) {
                std::string config_dir = config_path.substr(0, last_slash + 1);
                param_file = config_dir + param_file;
            }
        }
        
        robot_params  = parseRobotParams(param_file);
        Seial_baudrate = robot_params.Seial_baudrate;
        robot_name = robot_params.robot_name;
        motor_timeout_ms = robot_params.motor_timeout_ms;
        CANboard_num = robot_params.CANboard_num;
        Serial_Type = robot_params.Serial_Type;
        canport_error_output_flag = robot_params.canport_error_output_flag;
        board_special_flag = robot_params.board_special_flag;

        if (motor_timeout_ms < 0 || motor_timeout_ms > 32760)
        {
            ROS_ERROR("The value of motor_timeout_ms is out of the valid range [0, 32760]");
            exit(-1);
        }
        
        std::cout << "\033[1;32mGot params SDK_version: v" << SDK_version2 << "\033[0m" << std::endl;
        std::cout << "\033[1;32mThe robot name is " << robot_name << "\033[0m" << std::endl;
        std::cout << "\033[1;32mThe robot has " << CANboard_num << " CANboards\033[0m" << std::endl;
        std::cout << "\033[1;32mThe Serial type is " << Serial_Type << "\033[0m" << std::endl;

        init_ser();
        auto it = robot_params.CANboards.begin();
        for (int i = 1; i <= CANboard_num; i++, it++)
        {
            CANboards.push_back(canboard(i, &ser, it->second, canport_error_output_flag));
        }

        for (canboard &cb : CANboards)
        {
            cb.push_CANport(&CANPorts);
        }
        for (canport *cp : CANPorts)
        {
            cp->puch_motor(&Motors);
        }
        set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
        if (slave_v >= COMBINE_VERSION(4, 1, 0))
        {
            canboard_fdcan_reset();
        }

        if (slave_v < COMBINE_VERSION(4, 0, 0))  // 检测电机连接是否正常
        {
            fun_v = fun_v1;
            check_motor_connection_position();   
        }
        else
        {
            fun_v = fun_v2;
            check_motor_connection_version();
        }

        if (motor_timeout_ms != 0)
        {
            set_timeout(motor_timeout_ms);
        }
        
        send_get_motor_state_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        std::cout << "\033[1;32mThe robot has " << Motors.size() << " motors\033[0m" << std::endl;
        error_check_flag = true;
        error_check_thread_ = std::thread(&robot::check_error, this);
        std::cout << "robot init" << std::endl;
    }


    void robot::stop_reconnect()
    {
        {
            std::lock_guard<std::mutex> lock(error_check_mutex);
            error_check_flag = false;
        }
        error_check_cv.notify_all();
        if (error_check_thread_.joinable()) { error_check_thread_.join(); }
    }

    void robot::pause_background()
    {
        // Stop the object-mutating recovery worker before touching receivers.
        stop_reconnect();
        for (auto *s : ser) { if (s) { s->set_run_flag(false); } }
        for (auto &thread : ser_recv_threads) { if (thread.joinable()) { thread.join(); } }
        ser_recv_threads.clear();
        std::cout << "SDK_BACKGROUND_PAUSED" << std::endl;
    }

    void robot::resume_background()
    {
        if (communication_fault()) { throw std::runtime_error("Refusing resume after communication fault"); }
        if (error_check_thread_.joinable()) { return; }
        if (ser.empty() || Motors.empty())
        {
            throw std::runtime_error("Refusing resume without configured ports and motors");
        }
        for (auto *s : ser)
        {
            if (!s || s->is_serial_error())
            {
                throw std::runtime_error("Refusing resume after serial fault; reconfigure hardware explicitly");
            }
        }
        if (ser_recv_threads.empty())
        {
            for (auto *s : ser)
            {
                s->set_run_flag(true);
                ser_recv_threads.emplace_back(&serial_driver::recv_1for6_42, s);
            }
        }
        error_check_flag = true;
        error_check_thread_ = std::thread(&robot::check_error, this);
    }

    robot::~robot()
    {
        // Receiver/recovery threads must not close/replace ports during release.
        pause_background();
        try
        {
            if (communication_fault())
            {
                std::cerr << "SDK_FAULT_CLOSE_NO_MOTION: no stop/reset/brake retry" << std::endl;
            }
            else if (robot_params.exit_motor_brake_flag)
            {
                std::vector<uint64_t> before;
                for (auto *s : ser) { before.push_back(s ? s->completed_write_count() : 0); }
                std::cout << "SDK_BRAKE_WRITE_ATTEMPT" << std::endl;
                set_brake(); set_brake(); set_brake();
                bool ok = !ser.empty() && !Motors.empty();
                for (size_t i = 0; i < ser.size(); ++i)
                {
                    ok = ok && ser[i] && !ser[i]->is_serial_error() &&
                        ser[i]->completed_write_count() >= before[i] + 3;
                }
                std::cout << (ok ? "SDK_BRAKE_WRITES_COMPLETED_NO_DEVICE_ACK" : "SDK_BRAKE_WRITE_UNCONFIRMED") << std::endl;
            }
            else
            {
                set_stop(); set_stop(); set_reset();
                std::cout << "motor stop" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "SDK_RELEASE_ERROR: " << e.what() << std::endl;
        }
        for (auto *s : ser) { delete s; }
        ser.clear();
        std::cout << "SDK_SERIAL_CLOSED" << std::endl;
    }


    void robot::detect_motor_limit()
    {
        // 电机正常运行时检测是否超过限位，停机之后不检测
        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (motor *m : Motors)
            {
                if(m->pos_limit_flag)
                {                    
                    std::cerr << "\033[1;31m" << "robot pos limit, motor stop." << "\033[0m" << std::endl;
                    set_stop();
                    motor_position_limit_flag = m->pos_limit_flag;
                    break;
                }

                if(m->tor_limit_flag)
                {
                    std::cerr << "\033[1;31m" << "robot torque limit, motor stop." << "\033[0m" << std::endl;
                    set_stop();
                    motor_torque_limit_flag = m->tor_limit_flag;
                    break;
                }
            }            
        }
    }

    void robot::motor_send_cmd()
    {
        if (communication_fault()) { throw std::runtime_error("SDK communication fault: command rejected"); }
        if(!motor_position_limit_flag && !motor_torque_limit_flag)
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send_cmd();
            }
        }
        
    }


    int robot::serial_pid_vid(const char *name, int *pid, int *vid)
    {
        int r = 0;
        struct sp_port *port;
        try
        {
            /* code */
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, vid, pid) != SP_OK) 
            {
                r = 1;
            } 
            std::cout << "Port: " << name << ", PID: 0x" << std::hex << *pid << ", VID: 0x" << *vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << "\033[1;31m" << e.what() << "\033[0m" << '\n';
            sp_close(port);
            sp_free_port(port);
        }
        return r;
    }


    int robot::serial_pid_vid(const char *name)
    {
        int pid, vid;
        int r = 0;
        struct sp_port *port;
        try
        {
            sp_get_port_by_name(name, &port);
            if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
            {
                r = -1;
            } 
            else 
            {
                if (pid == 0xFFFF)
                {
                    if (board_special_flag)
                    {
                        if (vid == 0xFAE1)
                        {
                            r = 1;
                        }
                        else
                        {
                            r = -3;
                        }
                    }
                    else
                    {
                        switch (vid)
                        {
                        case (0xCAF1):
                        case (0xCAE1):
                            r = 1;
                            break;
                        default:
                            r = -3;
                            break;
                        }
                    }
                }
                else
                {
                    r = -1;
                }
            }
            // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

            // 关闭端口
            sp_free_port(port);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            r = -3;
            sp_close(port);
            sp_free_port(port);
        }

        return r;
    }


    std::vector<std::string> robot::list_serial_ports(const std::string& full_prefix) 
    {
        std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
        std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
        std::vector<std::string> serial_ports;
        DIR *directory;
        struct dirent *entry;

        directory = opendir(base_path.c_str());
        if (!directory)
        {
            std::cerr << "\033[1;31m << Could not open the directory " << base_path << "\033[0m" << std::endl;
            return serial_ports; 
        }

        while ((entry = readdir(directory)) != NULL)
        {
            std::string entryName = entry->d_name;
            if (entryName.find(prefix) == 0)
            { 
                serial_ports.push_back(base_path + entryName);
            }
        }

        closedir(directory);

        // 自然序（数值序）排序而非 reverse / 字典序：readdir 顺序由 devtmpfs 哈希
        // 决定（重启可能变化）；字典序会让 ttyACM10 < ttyACM2（两位数口号时失效）。
        // 数值序后 ttyACM0 < ttyACM1 < ... < ttyACM13（符号链接 ht_gc_0 < ht_gc_1
        // 同理），serial_id 1,2 恒对应控制盒接口 0,1（接线通道），左右臂顺序确定。
        std::sort(serial_ports.begin(), serial_ports.end(),
            [](const std::string& a, const std::string& b) {
                std::string na = a.substr(a.rfind('/') + 1);
                std::string nb = b.substr(b.rfind('/') + 1);
                size_t ia = 0, ib = 0;
                while (ia < na.size() && ib < nb.size())
                {
                    char ca = na[ia], cb = nb[ib];
                    if (std::isdigit((unsigned char)ca) && std::isdigit((unsigned char)cb))
                    {
                        size_t sa = ia, sb = ib;
                        while (sa < na.size() && std::isdigit((unsigned char)na[sa])) sa++;
                        while (sb < nb.size() && std::isdigit((unsigned char)nb[sb])) sb++;
                        unsigned long va = std::stoul(na.substr(ia, sa - ia));
                        unsigned long vb = std::stoul(nb.substr(ib, sb - ib));
                        if (va != vb) return va < vb;
                        ia = sa; ib = sb;
                    }
                    else
                    {
                        if (ca != cb) return ca < cb;
                        ia++; ib++;
                    }
                }
                return na.size() < nb.size();
            });

        return serial_ports;
    }


    robot::UsbId robot::usb_id_of(const std::string& dev) const
    {
        UsbId id;
        char rp[PATH_MAX];
        // 支持符号链接（如 /dev/ht_gc_0）：先解析到真实 /dev/ttyACMx
        if (realpath(dev.c_str(), rp) == nullptr)
            return id;
        std::string real = rp;
        std::string name = real;
        if (name.rfind("/dev/", 0) == 0)
            name = name.substr(5);   // ttyACM3
        std::string link = "/sys/class/tty/" + name + "/device";
        if (realpath(link.c_str(), rp) == nullptr)
            return id;
        id.syspath = rp;
        // 例: /sys/devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1.2/1-1.2:1.0/tty/ttyACM0
        // 从后往前找形如 "1-1.2"（数字-数字[.数字...]，无冒号）的段 = USB 设备自身
        std::stringstream ss(id.syspath);
        std::string seg;
        std::vector<std::string> segs;
        while (std::getline(ss, seg, '/'))
            if (!seg.empty())
                segs.push_back(seg);
        bool usb_before = false;
        for (int i = (int)segs.size() - 1; i >= 0; --i)
        {
            const std::string& s = segs[i];
            bool shape_ok = !s.empty() && std::isdigit((unsigned char)s[0])
                && s.find('-') != std::string::npos && s.find(':') == std::string::npos;
            if (shape_ok)
            {
                bool all_ok = true;
                for (char c : s)
                    if (!std::isdigit((unsigned char)c) && c != '-' && c != '.')
                    { all_ok = false; break; }
                if (all_ok)
                {
                    for (int j = i - 1; j >= 0; --j)
                        if (segs[j].rfind("usb", 0) == 0) { usb_before = true; break; }
                    if (usb_before)
                    {
                        id.kernels = s;
                        size_t dash = s.find('-');
                        id.id_path = "usb-0:" + s.substr(dash + 1);
                    }
                    break;
                }
            }
        }
        return id;
    }


    bool robot::usb_select_matches(const UsbId& id, const std::string& select) const
    {
        if (select.empty() || select == "auto")
            return true;
        // 三种表示任一种双向子串匹配即命中：
        //   kernels: "1-1.2"           <- udev KERNELS
        //   id_path: "usb-0:1.2"       <- udev ID_PATH（不含接口段）
        //   syspath: 完整 sysfs 路径    <- /sys/class/tty 的 realpath
        const std::string* haystacks[] = { &id.kernels, &id.id_path, &id.syspath };
        for (const std::string* h : haystacks)
        {
            if (h->empty())
                continue;
            if (h->find(select) != std::string::npos)
                return true;
            if (select.find(*h) != std::string::npos)
                return true;
        }
        return false;
    }


    void robot::print_usb_boxes(const std::vector<std::string>& candidates) const
    {
        std::cout << "检测到的控制盒端口（供 usb_select 选择）:" << std::endl;
        for (const std::string& p : candidates)
        {
            UsbId id = usb_id_of(p);
            std::cout << "  " << p << "  usb=" << (id.kernels.empty() ? "?" : id.kernels)
                      << "  ID_PATH=" << (id.id_path.empty() ? "?" : id.id_path) << std::endl;
        }
        std::cout << "可通过启动参数指定控制盒：xacro_usb_select:=<路径>（如 1-1.2 或 usb-0:1.2）"
                  << std::endl;
    }


    std::vector<std::string> robot::select_usb_ports(const std::vector<std::string>& candidates)
    {
        if (usb_select_ == "auto")
        {
            // 统计控制盒数量：按 USB 设备 sysname 去重（一个控制盒 = 一个 USB 设备）
            std::vector<std::string> boxes;
            for (const std::string& p : candidates)
            {
                UsbId id = usb_id_of(p);
                if (id.kernels.empty())
                    continue;
                if (std::find(boxes.begin(), boxes.end(), id.kernels) == boxes.end())
                    boxes.push_back(id.kernels);
            }
            if (boxes.empty())
            {
                std::cerr << "\033[1;31m未找到可用的控制盒串口（USB 连接 / 供电 / 权限）\033[0m" << std::endl;
                exit(-1);
            }
            if (boxes.size() > 1)
            {
                std::cerr << "\033[1;31m检测到 " << boxes.size()
                          << " 个控制盒，请用 xacro_usb_select:= 指定本启动要连接的控制盒\033[0m"
                          << std::endl;
                print_usb_boxes(candidates);
                exit(-1);
            }
            return candidates;  // 只有一个控制盒：全部候选口都属于它
        }
        // 指定路径：只保留匹配的控制盒端口
        std::vector<std::string> filtered;
        for (const std::string& p : candidates)
        {
            UsbId id = usb_id_of(p);
            if (usb_select_matches(id, usb_select_))
                filtered.push_back(p);
        }
        if (filtered.empty())
        {
            std::cerr << "\033[1;31musb_select=" << usb_select_
                      << " 未匹配到任何端口（请用 udevadm info -n /dev/ttyACMx 查看路径）\033[0m"
                      << std::endl;
            print_usb_boxes(candidates);
            exit(-1);
        }
        return filtered;
    }


    void robot::init_ser()
    {
        ser.clear();
        ser_recv_threads.clear();
        str.clear();   
        std::vector<std::string> ports = list_serial_ports(Serial_Type);
        std::vector<std::string> filtered;
        std::cout << "Serial Port List: " << std::endl;
        for (const std::string& port : ports) 
        {   
            const int8_t r = serial_pid_vid(port.c_str());
            if (r > 0)
            {
                std::cout << "Serial Port" << filtered.size() << " = " << port << std::endl;
                filtered.push_back(port);
            }
        }

        // 控制盒选择（usb_select）：auto=仅允许 1 个控制盒（多于 1 个报错退出并列出）；
        // 指定路径时只保留该控制盒的端口。多套机械臂同机时各启动指定自己的控制盒。
        std::cout << "\033[1;32musb_select=" << usb_select_ << "\033[0m" << std::endl;
        filtered = select_usb_ports(filtered);
        std::cout << "Using " << filtered.size() << " serial port(s)" << std::endl;
        str = filtered;

        for(auto board_params: robot_params.CANboards)
        {
            std::vector<int> serial_id_old;
            for(auto port_params: board_params.second.CANports)
            {
                int serial_id = port_params.second.serial_id;

                serial_id_old.push_back(serial_id);

                if (serial_id - 1 >= (int)str.size())
                {
                    std::cerr << "\033[1;31mserial_id " << serial_id
                              << " 超出可用端口数 " << str.size()
                              << "（控制盒接线通道不足，或 usb_select 选错了控制盒）\033[0m"
                              << std::endl;
                    exit(-1);
                }
                serial_driver *s = new serial_driver(&str[serial_id - 1], Seial_baudrate, canport_error_output_flag);
                ser.push_back(s);
                ser_recv_threads.push_back(std::thread(&serial_driver::recv_1for6_42, s));
            }
        }
    }

    bool robot::communication_fault()
    {
        if (ser.empty()) { return true; }
        for (auto* s : ser)
        {
            if (!s || s->is_serial_error())
            {
                return true;
            }
        }
        return false;
    }

    void robot::check_error()
    {
        // Never clear/rebuild objects used by the command/feedback threads.
        while (error_check_flag.load())
        {
            if (communication_fault())
            {
                std::cerr << "SDK_TRANSPORT_UNAVAILABLE: serial I/O failed; no automatic object rebuild" << std::endl;
                return;
            }
            std::unique_lock<std::mutex> lock(error_check_mutex);
            error_check_cv.wait_for(lock, std::chrono::milliseconds(10),
                                   [this]{ return !error_check_flag.load(); });
        }
    }

    int robot::check_serial_dev_exist(int file_num)
    {
        int exist_num = 0;
        std::cout << "check serial dev exist" << std::endl;
        std::vector<std::string> dev_vec;
        for (int i = 0; i < file_num; i++)
        {
            std::string _dev = std::string("/dev/ttyACM") + std::to_string(i);
            std::cout << "check: " << _dev << std::endl;
            dev_vec.push_back(_dev);
        }
        for (auto &dev : dev_vec)
        {
            if(access(dev.c_str(),F_OK) == 0)
            {
                exist_num++;
                std::cout << "exist: " << dev << std::endl;
            }
        }
        std::cout << "exist_num = " << exist_num << std::endl;
        return exist_num;
    }


    /**
     * @brief 设置每个通道的电机数量，并查询主控板固件版本
     */
    void robot::set_port_motor_num()
    {
        for (canboard &cb : CANboards)
        {
            slave_v = cb.set_port_motor_num();
        }
    }
    
    
    size_t robot::diagnostic_state_query()
    {
        // IO owner only. Retain all sticky faults; no reconnect, enable, or motion.
        size_t written = 0;
        for (auto* port : CANPorts)
            if (port && port->diagnostic_state_query()) ++written;
        return written;
    }

    void robot::send_get_motor_state_cmd()
    {
        if (fun_v >= fun_v4)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd2();
            }
        }
        else if (fun_v >= fun_v2)
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd();
            }
        }
        else
        {
            for (motor *m : Motors)
            {
                m->velocity(0.0f);
            }
            motor_send_cmd();
        }
    }


    void robot::send_get_motor_version_cmd()
    {
        if (slave_v < COMBINE_VERSION(4, 0, 0))
        {
            std::cerr << "\033[1;31m << The current communication board does not support this function!!! << \033[0m" << std::endl;
            exit(0);
        }

        for (canboard &cb : CANboards)
        {
            cb.send_get_motor_version_cmd();
        }
    }


    void robot::motor_version_detection()
    {
        uint16_t v_min = 0xFFFF;
        uint16_t v_max = 0;
        uint16_t i = 0;

        std::cout << "---------------motor version---------------------" << std::endl;
        for (motor *m : Motors)
        {
            const auto v = m->get_version();
            printf("motors[%02d]: id:%02d v%d.%d.%d\r\n", i++, v->id, v->major, v->minor, v->patch);
            const uint16_t v_new = COMBINE_VERSION(v->major, v->minor, v->patch);
            if (v_min > v_new && v_new != 0)
            {
                v_min = v_new;
            }

            if (v_max < v_new)
            {
                v_max = v_new;
            }
        }
        std::cout << "-------------------------------------------------" << std::endl;

        if (v_min >= COMBINE_VERSION(4, 4, 6))
        {
            fun_v = fun_v5;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 3))
        {
            fun_v = fun_v4;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 2))
        {
            fun_v = fun_v3;
        }
        else if (v_min >= COMBINE_VERSION(4, 2, 0))
        {
            fun_v = fun_v2;
        }
        else
        {
            fun_v = fun_v1;
        }

        for (canboard &cb : CANboards)
        {
            cb.set_fun_v(fun_v, v_min);
        }

        printf("fun_v = %d, motor_min_v = %d.%d.%d, motor_max_v = %d.%d.%d\n", fun_v, 
            GET_MAJOR_VERSION(v_min), GET_MINOR_VERSION(v_min), GET_PATCH_VERSION(v_min),
            GET_MAJOR_VERSION(v_max), GET_MINOR_VERSION(v_max), GET_PATCH_VERSION(v_max));
        
        if (slave_v < COMBINE_VERSION(4, 7, 0) || v_max < COMBINE_VERSION(4, 6, 0))
        {
            return;
        }

        check_tqe_adjust_flag();
    }

    void robot::send_get_tqe_adjust_flag_cmd()
    {
        for (canboard &cb : CANboards)
        {
            cb.send_get_tqe_adjust_flag_cmd();
        }
    }


    void robot::check_tqe_adjust_flag()
    {
        int t = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

        printf("Check the torque adjustment mark\n");
        while (t++ < 20)
        {
            send_get_tqe_adjust_flag_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                cdc_rx_motor_version_s* v = m->get_version();
                uint8_t tqe_flag = m->get_tqe_adjust_flag();

                if (COMBINE_VERSION(v->major, v->minor, v->patch) >= COMBINE_VERSION(4, 6, 0) && tqe_flag == 0xFF)
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
                else if (tqe_flag == 1)
                {
                    m->set_type(mNone);
                }
            }

            if (id.size() == 0)
            {
                break;
            }

            if (t % 5 == 0)
            {
                ROS_INFO(".");
            }
        }

        if (id.size() != 0)
        {
            for (size_t i = 0; i < id.size(); i++)
            {
                ROS_ERROR("CANboard(%d) CANport(%d) id(%d) Motor Failed to read torque adjust flag!!!", board[i], port[i], id[i]);
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }


    void robot::check_motor_connection_version()
    {
        int t = 0;
        size_t num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;

        std::cout << "Detecting motor connection" << std::endl;
        while (t++ < 20)
        {
            send_get_motor_version_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                cdc_rx_motor_version_s *v = m->get_version();
                if (v->major != 0)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 100 == 0)
            {
                std::cout << "." << std::endl;
            }
        }

        if (num == Motors.size())
        {
            std::cout << "\033[1;32mAll motor connections are normal\033[0m" << std::endl;
        }
        else
        {
            for (size_t i = 0; i < Motors.size() - num; i++)
            {
                std::cerr << "\033[1;31m" << "CANboard(" << board[i] << ") CANport(" << port[i] << ") id(" << id[i] << ") Motor connection disconnected!!!" << "\033[0m" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        motor_version_detection();
    }


    void robot::check_motor_connection_position()
    {
        int t = 0;
        size_t num = 0;
        std::vector<int> board;
        std::vector<int> port;
        std::vector<int> id;
        ROS_INFO("Detecting motor connection");
        while (t++ < 2000)
        {
            send_get_motor_state_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            num = 0;
            std::vector<int>().swap(board);
            std::vector<int>().swap(port);
            std::vector<int>().swap(id);
            for (motor *m : Motors)
            {
                if (m->get_current_motor_state()->position != 999.0f)
                {
                    ++num;
                }
                else
                {
                    board.push_back(m->get_motor_belong_canboard());
                    port.push_back(m->get_motor_belong_canport());
                    id.push_back(m->get_motor_id());
                }
            }

            if (num == Motors.size())
            {
                break;
            }

            if (t % 1000 == 0)
            {
                std::cout << "." << std::endl;
            }
        }

        if (num == Motors.size())
        {
            std::cout << "\033[1;32mAll motor connections are normal\033[0m" << std::endl;
        }
        else
        {
            for (size_t i = 0; i < Motors.size() - num; i++)
            {
                std::cerr << "\033[1;31m" << "CANboard(" << board[i] << ") CANport(" << port[i] << ") id(" << id[i] << ") Motor connection disconnected!!!" << "\033[0m" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void robot::set_brake()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_brake();
        }
        motor_send_cmd();
    }

    void robot::set_stop()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_stop();
        }
        motor_send_cmd();
    }


    void robot::set_reset()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }


    void robot::set_reset_zero()
    {
        for (canboard &cb : CANboards)
        {
            cb.set_reset_zero();
        }
    }


    void robot::set_reset_zero(std::initializer_list<int> motors)
    {
        for (auto const &motor : motors)
        {
            
            int board_id = Motors[motor]->get_motor_belong_canboard() - 1;
            int port_id = Motors[motor]->get_motor_belong_canport() - 1;
            int motor_id = Motors[motor]->get_motor_id();
            std::cout << board_id << ", " << port_id << ", " << motor_id << std::endl;

            set_reset();
            
            std::cout << "Motor " << motor << " settings have been successfully restored. Initiating zero position reset." << std::endl;
            if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
            {
                std::cout << "Motor " << motor << " reset to zero position successfully, awaiting settings save." << std::endl;
                if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
                {
                    std::cout << "Motor " << motor << " settings saved successfully." << std::endl;
                }
                else
                {
                    std::cerr << "\033[1;31m << Motor " << motor << " settings saved failed. << \033[0m" << std::endl;
                }
            }
            else
            {
                std::cerr << "Motor " << motor << " reset to zero position failed." << std::endl;
            }
        }
    }


    void robot::set_timeout(int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            for (canboard &cb : CANboards)
            {
                cb.set_time_out(t_ms);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void robot::set_timeout(uint8_t portx, int16_t t_ms)
    {
        for (int i = 0; i < 5; i++)
        {
            CANboards[0].set_time_out(portx, t_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void robot::canboard_bootloader()
    {
        for (canboard &cb : CANboards)
        {
            cb.canboard_bootloader();
        }
    }

    void robot::canboard_fdcan_reset()
    {
        std::cout << "canboard fdcan reset" << std::endl;
        for (canboard &cb : CANboards)
        {
            cb.canboard_fdcan_reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
