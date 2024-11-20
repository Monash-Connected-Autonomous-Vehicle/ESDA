#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__SERIAL_COMMS_HPP
#define ROS2_CONTROL_DEMO_EXAMPLE_2__SERIAL_COMMS_HPP


#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <iostream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

class SerialComms 
{
public:
    SerialComms() : timeout_ms_(1000) {} 

    std::vector<std::string> get_serial_ports(){
        std::vector<std::string> serial_ports;

        for (int i = 1; i <= 10; ++i) {
            std::string port = "COM" + std::to_string(i);
            try {
                LibSerial::SerialPort serial_port;
                serial_port.Open(port);

                if (serial_port.IsOpen()) {
                    serial_ports.push_back(port);
                    serial_port.Close();
                }
            }
            catch (const LibSerial::OpenFailed&) {
                // Port not available
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No port available. Iteration number: %d", i);
            }
        }
        return serial_ports;
    }

    void connect_to_device(const std::string &serial_device, int32_t baud_rate) {
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect_from_device() {
        serial_conn_.Close();
    }

    bool connected() const {
        return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false) {
        serial_conn_.FlushIOBuffers(); // Just in case
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try {
            // Responses end with \r\n so we will read up to (and including) the \n.
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }

        catch (const LibSerial::ReadTimeout&) {
            std::cerr << "The ReadByte() call has timed out." << std::endl ;
        }

        if (print_output) {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg() {
        std::string response = send_msg("\r");
    }

    // FOLLOWING FUNCTIONS MAY OR MAY NOT BE USED

    void read_encoder_values(int &val_1, int &val_2) {
        std::string response = send_msg("e\r");

        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());

        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
    }

    void set_motor_values(int val_1, int val_2) {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << "\r";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o) {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }


private: 
    LibSerial::BaudRate convert_baud_rate(int baud_rate)
    {
        switch (baud_rate)
        {
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cerr << "Error! Baud rate " << baud_rate << " not supported. Defaulting to 9600" << std::endl;
            return LibSerial::BaudRate::BAUD_9600;
        }
    }

    LibSerial::SerialPort serial_conn_;
    LibSerial::SerialStream my_serial_stream;

    int timeout_ms_;  // Declare the timeout_ms_ member variable
};

#endif