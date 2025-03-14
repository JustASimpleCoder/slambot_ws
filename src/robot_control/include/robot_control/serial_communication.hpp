#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <iostream>
#include <fcntl.h>     
#include <unistd.h>     
#include <termios.h>    
#include <cstring>  

class SerialCommunication {
    public:
        SerialCommunication(const std::string& port, int baud_rate);
        ~SerialCommunication();

        bool open();
        void close();
        bool is_open() const;
    
        ssize_t write(const std::string& data);
        std::string read();
    
    private:
        std::string m_port_;
        u_int32_t m_baud_rate_;
        int m_fd_;
};

#endif