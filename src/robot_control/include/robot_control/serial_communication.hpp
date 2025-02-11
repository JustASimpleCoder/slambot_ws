#ifndef serial_communication_HPP
#define serial_communication_HPP

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
        bool isOpen() const;
    
        ssize_t write(const std::string& data);
        std::string read();
    
    private:
        std::string port_;
        u_int32_t baud_rate_;
        int fd_;
};

#endif