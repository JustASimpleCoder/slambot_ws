
#include "robot_control/serial_communication.hpp"

SerialCommunication::SerialCommunication(const std::string& port, int baud_rate) 
    :   port_(port), baud_rate_(baud_rate), fd_(-1) {}

SerialCommunication::~SerialCommunication() {
        if (isOpen()) {
            close();
        }
    }
    
bool SerialCommunication::open() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        std::cerr << "Failed to open serial port: " << port_ << std::endl;
        return false;
    }

    // Configure the serial port
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Error getting terminal attributes" << std::endl;
        return false;
    }

    // Set baud rate
    speed_t speed;
    switch (baud_rate_) {
        case 9600:   
            speed = B9600;   
            break;
        case 19200:  
            speed = B19200;  
            break;
        case 38400:  
            speed = B38400;  
            break;
        case 57600:  
            speed = B57600;  
            break;
        case 115200: 
            speed = B115200; 
            break;
        default:
            speed = B9600;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Set other serial port settings
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;  // Clear data size bits
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT, and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST; // Disable output processing
    tty.c_oflag &= ~ONLCR; // Disable conversion of newline to carriage return/line feed

    // Set timeouts
    tty.c_cc[VMIN] = 0;  // Non-blocking read
    tty.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting terminal attributes" << std::endl;
        return false;
    }

    std::cout << "Serial port opened successfully: " << port_ << std::endl;
    return true;
}

void SerialCommunication::close() {
    if (isOpen()) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "Serial port closed: " << port_ << std::endl;
    }
}

bool SerialCommunication::isOpen() const {
    return fd_ != -1;
}

ssize_t SerialCommunication::write(const std::string& data) {
    if (!isOpen()) {
        std::cerr << "Serial port is not open" << std::endl;
        return -1;
    }
    return ::write(fd_, data.c_str(), data.size());
}

std::string SerialCommunication::read() {
    if (!isOpen()) {
        std::cerr << "Serial port is not open" << std::endl;
        return "";
    }

    char buffer[256];
    ssize_t n = ::read(fd_, buffer, sizeof(buffer) - 1);
    if (n > 0) {
        buffer[n] = '\0';
        return std::string(buffer);
    }
    return "";
}
