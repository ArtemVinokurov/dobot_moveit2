/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/10
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <cerrno>
#include <cstring>
#include <utility>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <dobot_bringup/tcp_socket.hpp>

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)
{
}

TcpClient::~TcpClient()
{
    close();
}

void TcpClient::close()
{
    if (fd_ >= 0)
    {
        shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
        RCLCPP_INFO(getLogger(), "close tcp connection");
    }
}

void TcpClient::connect()
{
    if (fd_ < 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};

    memset(&addr, 0, sizeof(addr));
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr);

    if (::connect(fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(errno));
    }
    is_connected_ = true;
    RCLCPP_INFO(getLogger(), "%s : connect successfully", toString().c_str());
    // ROS_INFO("%s : connect successfully", toString().c_str());
}

void TcpClient::disConnect()
{
    RCLCPP_INFO(getLogger(), "disConnect");
    if (is_connected_)
    {
        shutdown(fd_, SHUT_RDWR);
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

bool TcpClient::isConnect() const
{
    return is_connected_;
}

void TcpClient::tcpSend(const void* buf, uint32_t len)
{
    //RCLCPP_INFO(getLogger(), "send...");
    if (!is_connected_) {
        RCLCPP_INFO(getLogger(), "send fail1");
        throw TcpClientException(toString() + std::string(" ::send(blabla) ") + strerror(errno));
    }
        
    
    // RCLCPP_INFO(getLogger(), "send");
    // ROS_INFO("send : %s", (const char*)buf);

    const auto* tmp = (const uint8_t*)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, 0);
        if (err < 0)
        {
            RCLCPP_INFO(getLogger(), "send fail2");
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void* buf, uint32_t len, uint32_t& has_read, uint32_t timeout)
{
    uint8_t* tmp = (uint8_t*)buf;    // NOLINT(modernize-use-auto)

    fd_set read_fds;
    timeval tv = { 0, 0 };

    has_read = 0;
    while (len)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        int err = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (err < 0)
        {
            RCLCPP_INFO(getLogger(), "select");
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        else if (err == 0)
        {
            return false;
        }
        err = (int)::recv(fd_, tmp, len, 0);
        //err = (int)::read(fd_, tmp, len);
        if (err < 0)
        {
            RCLCPP_INFO(getLogger(), "read");
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        else if (err == 0)
        {
            //disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }
        len -= err;
        tmp += err;
        has_read += err;
    }

    return true;
}

std::string TcpClient::toString()
{
    return ip_ + ":" + std::to_string(port_);
}


rclcpp::Logger TcpClient::getLogger() {
  return rclcpp::get_logger("tcp_socket");
}