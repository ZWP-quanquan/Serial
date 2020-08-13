#pragma once
#ifdef _MSC_VER
#pragma warning (push)
#pragma warning (disable : 4005)
#include <intsafe.h>
#include <stdint.h>
#pragma warning (pop)
#endif

#include <cstdlib>
#include <deque>
#include <iostream>
#include <boost/asio.hpp>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "CommMsg.h"

using namespace boost::asio;
using std::string;

namespace CP
{
typedef std::deque<CommMsg> CommMsg_queue;
typedef std::deque<unsigned char *> Msg_queue;

class Serial_Asio
{
public:
    Serial_Asio(void);
    virtual ~Serial_Asio(void);

    //初始化串口(输入打开串口名,波特率及数据位)
    //返回false打开串口失败
    bool Init_Port(const string port, size_t BaudRate, size_t ByteSize);

    //错误信息处理
    virtual bool ErrorHandle();

    //关闭端口
    void Close();

    //发送数据
    void SendMsg(const CommMsg& msg);

    //读取数据
    //查看是否有数据可读
    bool isempty();
    //获取数据
    unsigned char* GetMsg();

    //端口状态
    //串口开关返回true,串口状态开,返回false,串口状态关
    bool PortSts();
    //获取串口名
    string GetPortName() const;
    //获取波特率
    size_t GetBaudRate() const;
    //获取数据位
    size_t GetByteSize() const;
    //获取系统错误信息
    string GetErrorCode() const;

    

protected:
    //初始化串口规则,包括帧头解析方法decode_header,数据校验方法decode_check
    //具体需要协议继承CommMsg类
    void Init_Msg(CommMsg *pMsg);

    bool err;                            //发生错误标识
    boost::system::error_code ec;        //错误信息
    boost::asio::io_service m_ios;       //I/O服务
    boost::asio::serial_port *m_pPort;   //串口
    string m_strCom;                     //串口号
    size_t m_BaudRate;                   //波特率
    size_t m_ByteSize;                   //数据位

    boost::shared_mutex read_;           //消息队列读写互斥锁
    boost::thread io_thread;             //串口线程

    int disloc_;                         //错位位置,记录当前帧头的下标
    int msg_offset;                      //帧头误差偏移量

    bool open_;                          //串口状态,用于区别去主动关闭串口还是被动关闭串口

    string ErrMsg;                       //错误信息

    int nNumber;                         //测试保留参数
private:
    //接收数据处理
    void handle_read_header(const boost::system::error_code& error,size_t nBytesReceived);
    void handle_read_body(const boost::system::error_code& error,size_t nBytesReceived);

    //发送数据
    void do_write(CommMsg msg);
    void handle_write(const boost::system::error_code& error, size_t bytes_sent);

    //未传入协议的帧头解析函数时使用
    //获取数据长度不确定，导致对方发送速度过快时需要额外解析队列中数据并移位
    void handle_read(const boost::system::error_code& error,size_t nBytesReceived);
    void do_close();

    CommMsg *pread_msg_;                  //协议信息结构指针
    CommMsg_queue write_msgs_;            //写数据队列
    Msg_queue read_msg_;                  //读数据队列
    CommMsg _read_msg_;                   //信息结构
};
}
