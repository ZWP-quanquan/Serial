#include "Serial_Asio.h"

///typedef std::deque<CommMsg> chat_message_queue;

namespace CP
{
Serial_Asio::Serial_Asio(void)
{
    m_pPort = new serial_port(m_ios);
    disloc_ = 0;
    msg_offset = 0;
    pread_msg_ = nullptr;
    err = false;
    open_ = false;

    ErrMsg = "Serial normal";

    nNumber = 0;
}


Serial_Asio::~Serial_Asio(void)
{

    if(m_pPort->is_open())
    {
        Close();
    }

    if(m_pPort)
        delete m_pPort;
}

void Serial_Asio::Init_Msg(CommMsg *pMsg)
{
    pread_msg_ = pMsg;
}

bool Serial_Asio::Init_Port(const string port, size_t BaudRate, size_t ByteSize)
{
    m_strCom = port;
    m_BaudRate = BaudRate;
    m_ByteSize= ByteSize;
    m_pPort->open(port,ec);

    if(ec)
    {
        err = true;
        ErrMsg = "Error while opening the device!";
        return false;
    }
    m_pPort->set_option(serial_port::baud_rate(BaudRate), ec);//设置波特率
    m_pPort->set_option(serial_port::character_size(ByteSize), ec);//设置数据位
    m_pPort->set_option(serial_port::flow_control(serial_port::flow_control::none), ec);//设置流控制
    m_pPort->set_option(serial_port::parity(serial_port::parity::none), ec);//设置奇偶校验
    m_pPort->set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec);//设置停止位


    if(pread_msg_ == nullptr)
    {
        m_pPort->async_read_some(
            buffer(_read_msg_.data(),CommMsg::max_body_length),
            boost::bind(&Serial_Asio::handle_read,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        boost::asio::async_read(
            *m_pPort,
            buffer(pread_msg_->data(), pread_msg_->buf_header()),
            boost::bind(&Serial_Asio::handle_read_header,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }

     io_thread= boost::thread(boost::bind(&boost::asio::io_service::run, &m_ios));
     open_ = true;
     err = false;

    return true;
}

//导入了帧头解析函数时使用的回调函数
void Serial_Asio::handle_read_header(const boost::system::error_code& error,size_t)
{
    if (!error)
    {
        if(pread_msg_->decode_header())
        {
            boost::asio::async_read(
                *m_pPort,
                buffer(pread_msg_->body()+pread_msg_->buf_header(), pread_msg_->body_length()),
                boost::bind(&Serial_Asio::handle_read_body, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
        }
        else
        {
            int len = (int)pread_msg_->buf_header();
            if(disloc_ != 0)
            {
                unsigned char tmpdata[64];
                memcpy(tmpdata,pread_msg_->data(),len);
                for(int i=0;i<disloc_;i++)
                {
                    tmpdata[i] = tmpdata[disloc_+i];
                }
                memcpy(pread_msg_->data(),tmpdata,len);
                len = msg_offset;
            }
            boost::asio::async_read(
                *m_pPort,
                buffer(pread_msg_->data()+disloc_, len),
                boost::bind(&Serial_Asio::handle_read_header, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
            disloc_ = 0;
            msg_offset = 0;
        }
    }
    else
    {
        if(open_)
        {
            err = true;
            ErrMsg = "Serial closes unexpectedly,Error while reading data!";
            ErrorHandle();
        }
        ec = error;
        do_close();

    }
}

void Serial_Asio::handle_read_body(const boost::system::error_code& error,size_t)
{
    if (!error)
    {
/*        size_t buf_length = pread_msg_->buf_length();*/
        size_t data_length = pread_msg_->length();

//         nNumber++;
//         CString str;
//         str.Format("%d\n",nNumber);
//         OutputDebugString(str);

        if(pread_msg_->decode_check())
        {
            boost::lock_guard<boost::shared_mutex> lock(read_); 
            unsigned char *pMsg = new unsigned char[64];
            memcpy(pMsg,pread_msg_->data(),data_length);
            read_msg_.push_back(pMsg);
        }    

        boost::asio::async_read(
            *m_pPort,
            buffer(pread_msg_->data(), pread_msg_->buf_header()),
            boost::bind(&Serial_Asio::handle_read_header, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if(open_)
        {
            err = true;
            ErrMsg = "Serial closes unexpectedly,Error while reading data!";
            ErrorHandle();
        }
        ec = error;
        do_close();
    }
}

//未导入帧头解析函数的回调函数,获取数据长度不确定，导致对方发送速度过快时需要额外解析队列中数据并移位
void Serial_Asio::handle_read(const boost::system::error_code& error,size_t nBytesReceived)
{
    if (!error)
    {
//         {
            boost::lock_guard<boost::shared_mutex> lock(read_); 
            unsigned char *pMsg = new unsigned char[1024];
            memcpy(pMsg,_read_msg_.data(),nBytesReceived);
            read_msg_.push_back(pMsg);
//         }    

//         nNumber++;
//         CString str;
//         str.Format("%d\n",nNumber);
//         OutputDebugString(str);

        m_pPort->async_read_some(
            buffer(_read_msg_.data(), CommMsg::max_body_length),
            boost::bind(&Serial_Asio::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if(open_)
        {
            err = true;
            ErrMsg = "Serial closes unexpectedly,Error while reading data!";
            ErrorHandle();
        }
        ec = error;
        do_close();
    }
}

void Serial_Asio::SendMsg(const CommMsg& msg)
{
    m_ios.post(boost::bind(&Serial_Asio::do_write, this, msg));
}

void Serial_Asio::do_write(CommMsg msg)
{
    bool write_in_progress = !write_msgs_.empty();
    write_msgs_.push_back(msg);
    if (!write_in_progress)
    {
        boost::asio::async_write(
            *m_pPort,
            buffer(write_msgs_.front().data(),
                write_msgs_.front().send_length()),
            boost::bind(
                &Serial_Asio::handle_write,
                this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void Serial_Asio::handle_write(const boost::system::error_code& error, size_t)
{
    if (!error)
    {
        write_msgs_.pop_front();
        if (!write_msgs_.empty())
        {
            boost::asio::async_write(
                *m_pPort,
                buffer(write_msgs_.front().data(),
                    write_msgs_.front().send_length()),
                boost::bind(&Serial_Asio::handle_write, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
        }
    }
    else
    {
        if(open_)
        {
            err = true;
            ErrMsg = "Serial closes unexpectedly,Error while writing data!";
            ErrorHandle();
        }
        ec = error;
        do_close();
    }
}

bool Serial_Asio::isempty()
{
    return read_msg_.empty();
}

unsigned char* Serial_Asio::GetMsg()
{
    unsigned char *Msg = read_msg_.front();
    read_msg_.pop_front();
    return Msg;
}

void Serial_Asio::Close()
{
    if(m_pPort->is_open())
    {
        open_ = false;
        m_ios.post(boost::bind(&Serial_Asio::do_close, this));
        io_thread.join();
        m_ios.reset();
    }
 }

void Serial_Asio::do_close()
 {
     //m_pPort->cancel();
     if(m_pPort->is_open())
     {
         m_pPort->cancel();
         m_pPort->close();
     }
     
 }

bool Serial_Asio::ErrorHandle()
{
    return err;
}

bool Serial_Asio::PortSts()
 {
     return m_pPort->is_open();
 }

string Serial_Asio::GetPortName() const
{
    return m_strCom;
}

size_t Serial_Asio::GetBaudRate() const
{
    return m_BaudRate;
}

size_t Serial_Asio::GetByteSize() const
{
    return m_ByteSize;
}

string Serial_Asio::GetErrorCode() const
{
    return ec.message();
}
}