#include "CommMsg.h"

#ifdef USE_QT_W
#include <QDebug>
#endif

namespace CP
{

void DebugString(const char *str)
{
#ifndef USE_QT_W
    OutputDebugString(str);
#else
    qDebug() << str;
#endif
}

void DebugString(std::string const& str)
{
    DebugString(str.c_str());
}

CommMsg::CommMsg(void)
        : buf_header_(header_length)
        , body_length_(0)
{
}

CommMsg::~CommMsg(void)
{
}



const unsigned char* CommMsg::data() const
{
    return data_;
}

unsigned char* CommMsg::data()
{
    return data_;
}

size_t CommMsg::length() const
{
    return header_length + body_length_;
}

const unsigned char* CommMsg::body() const
{
    return data_ + header_length;
}

unsigned char* CommMsg::body()
{
    return data_;
    //return data_ + header_length;
}

size_t CommMsg::buf_header() const
{
    return buf_header_;
}

void CommMsg::buf_header(size_t len)
{
    buf_header_ = len;
}


size_t CommMsg::body_length() const
{
    return body_length_;
}

void CommMsg::body_length(size_t length)
{
    body_length_ = length;
    if (body_length_ > max_body_length)
        body_length_ = max_body_length;
}

size_t CommMsg::send_length() const
{
    return send_length_;
}

void CommMsg::send_length(size_t length)
{
    send_length_ = length;
    if (send_length_ > max_body_length)
        send_length_ = max_body_length;
}

bool CommMsg::decode_header()
{
    //这里需要重新定义数据帧头和根据Cmd设置数据包长度
    body_length(5);
    return true;
}

bool CommMsg::decode_check()
{
    return true;
}

}