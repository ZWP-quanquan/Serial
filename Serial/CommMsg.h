#ifndef COMMMSG_H
#define COMMMSG_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace CP
{
void DebugString(const char* str);
void DebugString(std::string const& str);

class CommMsg
{
public:
    enum { header_length = 2 };
    enum { max_body_length = 2048 };

    CommMsg(void);
    virtual ~CommMsg(void);

    const unsigned char* data() const;
    unsigned char* data();

    size_t length() const;

    size_t buf_header() const;            //设置帧头长度
    void buf_header(size_t len);

    const unsigned char* body() const;    //获取数据
    unsigned char* body();

    size_t body_length() const;
    void body_length(size_t length);

    size_t send_length() const;
    void send_length(size_t length);


    virtual bool decode_header();
    virtual bool decode_check();

private:
    unsigned char data_[header_length + max_body_length];
    size_t buf_header_;
    size_t body_length_;
    size_t send_length_;

};

}
#endif // COMMMSG_H
