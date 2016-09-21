#ifndef _BOOST_SERIAL_PORT_WANGHONGTAO_
#define _BOOST_SERIAL_PORT_WANGHONGTAO_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "ByteList.hpp"
#include "Comm.h"

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();
    void Init(int CommNO);
   // void Init(std::string strPortName,int baud);
    void Send_data(unsigned char* s_data,unsigned short len);
    void Read_data(unsigned char* r_data,int &len,int need,int timeout);
 ByteList m_lReadBuffer;
protected:
private:
    unsigned char m_szReadTemp[1024];

    unsigned char m_szWriteBuffer[1024];
    int m_nWriteBufferSize;



    int nCommNO;
    std::string portName_;
    int baud_;
    bool open();
    void write();
    void read();
    void read_callback( const boost::system::error_code& error, size_t bytes_transferred);
    void wait_callback(const boost::system::error_code& error);
    boost::asio::serial_port* m_pSerialPort;
    boost::asio::io_service m_ios;
    boost::thread* m_threadRun;
    bool BeginThread();
    void EndThread();
    int ThreadRun();

};
#endif

