#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "SPort.h"
#include "Comm.h"

using namespace boost;
using namespace boost::asio;


SerialPort::SerialPort()
{
    m_pSerialPort = NULL;
    m_threadRun = NULL;


    m_lReadBuffer.Init(1024);
    memset(m_szWriteBuffer,0,1024);
    m_nWriteBufferSize = 0;

    //LOGS("SSS").Enable();
}

SerialPort::~SerialPort()
{
    if (m_pSerialPort != NULL)
    {
        delete m_pSerialPort;
        m_pSerialPort = NULL;
    }
}

bool SerialPort::open()
{
    try{
        if (m_pSerialPort != NULL){
            delete m_pSerialPort;
            m_pSerialPort = NULL;
        }
        string strComNo;
        //strComNo = "COM";
    #ifdef _WIN32
        strComNo = "\\\\.\\COM";
        strComNo += cComm::ConvertToString(nCommNO);
    #else
        strComNo = "/dev/ttyS0";
        //strComNo += cComm::ConvertToString(nCommNO);
        std::cout<<"linux serial port"<<strComNo<<std::endl;
    #endif
        m_pSerialPort = new boost::asio::serial_port(m_ios ,strComNo);
        m_pSerialPort->set_option(boost::asio::serial_port::baud_rate(115200));
        m_pSerialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        m_pSerialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        m_pSerialPort->set_option(boost::asio::serial_port::character_size(8));
        std::cout<<"Open Serial Port OK!!!--"<<1<<std::endl;

        return true;
    }catch (std::exception &e){
        //LOGS("SerialPort")<<"Serial port open err!";
        std::cout<<"Open Serial Port Err!!!--"<<1<<std::endl;
        return false;
    }

}

void SerialPort::Send_data(unsigned char* s_data, unsigned short len )
{
    if (len > 1024)
    {
        return;
    }
    m_lReadBuffer.Clear();
    memset(m_szWriteBuffer,0,1024);
    m_nWriteBufferSize = len;
    memcpy(m_szWriteBuffer,s_data,len);
    write();
}
void SerialPort::read_callback( const boost::system::error_code& error, std::size_t bytes_transferred)
{

    if (error ){
        std::cout<<"read data err"<<std::endl;
        return;
    }

    //m_szReadCallBack.Write(m_szReadTemp,bytes_transferred);
    std::cout << "data" << m_szReadTemp << std::endl;
    m_lReadBuffer.Write(m_szReadTemp,bytes_transferred);

    std::string str = cComm::ByteToHexString(m_szReadTemp,bytes_transferred);
    std::cout<<"SSSS Read len:"<<bytes_transferred<<" data:"<<str<<std::endl;

    read();
}
void SerialPort::Read_data( unsigned char* r_data,int &len,int need,int timeout )
{
    len = 0;
    int read_count = 20;
    int len_tmp = 0;

    while(1){

        len_tmp = m_lReadBuffer.Size();
        if ( len_tmp >= need){

            //std::cout<<"read data over"<<std::endl;
            break;
        }
        if (read_count--){
            usleep(10000);
        }else{
            std::cout<<"time out"<<std::endl;
            break;
        }
    }
    m_lReadBuffer.Read(r_data,len);
}

int SerialPort::ThreadRun()
{
    try{
        while (1)
        {
            if (open())
            {
                read();
                m_ios.run();
                m_ios.reset();
                this_thread::interruption_point();
            }

            this_thread::interruption_point();
            usleep(1000);
        }
        return 0;
    }catch(thread_interrupted){
        return -1;
    }
}

bool SerialPort::BeginThread()
{
    EndThread();

    m_threadRun = new thread(bind(&SerialPort::ThreadRun,this));
    if (m_threadRun){
        return true;
    }else{
        return false;
    }
}
void SerialPort::EndThread()
{

    if (m_threadRun != NULL)
    {
        m_ios.stop();
        m_threadRun->interrupt();
        m_threadRun->join();

        delete m_threadRun;
        m_threadRun = NULL;
    }

}

void SerialPort::read()
{
    std::cout << "m_szReadTemp" << m_szReadTemp << std::endl;
    m_pSerialPort->async_read_some(boost::asio::buffer(m_szReadTemp,1024),
        boost::bind(&SerialPort::read_callback,this,
        boost::asio::placeholders::error,   boost::asio::placeholders::bytes_transferred));
}

void SerialPort::write()
{
    m_pSerialPort->write_some(boost::asio::buffer(m_szWriteBuffer,m_nWriteBufferSize));

}

void SerialPort::Init(int CommNO)
{
    nCommNO = CommNO;
    BeginThread();
}
