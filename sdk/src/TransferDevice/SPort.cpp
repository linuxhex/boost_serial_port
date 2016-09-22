#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "SPort.h"
#include "Comm.h"
extern unsigned char data[1024];
extern int data_len;
using namespace boost;
using namespace boost::asio;

SerialPort::SerialPort()
{
    m_pSerialPort = NULL;
    m_threadRun = NULL;
    m_lReadBuffer.Init(1024);
    memset(m_szWriteBuffer,0,1024);
    m_nWriteBufferSize = 0;
}

SerialPort::~SerialPort()
{
    if (m_pSerialPort != NULL){
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
        std::cout<<"m_pSerialPort" <<std::endl;
        m_pSerialPort->set_option(boost::asio::serial_port::baud_rate(115200));
        m_pSerialPort->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        m_pSerialPort->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        m_pSerialPort->set_option(boost::asio::serial_port::character_size(8));
        std::cout<<"Open Serial Port OK!!!--"<<1<<std::endl;

        return true;
    }catch (std::exception &e){
        std::cout<<"Open Serial Port Err!!!--"<<1<<std::endl;
        return false;
    }

}

void SerialPort::Send_data(unsigned char* s_data, unsigned short len )
{
    if (len > 1024){
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

    m_lReadBuffer.Write(m_szReadTemp,bytes_transferred);
    int len = 0;
    m_lReadBuffer.Read(data,len);

//    str = cComm::ByteToHexString((unsigned char *)m_lReadBuffer.m_pBuffer,m_lReadBuffer.Size());
//    std::cout<<"Buff Read len:"<<m_lReadBuffer.Size()<<" data:"<<str<<std::endl;

    for(int i=0;i<len;i++){
        if(data[i] == 0xab){
            m_lReadBuffer.move(i+1);
            if (i >= SERIAL_BYTE_LEN-1) {
               data_len  = SERIAL_BYTE_LEN;
               for(int j=0,k=i-(SERIAL_BYTE_LEN-1);k<=i;k++,j++){
                   if(j == k)break;
                   data[j] = data[k];
               }
            }
            break;
        }
    }
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
            break;
        }
        if (read_count--){
            usleep(10000);
        }else{
            break;
        }
    }
    m_lReadBuffer.Read(r_data,len);
}

int SerialPort::ThreadRun()
{
    try{
        while (1){
            if (open()){
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

    std::cout << "BeginThread::Init" << std::endl;

    m_threadRun = new thread(bind(&SerialPort::ThreadRun,this));
    if (m_threadRun){
        return true;
    }else{
        return false;
    }
}
void SerialPort::EndThread()
{

    if (m_threadRun != NULL){
        m_ios.stop();
        m_threadRun->interrupt();
        m_threadRun->join();
        delete m_threadRun;
        m_threadRun = NULL;
    }

}

void SerialPort::read()
{
    m_pSerialPort->async_read_some(boost::asio::buffer(m_szReadTemp,1024),
                                   boost::bind(&SerialPort::read_callback,this,
                                   boost::asio::placeholders::error,
                                   boost::asio::placeholders::bytes_transferred));
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
