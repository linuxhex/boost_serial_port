
/*protocol.cpp 数据协议的处理
 */
#include "string.h"
#include "iostream"
#include "SPort.h"
#include <vector>
#include "protocol.h"


AGVProtocol sendProtocol;
AGVProtocol recProtocol;
int recLen = 0;

int bs_init = 0;
int rs_init = 0;

MSGStates rstatus;

#ifdef MCU

int volatile m_front_pos = 0;
int volatile m_rear_pos = 0;
int volatile m_front_delta = 0;
int volatile m_rear_delta = 0;

unsigned char send[1024] = {0};
int len = 0;

#else

int m_front_pos = 0;
int m_rear_pos = 0;
int volatile m_front_delta = 0;
int volatile m_rear_delta = 0;
unsigned int m_di = 0;
unsigned int cnt_time = 0;
unsigned int m_ad[4] = {0};

#endif

void SInit_Proto(AGVProtocol* agvProtocol,CMDTypes type){

    if(!bs_init){
        Init(&(sendProtocol.buflist),50);

//        agvProtocol->header[0] = HEADER0;
//        agvProtocol->header[1] = HEADER1;
//        agvProtocol->header[2] = HEADER2;
//        agvProtocol->header[3] = HEADER3;
//        agvProtocol->header[4] = HEADER4;
//        agvProtocol->header[5] = HEADER5;
//#ifdef MCU
//        agvProtocol->srcaddr = DEVCADDR;
//        agvProtocol->dstaddr = HOSTADDR;
//#else
//        agvProtocol->srcaddr = HOSTADDR;
//        agvProtocol->dstaddr = DEVCADDR;
//#endif
        bs_init = 1;
    }

    agvProtocol->chksum = 0;

    memset(&(agvProtocol->data),0,sizeof(Data));
    agvProtocol->len = 0;

    Clear(&(agvProtocol->buflist));
    agvProtocol->type = type;

}
void RInit_Proto(AGVProtocol* agvProtocol){

    if(!rs_init){

        Init(&(recProtocol.buflist),1024);

//        agvProtocol->header[0] = HEADER0;
//        agvProtocol->header[1] = HEADER1;
//        agvProtocol->header[2] = HEADER2;
//        agvProtocol->header[3] = HEADER3;
//        agvProtocol->header[4] = HEADER4;
//        agvProtocol->header[5] = HEADER5;

//#ifdef MCU
//        agvProtocol->srcaddr = HOSTADDR;
//        agvProtocol->dstaddr = DEVCADDR;
//#else
//        agvProtocol->srcaddr = DEVCADDR;
//        agvProtocol->dstaddr = HOSTADDR;
//#endif


        agvProtocol->type = NONE;

        memset(&(agvProtocol->data),0,sizeof(Data));
        agvProtocol->len = 0;

        agvProtocol->chksum = 0;

        Clear(&(agvProtocol->buflist));

        rs_init = 1;
        rstatus = CMDFIELD;
        //rstatus = SYNCHEAD0;
    }

}
unsigned char checksum(unsigned char* ch ,int len){
    unsigned char sum = 0;
    int i = 0;
    for(; i < len; ++i){
        sum += ch[i];
    }
    sum &= 0x00ff;
    return sum;
}
int Coder(unsigned char* ch,int* len,AGVProtocol* protol,Data* data){
    *len = 0;

    switch(protol->type){
        case CURRENT:
            break;
//        case POS:
//            protol->data.pos_.system_time = data->pos_.system_time;
//            protol->data.pos_.axis_id = data->pos_.axis_id;
//            protol->data.pos_.position = data->pos_.position;
//            protol->len = sizeof(PosProtocol);
//            break;
        case TIME:
            break;
        case DO:
            protol->data.do_.usdo = data->do_.usdo ;
            protol->len = sizeof(DoProtocol);
            break;
        case DA:
            protol->data.da_.system_time = data->da_.system_time;
            protol->data.da_.axis_id =  data->da_.axis_id;
            protol->data.da_.da_value = data->da_.da_value;
            protol->len = sizeof(DaProtocol);
            break;
        case RCURRENT:
            break;
        case RSPEED:
            break;
        case RPOS:
            protol->data.r_pos_.axis_id = data->r_pos_.axis_id;
            protol->len = sizeof(RPosProtocol);
            break;
        case RTIME:
            break;
        default:
            break;
    }

//    Write(&(protol->buflist),protol->header,6);
//    Write(&(protol->buflist),&(protol->srcaddr),1);
//    Write(&(protol->buflist),&(protol->dstaddr),1);
    Write(&(protol->buflist),(unsigned char*)&(protol->type),1);
    Write(&(protol->buflist),(unsigned char*)&(protol->len),1);
    Write(&(protol->buflist),(unsigned char*)&(protol->data),protol->len);

    Read(&(protol->buflist),ch,len);
    //protol->chksum = checksum(ch+6,*len-6);
    protol->chksum = checksum(ch,*len);
    ch[*len] = protol->chksum;
    (*len)++;
    return *len;
}
int Decoder(AGVProtocol* protol,unsigned char* ch,int len){
    memcpy(&(protol->data), ch + 2, sizeof(Data));

    switch(protol->type){
        case CURRENT:
            break;
        case POS:
                m_front_pos = protol->data.pos_.Fposition;
                m_rear_pos  = protol->data.pos_.Rposition;
            break;
        case TIME:
            break;
        case RCURRENT:
            break;
        case RSPEED:
            break;
        case RPOS:
//#ifdef MCU
//            if(protol->data.angle_.axis_id == 0){
//                CreatePos(send,&len,0,m_left_pos);
//                uart0SendStr(send,len);
//            }else if(protol->data.angle_.axis_id == 0){
//                CreatePos(send,&len,1,m_rear_pos);
//                uart0SendStr(send,len);
//            }
//#endif
            break;
        case RTIME:
            break;
        default:
            break;
    }

    return 1;
}



void CreateDO(unsigned char* ch,int* len,int id,unsigned int usdo){
  Data data;

  data.do_.usdo = usdo;

  SInit_Proto(&sendProtocol,DO);

  Coder(ch,len,&sendProtocol,&data);
}

//void CreatePos(unsigned char* ch,int* len,int id,int pos){
//    Data data;

//    data.pos_.axis_id = id;

//    SInit_Proto(&sendProtocol,POS);

//    Coder(ch,len,&sendProtocol,&data);
//}
void CreateDA(unsigned char* ch,int* len,int id,float v){
  unsigned int i_value = (v / 5.0) * 0x000fff ;
  i_value &= 0x000fff;

  Data data;

  data.da_.system_time = 0xffffffff;
  data.da_.axis_id = id;
  data.da_.da_value = i_value;

  SInit_Proto(&sendProtocol,DA);

  Coder(ch,len,&sendProtocol,&data);
}

void CreateRPos(unsigned char* ch,int* len,int id){

    Data data;
    data.r_pos_.axis_id = id;

    SInit_Proto(&sendProtocol,RPOS);

    Coder(ch,len,&sendProtocol,&data);

    rs_init = 0;
}

unsigned int GetDI(){
  return m_di;
}
float GetAD(int id){
  float ad = 0;
  if(id < 4){
    ad = m_ad[id];
    ad = 5.0 * (ad / 4096);
  }
  return ad;
}
int GetDelta(int id)
{
  if (id == 0) {
    return m_front_delta;
  } else{
    return m_rear_delta;
  }
}


int GetPos(int id){
    if(id == 0){
        return m_front_pos;
    }else {
        return m_rear_pos;
    }
}

int IRQ_CH(unsigned char c){
    unsigned char tmp[50];
    int len = 0;

    RInit_Proto(&recProtocol);

#ifdef MCU
    zyIrqDisable();
#endif

    switch(rstatus){
//    case SYNCHEAD0:
//        if(c==HEADER0) rstatus = SYNCHEAD1;
//        else rstatus = SYNCHEAD0;
//        break;
//    case SYNCHEAD1:
//        if(c==HEADER1) rstatus = SYNCHEAD2;
//        else rstatus = SYNCHEAD0;
//        break;
//    case SYNCHEAD2:
//        if(c==HEADER2) rstatus = SYNCHEAD3;
//        else rstatus = SYNCHEAD0;
//        break;
//    case SYNCHEAD3:
//        if(c==HEADER3) rstatus = SYNCHEAD4;
//        else rstatus = SYNCHEAD0;
//        break;
//    case SYNCHEAD4:
//        if(c==HEADER4) rstatus = SYNCHEAD5;
//        else rstatus = SYNCHEAD0;
//        break;
//    case SYNCHEAD5:
//        if(c==HEADER5){
//            Clear(&(recProtocol.buflist));
//            rstatus = SRCADDR;
//        }
//        else rstatus = SYNCHEAD0;
//        break;
//    case SRCADDR:
//        if(recProtocol.srcaddr == c)
//        {
//            Write(&(recProtocol.buflist),&c,1);
//            rstatus = DSTADDR;
//        }
//        else rstatus = SYNCHEAD0;
//        break;
//    case DSTADDR:
//        if(recProtocol.dstaddr == c)
//        {
//            Write(&(recProtocol.buflist),&c,1);
//            rstatus = CMDFIELD;
//        }
//        else rstatus = SYNCHEAD0;
//        break;
    case CMDFIELD:
        recProtocol.type = (CMDTypes)c;
        Write(&(recProtocol.buflist),&c,1);
        rstatus = DATLEN;
        break;
    case DATLEN:
        recProtocol.len = c;
        Write(&(recProtocol.buflist),&c,1);
        rstatus = DATFIELD;
        break;
    case DATFIELD:
        Write(&(recProtocol.buflist),&c,1);
        //if(Size(&(recProtocol.buflist)) < (recProtocol.len + 4)){

        //}else{
            rstatus = CHKSUM;
       // }
        break;
    case CHKSUM:

        Read(&(recProtocol.buflist),&tmp[0],&len);
        if(c == checksum(&tmp[0],len)){
            if(!Decoder(&recProtocol,&tmp[0],len)){
                rstatus = CMDFIELD;
#ifdef MCU
                zyIrqEnable();
#endif
                return 0;
            }
        }else{
          std::string str = cComm::ByteToHexString(tmp, len);
          std::string strc = cComm::ByteToHexString(&c, 1);
          std::cout << "check sum err: " << str << " c: " << strc << std::endl;
        }

        rstatus = CMDFIELD;
#ifdef MCU
        zyIrqEnable();
#endif
        return 1;
        break;
    default:
        rstatus = CMDFIELD;
        rs_init = 1;
    }
#ifdef MCU
    zyIrqEnable();
#endif
    return 0;
}


