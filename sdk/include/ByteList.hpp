#ifndef _BYTE_LIST_WANGHONGTAO_2014_08_30_
#define _BYTE_LIST_WANGHONGTAO_2014_08_30_


class ByteList{
protected:
	boost::mutex m_mutex;
    int			   m_iOffset = 0;
	int			   m_iMax;
public:
    char* m_pBuffer=NULL;

    ByteList(): m_pBuffer(NULL),m_iMax(1024),m_iOffset(0){}

    ~ByteList()
    {
		if (m_pBuffer != NULL){
			delete[] m_pBuffer;
			m_pBuffer = NULL;
		}
    }

    void move(int pose)
    {
        for(int i=pose,j=0;i<m_iOffset;j++,i++){
            m_pBuffer[j] = m_pBuffer[i];
        }
        m_iOffset = m_iOffset - pose;
    }

    int Size()
    {
		boost::mutex::scoped_lock lock(m_mutex);
		return m_iOffset;
    }

    int Write(unsigned char* pWrite,int len)
    {
		boost::mutex::scoped_lock lock(m_mutex);
		if ( len > (m_iMax - m_iOffset) ){
			len = m_iMax - m_iOffset;
		}

		if (m_pBuffer == NULL){
			return 0;
		}
		memset(m_pBuffer + m_iOffset,0,len);
		memcpy(m_pBuffer + m_iOffset,pWrite,len);

		m_iOffset+=len;

		return len;
    }

    int Read(unsigned char* pRead,int& len)
    {
		boost::mutex::scoped_lock lock(m_mutex);
		if (m_iOffset>0){
			assert(m_pBuffer!=NULL);
			memset(pRead,0,m_iOffset);
			memcpy(pRead,m_pBuffer,m_iOffset);
			len = m_iOffset;
            //if (m_pBuffer != NULL){
               //memset(m_pBuffer,0,m_iMax);
            //}
            //m_iOffset = 0;
			return m_iOffset;
		}
		else{
			return 0;
		}
	}

    void Init(int iLen)
    {
		if (m_pBuffer != NULL){
            delete[] m_pBuffer;
			m_pBuffer = NULL;
		}
        if (iLen>0){
			boost::mutex::scoped_lock lock(m_mutex);
            m_pBuffer = new char[iLen];
			m_iMax = iLen;
		}

	}

    void Clear()
    {
		boost::mutex::scoped_lock lock(m_mutex);
		if (m_pBuffer != NULL){
			memset(m_pBuffer,0,m_iMax);
		}
		m_iOffset = 0;
	}

    bool IsPull()
    {
		boost::mutex::scoped_lock lock(m_mutex);
		if (m_iOffset == m_iMax ){
			return true;
		}else{
			return false;
		}

	}
};
#endif//_BYTE_LIST_WANGHONGTAO_2014_08_30_
