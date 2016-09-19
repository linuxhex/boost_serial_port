#ifndef BUFFER_H
#define BUFFER_H
#include <stdlib.h>
#include <assert.h>

typedef struct _BufList{
    unsigned char* m_pBuffer;
    int			   m_iOffset;
    int			   m_iMax;

}BufList;

void Init(BufList* list,int iLen);
void Des(BufList* list);
int Size(BufList* list);
int Write(BufList* list,unsigned char* pWrite,int len);
int Read(BufList* list,unsigned char* pRead,int* len);
void Clear(BufList* list);
int IsPull(BufList* list);
void Print(BufList* list);
#endif // BUFFER_H
