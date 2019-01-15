#ifndef   __FILEREADWRITE_H__
#define   __FILEREADWRITE_H__
#include "bsp_include.h"






uint8_t FsInit(void);
uint8_t fileWrite(char *Write_Buffer,const TCHAR* path);
void FileRead(void);




extern char WriteBuffer[1024];




#endif




































