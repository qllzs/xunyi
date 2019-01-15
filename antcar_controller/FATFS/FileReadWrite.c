#include "FileReadWrite.h"
/**
  ******************************************************************************
  *                              定义变量
  ******************************************************************************
  */
FATFS fs;													/* FatFs文件系统对象 */
FIL fnew;													/* 文件对象 */
FRESULT res_sd;                /* 文件操作结果 */
UINT fnum;            					  /* 文件成功读写数量 */
BYTE ReadBuffer[1024]={0};        /* 读缓冲区 */
char WriteBuffer[1024] = {0};



uint8_t FsInit(void)
{
//在外部SD卡挂载文件系统，文件系统挂载时会对SD卡设备初始化
	
	res_sd = f_mount(&fs,"0:",1);
/*----------------------- 格式化测试 ---------------------------*/  
	/* 如果没有文件系统就格式化创建创建文件系统 */
	if(res_sd == FR_NO_FILESYSTEM)
	{
		#ifndef Debug_CCD
			printf("》SD卡还没有文件系统，即将进行格式化...\r\n");
		#endif
    /* 格式化 */
		res_sd=f_mkfs("0:",0,0);							
		
		if(res_sd == FR_OK)
		{
			#ifndef Debug_CCD
			printf("》SD卡已成功格式化文件系统。\r\n");
			#endif
      /* 格式化后，先取消挂载 */
			res_sd = f_mount(NULL,"0:",1);			
      /* 重新挂载	*/			
			res_sd = f_mount(&fs,"0:",1);
			
			return 0;
		}
		else
		{
			LED_RED;
			#ifndef Debug_CCD
			printf("《《格式化失败。》》\r\n");
			#endif
			return 1;
		}
	}
  else if(res_sd!=FR_OK)
  {
		#ifndef Debug_CCD
    printf("！！SD卡挂载文件系统失败。(%d)\r\n",res_sd);
    printf("！！可能原因：SD卡初始化不成功。\r\n");
		#endif
		//while(1);
		return 1;
  }
  else
  {
		#ifndef Debug_CCD
    printf("》文件系统挂载成功，可以进行读写测试\r\n");
		#endif
		return 0;
  }
}
/*
*@brief:文件写入函数
*@param:Write_Buffer-待写入数据的指针，path-写入文件的路径及文件名
*
*/
uint8_t fileWrite(char *Write_Buffer,const TCHAR* path)
{
	uint8_t	ret;
	ret = f_lseek (&fnew,fnew.fsize);
	if( ret)
	{
		#ifndef Debug_CCD
		printf("****** 偏移失败 %d******\r\n",ret);
		#endif
		return 1;
	}

	ret = f_write(&fnew,Write_Buffer,strlen(Write_Buffer),&fnum);
	if( ret)
	{
		#ifndef Debug_CCD
		printf("****** 写文件失败 %d******\r\n",ret);
		#endif
		return 2;
	}
	return 0;
}
void FileRead(void)
{
/*------------------- 文件系统测试：读测试 ------------------------------------*/
	#ifndef Debug_CCD
	printf("****** 即将进行文件读取测试... ******\r\n");
	#endif
	res_sd = f_open(&fnew, "0:FatFs读写测试文件.txt", FA_OPEN_ALWAYS | FA_READ); 	 
	if(res_sd == FR_OK)
	{
		LED_GREEN;
		#ifndef Debug_CCD
		printf("》打开文件成功。\r\n");
		#endif
		res_sd = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum); 
    if(res_sd==FR_OK)
    {
			#ifndef Debug_CCD
      printf("》文件读取成功,读到字节数据：%d\r\n",fnum);
      printf("》读取得的文件数据为：\r\n%s \r\n", ReadBuffer);	
			#endif
    }
    else
    {
			#ifndef Debug_CCD
      printf("！！文件读取失败：(%d)\n",res_sd);
			#endif
    }		
	}
	else
	{
		LED_RED;
		#ifndef Debug_CCD
		printf("！！打开文件失败。\r\n");
		#endif
	}
	/* 不再读写，关闭文件 */
	f_close(&fnew);	
}




















