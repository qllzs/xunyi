#include "FileReadWrite.h"
/**
  ******************************************************************************
  *                              �������
  ******************************************************************************
  */
FATFS fs;													/* FatFs�ļ�ϵͳ���� */
FIL fnew;													/* �ļ����� */
FRESULT res_sd;                /* �ļ�������� */
UINT fnum;            					  /* �ļ��ɹ���д���� */
BYTE ReadBuffer[1024]={0};        /* �������� */
char WriteBuffer[1024] = {0};



uint8_t FsInit(void)
{
//���ⲿSD�������ļ�ϵͳ���ļ�ϵͳ����ʱ���SD���豸��ʼ��
	
	res_sd = f_mount(&fs,"0:",1);
/*----------------------- ��ʽ������ ---------------------------*/  
	/* ���û���ļ�ϵͳ�͸�ʽ�����������ļ�ϵͳ */
	if(res_sd == FR_NO_FILESYSTEM)
	{
		#ifndef Debug_CCD
			printf("��SD����û���ļ�ϵͳ���������и�ʽ��...\r\n");
		#endif
    /* ��ʽ�� */
		res_sd=f_mkfs("0:",0,0);							
		
		if(res_sd == FR_OK)
		{
			#ifndef Debug_CCD
			printf("��SD���ѳɹ���ʽ���ļ�ϵͳ��\r\n");
			#endif
      /* ��ʽ������ȡ������ */
			res_sd = f_mount(NULL,"0:",1);			
      /* ���¹���	*/			
			res_sd = f_mount(&fs,"0:",1);
			
			return 0;
		}
		else
		{
			LED_RED;
			#ifndef Debug_CCD
			printf("������ʽ��ʧ�ܡ�����\r\n");
			#endif
			return 1;
		}
	}
  else if(res_sd!=FR_OK)
  {
		#ifndef Debug_CCD
    printf("����SD�������ļ�ϵͳʧ�ܡ�(%d)\r\n",res_sd);
    printf("��������ԭ��SD����ʼ�����ɹ���\r\n");
		#endif
		//while(1);
		return 1;
  }
  else
  {
		#ifndef Debug_CCD
    printf("���ļ�ϵͳ���سɹ������Խ��ж�д����\r\n");
		#endif
		return 0;
  }
}
/*
*@brief:�ļ�д�뺯��
*@param:Write_Buffer-��д�����ݵ�ָ�룬path-д���ļ���·�����ļ���
*
*/
uint8_t fileWrite(char *Write_Buffer,const TCHAR* path)
{
	uint8_t	ret;
	ret = f_lseek (&fnew,fnew.fsize);
	if( ret)
	{
		#ifndef Debug_CCD
		printf("****** ƫ��ʧ�� %d******\r\n",ret);
		#endif
		return 1;
	}

	ret = f_write(&fnew,Write_Buffer,strlen(Write_Buffer),&fnum);
	if( ret)
	{
		#ifndef Debug_CCD
		printf("****** д�ļ�ʧ�� %d******\r\n",ret);
		#endif
		return 2;
	}
	return 0;
}
void FileRead(void)
{
/*------------------- �ļ�ϵͳ���ԣ������� ------------------------------------*/
	#ifndef Debug_CCD
	printf("****** ���������ļ���ȡ����... ******\r\n");
	#endif
	res_sd = f_open(&fnew, "0:FatFs��д�����ļ�.txt", FA_OPEN_ALWAYS | FA_READ); 	 
	if(res_sd == FR_OK)
	{
		LED_GREEN;
		#ifndef Debug_CCD
		printf("�����ļ��ɹ���\r\n");
		#endif
		res_sd = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum); 
    if(res_sd==FR_OK)
    {
			#ifndef Debug_CCD
      printf("���ļ���ȡ�ɹ�,�����ֽ����ݣ�%d\r\n",fnum);
      printf("����ȡ�õ��ļ�����Ϊ��\r\n%s \r\n", ReadBuffer);	
			#endif
    }
    else
    {
			#ifndef Debug_CCD
      printf("�����ļ���ȡʧ�ܣ�(%d)\n",res_sd);
			#endif
    }		
	}
	else
	{
		LED_RED;
		#ifndef Debug_CCD
		printf("�������ļ�ʧ�ܡ�\r\n");
		#endif
	}
	/* ���ٶ�д���ر��ļ� */
	f_close(&fnew);	
}




















