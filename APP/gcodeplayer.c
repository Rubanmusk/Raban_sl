#include "gcodeplayer.h"
#include "Marlin.h"	 
#include "sdio_sdcard.h"
#include "usart.h"
#include "exfuns.h"
#include "malloc.h"		  
#include "string.h"
#include "language.h"
#include "temperature.h"
#include "stepper.h"
#include "lcdmenu.h"//20160409
//#include "guix.h"

#ifdef SDSUPPORT

//void gcode_ls(u8 * path)
//{
//	u8 res;	  
//    u8 *fn;   /* This function is assuming non-Unicode cfg. */
//#if _USE_LFN
// 	fileinfo.lfsize = _MAX_LFN * 2 + 1;
//	fileinfo.lfname = mymalloc(SRAMIN,fileinfo.lfsize);
//#endif		  

//    res = f_opendir(&dir,(const TCHAR*)path); //打开一个目录
//    if (res == FR_OK) 
//	{	

//		while(1)
//		{
//	        res = f_readdir(&dir, &fileinfo);                   //读取目录下的一个文件  读取目录项，索引会自动下移。
//	        if (res != FR_OK || fileinfo.fname[0] == 0) break;  //错误了/到末尾了,退出
//	        //if (fileinfo.fname[0] == '.') continue;             //忽略上级目录
//#if _USE_LFN
//        	fn =(u8*)( *fileinfo.lfname ? fileinfo.lfname : fileinfo.fname);
//#else							   
//        	fn =(u8*)( fileinfo.fname);
//#endif	                                              /* It is a file. */
//		//	printf("%s/", path);//打印路径	
//	 	    res=f_typetell(fn);	
//			if((res&0XF0)==0X60)//取高四位,看看是不是Gcode文件	
//		  	{ printf("%s\n",  fn);//打印文件名	  
//			}
//		} 
//    }	  
//	myfree(SRAMIN,fileinfo.lfname);
//  //  return res;	  
//}

//void card_ls(void)
//{ 
//  if(card.lsAction==LS_Count)
// // nrFiles=0;
//  card.lsAction=LS_SerialPrint;
//  gcode_ls("0:\\GCODE");
//}

void card_initsd(void)
{ 
	card.cardOK = false;
  //SERIAL_ECHO_START;
  if(SD_Init())	//初始化失败
	{  		
		printf(MSG_SD_INIT_FAIL);	  //检测不到SD卡
	}
	 else 
	{   	
		card.cardOK = true;
		//printf(MSG_SD_CARD_OK);		
	}
}
void card_release(void)
{ 
	card.sdprinting = false;
  card.cardOK = false;
}
//void card_openFile(char* fname,bool read)  //只支持/GCODE 目录下的打印  //20160401
//{
//  u8 *pname;			//带路径的文件名

//  pname=mymalloc(SRAMIN,_MAX_LFN*2+1);//pname=mymalloc(SRAMIN,strlen((const char*)fname));//				//为带路径的文件名分配内存  
////  if(!card.cardOK)
////    return;
////  card.sdprinting = false;
//	strcpy((char *)card.filename,(const char*)fname);
//	{
//	const u8 chgchar[2]={0X5C,0X00};//转义符 等效"\"    
//	strcpy((char *)pname,"0:\\GCODE");	//拷贝path到pname里面
//	strcat((char*)pname,(const char*)chgchar);	//添加转义符
//	strcat((char*)pname,(const char*)fname);		//添加新增的名字
//  }
////	strcpy((char*)pname,"0:\\");	//"0:\\GCODE"			//复制路径(目录)
////	strcat((char*)pname,(const char*)fname);  			//将文件名接在后面
//  if(read)           	//M23 仅仅打开或创建文件
//  {
//		if(!f_open(&card.fgcode,(const TCHAR*)pname,FA_READ))//write	FA_OPEN_ALWAYS:	如果文件存在，则打开；否则，创建一个新文件。
//		{	
//			quickStop();				//20160412
//		  card.sdprinting = true;
//			feedmultiply = 100;
//			starttime=millis();
//			
////			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);////20170213-8266
////			TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE );//TIM_Cmd(TIM7, DISABLE);
//		}
//  }
//  else 
//  {                   //M28打开或创建文件，然后 card.saving = true;开始往文件里写数据    //FA_CREATE_ALWAYS 新建一个文件，如果文件已存在就覆盖
//		if(!f_open(&card.fgcode,(const TCHAR*)pname,FA_WRITE|FA_OPEN_ALWAYS))//write	FA_OPEN_ALWAYS:	如果文件存在，则打开；否则，创建一个新文件。
//		{
//			card.saving = true;
//			printf("Create a file successfully\r\n");
//		}
//  }
//   myfree(SRAMIN,pname);				//释放内存		
//}

//void card_removeFile(char* fname)
//{  u8 res;
//  res = f_unlink(fname);
//  if (res==FR_OK) 
//    {
//      printf("File deleted:");
//      printf(fname);
////      card.sdpos = 0;
//    }
//    else
//    {
//      printf("Deletion failed, File: ");
//      printf(fname);
//      printf(".\n");
//    }
//}

void card_startFileprint(void)
{
  if(card.cardOK)
  {
    card.sdprinting = true; 
  }

}

void card_pauseSDPrint(void)
{
  if(card.sdprinting)
  {
    card.sdprinting = false;
  }
}

void card_setIndex(long index)
{//card.sdpos = index;
 f_lseek(&card.fgcode,index);//设置光标在文件的位置
}

void card_getStatus(void)
{  if(card.cardOK)
  {
    printf(MSG_SD_PRINTING_BYTE);
    printf("%d",f_tell(&card.fgcode));
    printf("/");
    printf("%d",f_size(&card.fgcode));
  }
  else{
    printf(MSG_SD_NOT_PRINTING);
  }
  printf("\n");
}

void card_closefile(void)
{ f_close(&card.fgcode);
  card.saving = false; 
}

void card_write_command(char *buf)  //20160401
{ 
	char* begin;
  char* npos;
  char* end;

  begin=mymalloc(SRAMIN,128);
	end=mymalloc(SRAMIN,3);
  if((npos = strchr(buf, 'N')) != NULL)
  {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r'; 
  end[2] = '\n';
//  end[3] = '\0';
//	f_write(&card.fgcode,"M117 ETE 06s\r\n",15,&bw);
	f_write(&card.fgcode,(u8*)begin,end-begin+3,&bw);  // f_write(&card.fgcode,"M117 ETE 06s\r\n",15,&bw);
//	f_write(&card.fgcode,(u8*)begin,end-begin+4,&bw);
//	card_closefile();
//	printf("jieshou\r\n");
//	printf("%s",begin);
	
	
//  file.writeError = false;
//  file.write(begin);
//  if (file.writeError)
//  {
//    SERIAL_ERROR_START;
//    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
//  }
  
	 myfree(SRAMIN,begin);				//释放内存	
	 myfree(SRAMIN,end);				//释放内存	
	
}

void card_checkautostart()
{
  if((!card.cardOK)&&(!SD_CD))
  {
		f_mount(0,fs[0]); 					 	//挂载SD卡 //20160412
    card_initsd();
    if(!card.cardOK) //fail
      return;
  }
  /*
  char autoname[30];
  sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
  for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
    autoname[i]=tolower(autoname[i]);
  dir_t p;

  root.rewind();
  
  bool found=false;
  while (root.readDir(p, NULL) > 0) 
  {
    for(int8_t i=0;i<(int8_t)strlen((char*)p.name);i++)
    p.name[i]=tolower(p.name[i]);
    //Serial.print((char*)p.name);
    //Serial.print(" ");
    //Serial.println(autoname);
    if(p.name[9]!='~') //skip safety copies
    if(strncmp((char*)p.name,autoname,5)==0)
    {
      char cmd[30];

      sprintf_P(cmd, PSTR("M23 %s"), autoname);
      enquecommand(cmd);
      enquecommand_P(PSTR("M24"));
      found=true;
    }
  }
  if(!found)
    lastnr=-1;
  else
    lastnr++;
*/	
}
void card_printingHasFinished(void)
{
    st_synchronize();

	   card.sdprinting=false;
     Abnormal_Flag = 0x06;
	
   	 //Discard_Buf_and_Block();//此处不能用这个函数，因为执行到这的时候，虽然命令都已填充完，但是填充的最后几条命令还都没有执行
		//Abnormal_Flag = 0x03;
     SPI_Flash_Erase_Sector(FILE_NAME_ADDRESS/1024/4);//清除存储文件名2018
	
//	   Discard_Buf_and_Block();
//    quickStop();
//    card_closefile();
//	starttime=0;
//	card.sdprinting = false;
//    if(SD_FINISHED_STEPPERRELEASE)
//    {
//      //  finishAndDisableSteppers();
//        enquecommand(PSTR(SD_FINISHED_RELEASECOMMAND));
//    }
//    autotempShutdown();
//		//setTargetHotend(0,tmp_extruder);//20161218
//		{//20170111
//			int e; 
//			for(e = 0; e < EXTRUDERS; e++) 
//			setTargetHotend(0,e);//20161125  20170111
//		 }
//		target_temperature_bed=0;
//		 {//20170111
//			int e; 
//			for(e = 0; e < EXTRUDERS; e++) 
//			heater_temp[e]=0;//20161125  20170111
//		 }
////		heater_0_temp = 0;//20160409
//    bed_temp = 0;
//		feedmultiply = 100;
//		LaserPower = 0;
//		
//		active_extruder = 0;//20161223
//		tmp_extruder = 0;
		
//		Print_end();//HMI 20170213
		
//		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);////20170213-8266
//		TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );//TIM_Cmd(TIM7, ENABLE); 
//	 printf("5.\r\n");
}

bool card_eof(void)
{ return f_eof(&card.fgcode);
}

int16_t card_get(void) 
{ //card.sdpos = f_tell(&card.fgcode);
  return  file_read();
}

int16_t file_read(void)
{ 	u8 buffer[2]; 
	u8 res;
	UINT br;
	res = f_read(&card.fgcode, buffer, 1, &br);
	if (res == FR_OK )
	{// printf("%s",buffer);
	return *buffer;//
	}else
	{ return -1;// printf("f_read() fail .. \r\n");  
	}
}
#endif //SDSUPPORT
