
/*SD卡SPI模式驱动实现文件*/


#include "SD_Drv.h"


/*延时函数*/
void Delay_100us(void)
{	
	unsigned int i=8;
	while(i--)
		*P_Watchdog_Clear=0x0001;
}
//IOB口初始化函数	CS,DO,CLK设置为高电平输出	DI设置为无唤醒的悬浮输入
void	SPI_IO_Init(void)
{
	unsigned int	tmp;
	tmp=*P_IOB_Dir;
	tmp |= (SPI_CS+SPI_DOUT+SPI_CLK);
	tmp&=~SPI_DIN;
	*P_IOB_Dir=tmp;
	
	tmp=*P_IOB_Attrib;
	tmp|=(SPI_CS+SPI_DOUT+SPI_CLK+SPI_DIN);
	*P_IOB_Attrib=tmp;
	
	tmp=*P_IOB_Data;
	tmp|=(SPI_CS+SPI_DOUT+SPI_CLK+SPI_DIN);
	*P_IOB_Data=tmp;
}



/*向SD卡发送一个字节的数据*/

void	SPI_Send_Byte(unsigned char Data)
{
	unsigned int	nCount;
	unsigned char	Buffer;
	Buffer=Data;				
	for(nCount=0;nCount<8;nCount++)
	{
		SPI_CLK_RESET();			//CLK置低以备产生上升沿
		
		if(Buffer&0x80)			//待发数据为1
			SPI_DOUT_SET();				//DO输出高电平
		
		else					//否则
			SPI_DOUT_RESET();			//D0输出低电平	
		
		SPI_CLK_SET();				//产生时钟上升沿
		
		Buffer=Buffer<<1;				// CLK置高								
	}			
}


/*从SD卡读取一个字节的数据*/

unsigned char SPI_Get_Byte(void)
{
	unsigned char	echo;
	unsigned int	nCount;
	echo=0;
	for(nCount=0;nCount<8;nCount++)
	{
		echo=echo<<1;
		SPI_CLK_RESET();	//CLK置低以备产生上升沿
		
		SPI_CLK_SET();		//CLK置高产生上升沿，读取数据
		
		if(*P_IOB_Data & SPI_DIN)
			echo|=0x01;
		else
			echo&=0xfe;		
	*P_Watchdog_Clear = 1;		
	}
	
	return	echo;
	
}

/*向SD卡发送命令*/

unsigned char	SPI_Send_Cmd(unsigned char *Cmd)
{
	unsigned char tmp;
  	 unsigned char retry=0;
	 unsigned char i;
	 
	 SPI_CS_SET();			//禁止SD卡
	 
	 SPI_Send_Byte(0xff);		
	 SPI_CS_RESET();				//使SD卡使能
	 
	 for (i=0;i<6;i++) 
	 { 
		 SPI_Send_Byte(*Cmd++);
	 }
	 
	 //获得8位的回应
	 SPI_Get_Byte();
	 do 
	 {  //读取响应
		 tmp = SPI_Get_Byte();
		 retry++;
	 }
	 while((tmp==0xff)&&(retry<100)); 
	 return(tmp);
}

/*SD卡初始化程序*/
unsigned char	SPI_Init(void)
{	
	unsigned char retry,temp;
	unsigned char i;
	//unsigned	Init_Flag;
	unsigned char CMD[] = {0x40,0x00,0x00,0x00,0x00,0x95};
	SPI_IO_Init(); //初始化驱动端口
	
	Delay_100us();
	for (i=0;i<16;i++) 
	{
		SPI_Send_Byte(0xff); //发送至少74个时钟信号
	}
	SPI_CS_RESET();			//使SD卡使能
	
	//向SD卡发送CMD0
	retry=0;
	do
	{ //为了能够成功写入CMD0,在这里写200次
		temp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==200) 
		{ //超过200次
			return(INIT_CMD0_ERROR);//CMD0 Error!
		}
	} 
	while(temp!=1);  //回应01h，停止写入
	
	//发送CMD1到SD卡
	CMD[0] = 0x41; //CMD1
	CMD[5] = 0xFF;
	retry=0;
	do
	{ //为了能成功写入CMD1,写100次
		temp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==100) 
		{ //超过100次
			return(INIT_CMD1_ERROR);//CMD1 Error!
		}
	} 
	while(temp!=0);//回应00h停止写入
    
    SPI_CS_SET();  //使SD卡片选无效
	
	return(INIT_SUCCESSED); //初始化成功
	
}		


//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
unsigned char SD_write_sector(unsigned long addr,unsigned char *Buffer)
{  
	unsigned char tmp,retry;
	unsigned int i;
	//命令24
	unsigned char CMD[] = {0x58,0x00,0x00,0x00,0x00,0xFF}; 
	addr = addr << 9; //addr = addr * 512
	
	CMD[1] = ((addr & 0xFF000000) >>24 );
	CMD[2] = ((addr & 0x00FF0000) >>16 );
	CMD[3] = ((addr & 0x0000FF00) >>8 );
	
	//写命令24到SD卡中去
	retry=0;
	do
	{  //为了可靠写入，写100次
		tmp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==100) 
		{ 
			return(tmp); //send commamd Error!
		}
	}
	while(tmp!=0); 
	
	
	//在写之前先产生100个时钟信号
	for (i=0;i<100;i++)
	{
		SPI_Get_Byte();
	}
	
	//写入开始字节
	SPI_Send_Byte(0xFE);	
	
	//现在可以写入512个字节
	for (i=0;i<512;i++)
	{
		SPI_Send_Byte(*Buffer++); 
	}
	
	//CRC-Byte 
	SPI_Send_Byte(0xFF); //Dummy CRC
	SPI_Send_Byte(0xFF); //CRC Code
	
    
	tmp=SPI_Get_Byte();   // read response
	if((tmp & 0x1F)!=0x05) // 写入的512个字节是未被接受
	{
		// SPI_CS_SET();
		*P_IOB_Data|=SPI_CS;
		return(WRITE_BLOCK_ERROR); //Error!
	}
	//等到SD卡不忙为止
	//因为数据被接受后，SD卡在向储存阵列中编程数据
	while (SPI_Get_Byte()!=0xff){};
	
	//禁止SD卡
	// SPI_CS_SET();
	*P_IOB_Data|=SPI_CS;
	return(0);//写入成功
}


//----------------------------------------------------------------------------
//   获取数据到buffer中
//----------------------------------------------------------------------------
void SD_get_data(unsigned int Bytes,unsigned char *buffer) 
{
   unsigned int j;
   for (j=0;j<Bytes;j++)
      *buffer++ = SPI_Get_Byte();
}


//-------------------------------
//读一个扇区
//-------------------------------------
unsigned char SD_Read_Sector(unsigned long sector,unsigned char *buffer)
{  
   unsigned char retry;
   //命令16
   unsigned char CMD[] = {0x51,0x00,0x00,0x00,0x00,0xFF}; 
   unsigned char temp;
   
   //地址变换   由逻辑块地址转为字节地址
   sector = sector << 9; //sector = sector * 512

   CMD[1] = ((sector & 0xFF000000) >>24 );
   CMD[2] = ((sector & 0x00FF0000) >>16 );
   CMD[3] = ((sector & 0x0000FF00) >>8 );

   //将命令16写入SD卡
   retry=0;
   do
   {  //为了保证写入命令  一共写100次
      temp=SPI_Send_Cmd(CMD);
      retry++;
      if(retry==100) 
      {
        return(READ_BLOCK_ERROR); //block write Error!
      }
   }
   while(temp!=0); 

   while (SPI_Get_Byte() != 0xfe);
   readPos=0;
  SD_get_data(512,buffer) ;  //512字节被读出到buffer中
 return 0;
}


