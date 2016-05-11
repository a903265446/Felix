
/*SD��SPIģʽ����ʵ���ļ�*/


#include "SD_Drv.h"


/*��ʱ����*/
void Delay_100us(void)
{	
	unsigned int i=8;
	while(i--)
		*P_Watchdog_Clear=0x0001;
}
//IOB�ڳ�ʼ������	CS,DO,CLK����Ϊ�ߵ�ƽ���	DI����Ϊ�޻��ѵ���������
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



/*��SD������һ���ֽڵ�����*/

void	SPI_Send_Byte(unsigned char Data)
{
	unsigned int	nCount;
	unsigned char	Buffer;
	Buffer=Data;				
	for(nCount=0;nCount<8;nCount++)
	{
		SPI_CLK_RESET();			//CLK�õ��Ա�����������
		
		if(Buffer&0x80)			//��������Ϊ1
			SPI_DOUT_SET();				//DO����ߵ�ƽ
		
		else					//����
			SPI_DOUT_RESET();			//D0����͵�ƽ	
		
		SPI_CLK_SET();				//����ʱ��������
		
		Buffer=Buffer<<1;				// CLK�ø�								
	}			
}


/*��SD����ȡһ���ֽڵ�����*/

unsigned char SPI_Get_Byte(void)
{
	unsigned char	echo;
	unsigned int	nCount;
	echo=0;
	for(nCount=0;nCount<8;nCount++)
	{
		echo=echo<<1;
		SPI_CLK_RESET();	//CLK�õ��Ա�����������
		
		SPI_CLK_SET();		//CLK�ø߲��������أ���ȡ����
		
		if(*P_IOB_Data & SPI_DIN)
			echo|=0x01;
		else
			echo&=0xfe;		
	*P_Watchdog_Clear = 1;		
	}
	
	return	echo;
	
}

/*��SD����������*/

unsigned char	SPI_Send_Cmd(unsigned char *Cmd)
{
	unsigned char tmp;
  	 unsigned char retry=0;
	 unsigned char i;
	 
	 SPI_CS_SET();			//��ֹSD��
	 
	 SPI_Send_Byte(0xff);		
	 SPI_CS_RESET();				//ʹSD��ʹ��
	 
	 for (i=0;i<6;i++) 
	 { 
		 SPI_Send_Byte(*Cmd++);
	 }
	 
	 //���8λ�Ļ�Ӧ
	 SPI_Get_Byte();
	 do 
	 {  //��ȡ��Ӧ
		 tmp = SPI_Get_Byte();
		 retry++;
	 }
	 while((tmp==0xff)&&(retry<100)); 
	 return(tmp);
}

/*SD����ʼ������*/
unsigned char	SPI_Init(void)
{	
	unsigned char retry,temp;
	unsigned char i;
	//unsigned	Init_Flag;
	unsigned char CMD[] = {0x40,0x00,0x00,0x00,0x00,0x95};
	SPI_IO_Init(); //��ʼ�������˿�
	
	Delay_100us();
	for (i=0;i<16;i++) 
	{
		SPI_Send_Byte(0xff); //��������74��ʱ���ź�
	}
	SPI_CS_RESET();			//ʹSD��ʹ��
	
	//��SD������CMD0
	retry=0;
	do
	{ //Ϊ���ܹ��ɹ�д��CMD0,������д200��
		temp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==200) 
		{ //����200��
			return(INIT_CMD0_ERROR);//CMD0 Error!
		}
	} 
	while(temp!=1);  //��Ӧ01h��ֹͣд��
	
	//����CMD1��SD��
	CMD[0] = 0x41; //CMD1
	CMD[5] = 0xFF;
	retry=0;
	do
	{ //Ϊ���ܳɹ�д��CMD1,д100��
		temp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==100) 
		{ //����100��
			return(INIT_CMD1_ERROR);//CMD1 Error!
		}
	} 
	while(temp!=0);//��Ӧ00hֹͣд��
    
    SPI_CS_SET();  //ʹSD��Ƭѡ��Ч
	
	return(INIT_SUCCESSED); //��ʼ���ɹ�
	
}		


//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
unsigned char SD_write_sector(unsigned long addr,unsigned char *Buffer)
{  
	unsigned char tmp,retry;
	unsigned int i;
	//����24
	unsigned char CMD[] = {0x58,0x00,0x00,0x00,0x00,0xFF}; 
	addr = addr << 9; //addr = addr * 512
	
	CMD[1] = ((addr & 0xFF000000) >>24 );
	CMD[2] = ((addr & 0x00FF0000) >>16 );
	CMD[3] = ((addr & 0x0000FF00) >>8 );
	
	//д����24��SD����ȥ
	retry=0;
	do
	{  //Ϊ�˿ɿ�д�룬д100��
		tmp=SPI_Send_Cmd(CMD);
		retry++;
		if(retry==100) 
		{ 
			return(tmp); //send commamd Error!
		}
	}
	while(tmp!=0); 
	
	
	//��д֮ǰ�Ȳ���100��ʱ���ź�
	for (i=0;i<100;i++)
	{
		SPI_Get_Byte();
	}
	
	//д�뿪ʼ�ֽ�
	SPI_Send_Byte(0xFE);	
	
	//���ڿ���д��512���ֽ�
	for (i=0;i<512;i++)
	{
		SPI_Send_Byte(*Buffer++); 
	}
	
	//CRC-Byte 
	SPI_Send_Byte(0xFF); //Dummy CRC
	SPI_Send_Byte(0xFF); //CRC Code
	
    
	tmp=SPI_Get_Byte();   // read response
	if((tmp & 0x1F)!=0x05) // д���512���ֽ���δ������
	{
		// SPI_CS_SET();
		*P_IOB_Data|=SPI_CS;
		return(WRITE_BLOCK_ERROR); //Error!
	}
	//�ȵ�SD����æΪֹ
	//��Ϊ���ݱ����ܺ�SD�����򴢴������б������
	while (SPI_Get_Byte()!=0xff){};
	
	//��ֹSD��
	// SPI_CS_SET();
	*P_IOB_Data|=SPI_CS;
	return(0);//д��ɹ�
}


//----------------------------------------------------------------------------
//   ��ȡ���ݵ�buffer��
//----------------------------------------------------------------------------
void SD_get_data(unsigned int Bytes,unsigned char *buffer) 
{
   unsigned int j;
   for (j=0;j<Bytes;j++)
      *buffer++ = SPI_Get_Byte();
}


//-------------------------------
//��һ������
//-------------------------------------
unsigned char SD_Read_Sector(unsigned long sector,unsigned char *buffer)
{  
   unsigned char retry;
   //����16
   unsigned char CMD[] = {0x51,0x00,0x00,0x00,0x00,0xFF}; 
   unsigned char temp;
   
   //��ַ�任   ���߼����ַתΪ�ֽڵ�ַ
   sector = sector << 9; //sector = sector * 512

   CMD[1] = ((sector & 0xFF000000) >>24 );
   CMD[2] = ((sector & 0x00FF0000) >>16 );
   CMD[3] = ((sector & 0x0000FF00) >>8 );

   //������16д��SD��
   retry=0;
   do
   {  //Ϊ�˱�֤д������  һ��д100��
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
  SD_get_data(512,buffer) ;  //512�ֽڱ�������buffer��
 return 0;
}


