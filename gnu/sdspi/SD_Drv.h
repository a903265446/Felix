/*SD��spiģʽ����ͷ�ļ�*/

#ifndef	SD_SPI
#define	SD_SPI

//IOB�ڵ�8λ����
#define		SPI_CS	 	 0x0080		//SD��Ƭѡ			IOB.7
#define		SPI_DOUT	 0x0040		//��Ƭ������������	IOB.6
#define		SPI_CLK		 0x0020		//��Ƭ������������	IOB.5
#define		SPI_DIN	 	 0x0010		//SD��ʱ���ź�		IOB.4

/*��ʼ��������*/
#define		INIT_CMD0_ERROR				0x01	
#define		INIT_CMD1_ERROR				0x02
#define		INIT_SUCCESSED				0x00
#define		WRITE_BLOCK_ERROR			0x03
#definee	READ_BLOCK_ERROR			0x04

#define		SPI_CS_SET()				*P_IOB_Data|=SPI_CS
#define		SPI_CS_RESET()				*P_IOB_Data&=~SPI_CS

#define		SPI_CLK_SET()				*P_IOB_Data|=SPI_CLK
#define		SPI_CLK_RESET()					*P_IOB_Data&=~SPI_CLK

#define		SPI_DOUT_SET()				*P_IOB_Data|=SPI_DOUT
#define		SPI_DOUT_RESET()				*P_IOB_Data&=~SPI_DOUT

#define		SPI_DIN_SET()					*P_IOB_Data|=SPI_DIN
#define		SPI_DIN_RESET()				*P_IOB_Data&=~SPI_DIN




void		SPI_Send_Byte(unsigned char Data);
unsigned char 	SPI_Get_Byte(void);
unsigned char	SPI_Send_Cmd(unsigned char *Cmd);
unsigned char	SPI_Init(void);
unsigned char 	SD_write_sector(unsigned long addr,unsigned char *Buffer);
unsigned char 	SD_Read_Sector(unsigned long sector,unsigned char *buffer);

#endif