#include <STC_CPU.H>
#include <INTRINS.H>
#include <main.h>
#include <uart.h>
#include <AD.h>
#include <String.h>
#include <PWM_NEW.h>
#define uchar unsigned char
#define uint unsigned int
#define uint32 unsigned long


BYTE Lfastcounter,Lslowcounter,Rfastcounter,Rslowcounter;
bit Lfastflag,Lslowflag,Rfastflag,Rslowflag;
uint Lslowfq,Lstime,Lfastfq,Lftime;
uint Rslowfq,Rstime,Rfastfq,Rftime;

void IAP_READ(void);
void IAP_WRITE(void);
void IAP_CLEAR(void);
void Jiaohuan(u16,uint32);
void  savecombuf(BYTE* buf);
BYTE i=0;
BYTE timer1_flag=0;
BYTE timer2_flag=0;
BYTE rec_buf[20];
BYTE Set_cfg[20];
BYTE rpy_addr[5],snd_addr[5],timer1_counter,timer2_counter,t1_flag,t2_flag;
BYTE addr,addrlast;
BYTE timer3_counter,timer4_counter;
extern BYTE g_byBegT1buf,g_byEndT1buf,g_byBegR1buf,g_byEndR1buf;
extern BYTE g_byT1buf[T1BUF_MAX_LEN];
extern BYTE g_byR1buf[R1BUF_MAX_LEN],g_byR2buf[R2BUF_MAX_LEN];
extern BYTE rcv_flag,snd_flag,rcv_length,rcv_flag2,snd_flag2,rcv_length2;
BYTE snd_flag3;//串口2发送标志位
BYTE save_flag;//在有向命令缓存写指令时，不再写另一个，防止重复调用出现错误；
u16 spd_val;
extern BYTE Receive[20],Receive2[20];
extern void rcv_handle(BYTE rcv);
extern void rcv_handle2(BYTE rcv);
extern void Pwm_Init(void);
extern void AD_init();
void UartACK2(BYTE status);
void DealReadRegister(uchar addr,uchar len);
unsigned int crc16(unsigned char *puchMsg, unsigned int usDataLen); 
extern void Send1(BYTE* buf, BYTE len);
extern BYTE check_modbus(void);
extern unsigned int AD_get(BYTE channel);
extern BYTE t1_rcvtimeout,t2_rcvtimeout;


sbit REL = P2^0;
sbit DEL = P2^1;
sbit RER = P2^2;
sbit DER = P2^3;

uint32 temp_cl1,temp_cl2,temp_clz,cl_j,cl_y,cl_b,cl_d;
BYTE command[11][6];     //存放触摸屏发下来的需要向下位机发的指令，最后一位为1时发一个字节，为2时发两个字节，为4时发4个字节；                         
//u16 g_usRegister[32];  //该变量放置对应的MODBUS地址40001-40032
u16 g_usRegister[66]; 
//左面快速频率	40001,左面慢速频率	40002,左面快速时间	40003,左面慢速时间	40004
//左面长度设置	40005,左面探纱选择	40007
//右面快速频率	40008,右面慢速频率	40009,右面快速时间	40010,右面慢速时间	40011
//右面长度设置	40012,右面探纱选择	40014
//甲班卷绕产量	40015,乙班卷绕产量	40017,丙班卷绕产量	40019,丁班卷绕产量	40021
//累计总产量	40023,甲乙丙丁选择	40031
//左边当前卷绕速度	40025,右面当前卷绕速度	40026
//下位机统计总数个数	40027	
//置零按钮	40028
//左面故障指示	40029,右面故障指示	40030,设置下位机的个数40032


/***************延时子程序****************************/
void delay(char a)//
{
	uchar n;//
	uchar b;//
	for(b=a;b>0;b--)
	{
		n=210;
		while(n!=0) --n;
	}
}

BYTE Lfastcounter,	Lslowcounter,Rfastcounter,	Rslowcounter;
bit Lfastflag,Lslowflag,Rfastflag,Rslowflag;
uint Lslowfq,Lstime,Lfastfq,Lftime;
uint Rslowfq,Rstime,Rfastfq,Rftime;
extern void AD_work( BYTE val,BYTE channel);
sbit AOUT0 = P1^0;
sbit AOUT1 = P1^1;
sbit PWM0 = P1^3;
sbit PWM1 = P1^4;
sbit REL0 = P2^4;
sbit REL1 = P2^5;

												        

/************T1中断处理*****************/
void T1_IRQ(void) interrupt 3//10ms中断
{
 	TF1=0;
	TH1=0xdc;
	TL1=0x00;
	if(Lfastflag==1)
	{
		if(Lfastcounter>0)
			Lfastcounter--;
	}
	if(Lslowflag==1)
	{
		if(Lslowcounter>0)
			Lslowcounter--;
	}
	if(Rfastflag==1)
	{
		if(Rfastcounter>0)
			Rfastcounter--;
	}
	if(Rslowflag==1)
	{
		if(Rslowcounter>0)
			Rslowcounter--;
	}
	if (t2_rcvtimeout>0) t2_rcvtimeout--;
}
	

void T0_IRQ(void)interrupt 1 //定时间隔70ms
{
	TF0=0;
	TH0=0x04;
	TL0=0x00;
	if(snd_flag3==0)//命令已经发送完
	{
	 	if(command[0][0]!=0x00)
		{
			Send2(command[0],command[0][4],command[0][5]); //从串口2发出左面长度,0表示左面，1表示右面
			snd_flag3=1;
			timer2_counter=5;//350ms
			t2_flag=1;	
		}
	}	
	if(t2_flag==1)
	{
		if(timer2_counter>0)
			timer2_counter--;
		if(timer2_counter==0)
		{ 
			snd_flag3=0;
			timer2_flag++;
		}
	}
	if(timer2_flag>4)//如果4次后仍未收到数据，这放弃该指令，继续执行下面指令。
	{
		memset(command[0],0,6);//处理完毕,清除命令行;
		for(i=0;i<10;i++)
		{
			memcpy(command[i],command[i+1],6);
		}	  
		snd_flag3=0;
		timer2_flag=0;
		t2_flag=0;
	}
	if(timer3_counter>0)
		timer3_counter--;
	if(timer3_counter==0)//读地址位指令每21秒发一次先发左面再发右面
	{
		snd_addr[0]=0x01;
		snd_addr[1]=0x00;
		snd_addr[4]=2;
		snd_addr[5]=0;
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			save_flag=0;
		}
		//snd_addr[0]=0x01;
		//snd_addr[1]=0x00;
		//snd_addr[4]=2;
		snd_addr[5]=1; //left-right
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			save_flag=0;
		}
		timer3_counter=200; 	
	} 
	if(timer4_counter>0)
		timer4_counter--;

	if(timer4_counter==0)//读产量指令每7秒发一次
	{
		snd_addr[0]=0x04;
		snd_addr[4]=1;
		snd_addr[5]=0;
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			save_flag=0;
		}
		//snd_addr[0]=0x04;
		//snd_addr[4]=1;
		snd_addr[5]=1;
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			save_flag=0;
		}
		timer4_counter=100; 	
	}		
	  

}

/********************************************************************************************************************
该函数主要实现接收到的MODBUS读保持寄存器指令的处理
//触摸屏发送的命令有：01 03 00 00 00 06 XX XX  ; 01 03 00 06 00 06 XX XX ; 
01 03 00 10 00 05 XX XX; 01 03 00 15 00 02 XX XX;  读指令
当触摸屏有写操作时，发送的命令格式为：01 10 00 04 00 01 02 00 05 XX XX ；共计11个字节 
此外触摸屏还可能发送01 01 00 60 00 10 3D D8这类读线圈指令，该类命令可以不理。
g_byR1buf为处理缓冲区，当接收到完整的MODBUS命令后，将接收缓冲区复制到处理缓冲区
参数：addr保持寄存器的地址
********************************************************************************************************************/
void DealReadRegister(uchar addr,uchar len)
{
	
//	uchar  g_usRegister1[32];	   //32的数组g_usRegister来保存产量、长度等信息对应40001-40032
	uchar data1[80];
	uint crc ;

	memset(data1,0,80);
	data1[0] = 0x01; //上位机对应的MODBUS地址，
	data1[1] = 0x03; //说明当前是对读寄存器命令的响应
	data1[2] = len + len ;

	memcpy((char *)(data1+3),(char *)&g_usRegister[addr],data1[2]);  //返回的数据长度为寄存器长度的两倍

	//计算CRC
	crc = crc16(data1,data1[2] + 3);
	memcpy(data1 + data1[2] + 3,&crc,2);//将CRC的校验码也放到发送缓冲区

	//这里开始准备给触摸屏发送响应了。
	Send1(data1,data1[2] + 5) ;  
}

/********************************************************************************************************************
该函数主要实现接收到的MODBUS读保持寄存器指令的处理
//触摸屏发送的命令有：01 03 00 00 00 06 XX XX  ; 01 03 00 06 00 06 XX XX ; 
01 03 00 10 00 05 XX XX; 01 03 00 15 00 02 XX XX;  读指令
当触摸屏有写操作时，发送的命令格式为：01 10 00 04 00 01 02 00 05 XX XX ；共计11个字节 
此外触摸屏还可能发送01 01 00 60 00 10 3D D8这类读线圈指令，该类命令可以不理。
g_byR1buf为处理缓冲区，当接收到完整的MODBUS命令后，将接收缓冲区复制到处理缓冲区
参数：addr保持寄存器的地址
********************************************************************************************************************/
void DealWriteRegister(uchar addr,uchar val[4],uchar len)
{
	uchar data1[34];  //
	uint crc ;

	memset(data1,0,34);
	data1[0] = 0x01; //上位机对应的MODBUS地址，
	data1[1] = 0x10; //说明当前是对读寄存器命令的响应
	data1[2] = 0x00 ;
	data1[3] = addr;
	data1[4] = 0x00;
	data1[5] = 0x00;
	//计算CRC
	crc = crc16(data1,6);
	memcpy(data1 + 6,&crc,2);//将CRC的校验码也放到发送缓冲区

	//这里开始准备给触摸屏发送响应了。
	Send1(data1,8) ; 
	 

	//对写命令而言，还需要给下位机发送写命令。
		//左面快速频率	40001,左面慢速频率	40002,左面快速时间	40003,左面慢速时间	40004
		//左面长度设置	40005,左面探纱选择	40007
		//右面快速频率	40008,右面慢速频率	40009,右面快速时间	40010,右面慢速时间	40011
		//右面长度设置	40012,右面探纱选择	40014
		//甲班卷绕产量	40015,乙班卷绕产量	40017,丙班卷绕产量	40019,丁班卷绕产量	40021
		//累计总产量	40023,甲乙丙丁选择	40031
		//左边当前卷绕速度	40025,右面当前卷绕速度	40026
		//左面下位机统计个数	40027	  右面下位机统计个数 40034
		//置零按钮	40028
		//左面故障指示	40029,右面故障指示	40030,
		//左面设置下位机的个数40032	 右面设置下位机的个数  40033
	if(addr == 0 )//收到左面快速频率
	{
		memcpy(g_usRegister+addr,val,2);
	 	Lfastfq=(uint)val[1];
	}
	else if(addr == 1 )//收到左面慢速频率
	{
		memcpy(g_usRegister+addr,val,2);
	 	Lslowfq=(uint)val[1];
	}
	else if(addr == 2 )//收到左面快速时间
	{
		memcpy(g_usRegister+addr,val,2);
	 	 Lftime=((uint)val[0]<<8)|((uint)val[1]);
	}
	else if(addr == 3 )//收到左面慢速时间
	{
		memcpy(g_usRegister+addr,val,2);
	 	Lstime=((uint)val[0]<<8)|((uint)val[1]);
	}		
	else if (addr == 4 )//左面长度设置
	{
	//	memcpy(g_usRegister+addr,val,4); 
		snd_addr[0]=0x02;
		snd_addr[1]=val[3];//
		snd_addr[2]=val[0];//
		snd_addr[3]=val[1];//
		snd_addr[4]=4;//发送数据长度
		snd_addr[5]=0;//0表示左面，1表示右面
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			memcpy(g_usRegister+addr,val,4); 
			save_flag=0;
		} 
	}
	else if(addr == 7 )	//收到右面快速频率
	{
		memcpy(g_usRegister+addr,val,2);
	 	Rfastfq=(uint)val[1];
	}
	else if(addr == 8 )//收到右面慢速频率
	{
		memcpy(g_usRegister+addr,val,2);
	 	Rslowfq=(uint)val[1];
	}
	else if(addr == 9 )	//收到右面快速时间
	{
		memcpy(g_usRegister+addr,val,2);
	 	 Rftime=((uint)val[0]<<8)|((uint)val[1]);
	}
	else if(addr == 10 )//收到右面慢速时间
	{
		memcpy(g_usRegister+addr,val,2);
	 	Rstime=((uint)val[0]<<8)|((uint)val[1]);
	}
	else if (addr == 11 )//右面长度设置
	{
		memcpy(g_usRegister+addr,val,4); 
		snd_addr[0]=0x02;
		snd_addr[1]=val[3];//
		snd_addr[2]=val[0];//
		snd_addr[3]=val[1];//
		snd_addr[4]=4;
		snd_addr[5]=1;//0表示左面，1表示右面	
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			save_flag=0;
		} 	
	}
 
	else if(addr == 6 )//收到左面设定探丝器
	{
	//	memcpy(g_usRegister+addr,val,2);
	 	snd_addr[0]=0X03;
		snd_addr[1]=val[1];
		snd_addr[4]=2;
		snd_addr[5]=0;
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			memcpy(g_usRegister+addr,val,2);
			save_flag=0;
		}
	}
 	else if(addr == 13)//收到右面设定探丝器
	{
	//	memcpy(g_usRegister+addr,val,2);		 
	 	snd_addr[0]=0X03;
		snd_addr[1]=val[1];
		snd_addr[4]=2;
		snd_addr[5]=1;
		if(save_flag==0)
		{	
			save_flag=1;
			savecombuf(snd_addr);
			memcpy(g_usRegister+addr,val,2);
			save_flag=0;
		}
	}
			 
	else if(addr == 31)
	{
		memcpy(g_usRegister+addr,val,2);
	}
	else if	(addr == 32)
	{
		memcpy(g_usRegister+addr,val,2); 
	}  
}




/*********************************************************************************************************************
孔亚广：以上部分请周少波看看哪些是在上位机没用的。
**********************************************************************************************************************/

//	PWM_clock(2);      // PCA/PWM时钟源为定时器0的溢出
//	PWM_start(2,0); // 模块0,设置为PWM输出,无中断,初始占空因素为25%
/*********************************************************************************************************************
孔亚广：这一部分看看PWM的输出还要求有哪些参数设置。
**********************************************************************************************************************/
   /************T0中断处理*****************/


void main(void) 
{ 
	uchar buf[4];
	REL=0; 
	DEL=0;
	RER=0;
	DER=0; 
	InitUart();
	InitUart2();
	Pwm_Init();
	AD_init();        //A/D转换初始化
	delay(100);
	TMOD =0x11; /* timer 0 mode 1: 8-Bit reload */ 
//	TH0=0x00;
//	TL0=0x3f;
	TH0=0x04;
	TL0=0x00;
	ET0=1;
	IT0=1;
	TF0=0;
	TR0=1;
//	TH1=0x0d;
//	TL1=0xbf;
	TH1=0xdc;
	TL1=0x00;
	ET1=1;
	IT1=1;
	TF1=0;
	TR1=1;  
	EA=1;    
	Lfastflag=1;
	Lslowflag=0; 
	Lfastfq=0;
	Lslowfq=0;
	Lftime=0;
	Lstime=0;
	Rfastflag=1;
	Rslowflag=0;		  
	Rfastfq=0;
	Rslowfq=0;
	Rftime=0;
	Rstime=0;
	REL0=0;
	REL1=0;
	temp_cl1=0;
	temp_cl2=0;
	temp_clz=0;
	cl_j=cl_y=cl_b=cl_d=0;
	rcv_flag=rcv_flag2=0;
	snd_flag=snd_flag2=0;
	snd_flag3=0;/////////////////////////////////////////////////////////////////////////
	save_flag=0;
	spd_val=0;
	timer3_counter=timer4_counter=0;
	memset(g_usRegister,0,132);
//	for(i=0;i<11;i++)
	{
		 memset(&command[0][0],0,66); 
	}
	 
	while(1) 
	{  
		if(Lfastflag==1)
		{
			if(Lfastcounter==0)	//左边快速频率运行时间到
			{
				CCAP0H =0xFF*(Lslowfq/10);	   //占空比设置
				Lslowflag=1;
				Lfastflag=0;
				Lslowcounter=Lstime;
		   		AD_work( CCAP0H,0);
			} 					
		}
		if(Lslowflag==1)
		{
			if(Lslowcounter==0)//左边慢速频率运行时间到
			{
				CCAP0H =0xFF*(Lfastfq/10);
				Lfastflag=1;
				Lslowflag=0;
				Lfastcounter=Lftime;
				AD_work( CCAP0H,0);
			}
					
		}
		if(Rfastflag==1)
		{
			if(Rfastcounter==0)//右边快速频率运行时间到
			{
				CCAP1H = 0xFF*(Rslowfq/10);
				Rslowflag=1;
				Rfastflag=0;
				Rslowcounter=Rstime;
				AD_work( CCAP1H,1);
			}
					
		}
		if(Rslowflag==1)
		{
			if(Rslowcounter==0)//右边慢速频率运行时间到
			{
				CCAP1H =0xFF*(Rfastfq/10);
				Rfastflag=1;
				Rslowflag=0;
				Rfastcounter=Rftime;
				AD_work( CCAP1H,1);
			}
				
		}
/*********************************************************
这一点是写卷绕速度
*******************************************************/				 
	   //spd_val=AD_get(0);
	   //memcpy(&g_usRegister[24],&spd_val,2);
	   //spd_val=AD_get(1);
	   //memcpy(&g_usRegister[25],&spd_val,2);

/********************************************************************************************************************
	//串口1与触摸屏连接。
	//触摸屏发送的命令有：01 03 00 00 00 06 XX XX  ; 01 03 00 06 00 06 XX XX ; 
	                      01 03 00 10 00 05 XX XX; 01 03 00 15 00 02 XX XX;  读指令
	当触摸屏有写操作时，发送的命令格式为：01 10 00 04 00 01 02 00 05 XX XX ；共计11个字节 
	                                      01 10 00 04 00 02 04 00 00 00 00 XX XX									
	此外触摸屏还可能发送01 01 00 60 00 10 3D D8这类读线圈指令，该类命令可以不理。
*********************************************************************************************************************/
		if (rcv_flag > 0)
		{
			//串口1收到的触摸屏发过来的数据
			//判断是否为一个完整的MODBUS命令。如果是完整的MODBUS命令，则将数据从接收缓冲区放到处理缓冲区进行处理，同时Rcv_flag置0，接收缓冲区复位。
			snd_flag = check_modbus(); 
		}

		if(snd_flag == 1)	 //g_byR1buf为处理缓冲区，当接收到完整的MODBUS命令后，将接收缓冲区复制到处理缓冲区
		{
			//如果触摸屏发来的是读命令，则从串口1将当前的寄存器值发送到触摸屏
			switch (g_byR1buf[1])  //
			{
			case 0x03:  //读保持寄存器命令，如产量、长度、下位机个数等信息
				//这里要做处理		
				DealReadRegister(g_byR1buf[3],g_byR1buf[5]);   //要读的保持寄存器的地址
				break;
			case 0x01:  //读线圈
				//该命令应该可以不做处理
				break; 
		    case 0x10:  //写寄存器，当为写命令时，一方面要从UART1给触摸屏响应，另一方要从串口2给下位机将当前的寄存器更新
				//这里要区分要写的是两字节（频率）还是四字节（长度）
				if (g_byR1buf[5] == 0x01)
				{
					buf[0] = g_byR1buf[7];
					buf[1] = g_byR1buf[8];
					buf[2] = 0x00;
					buf[3] = 0x00;
					DealWriteRegister(g_byR1buf[3],buf,1);  //这里肯定是写一个寄存器，我们就简单处理吧。
				}
				if (g_byR1buf[5] == 0x02)
				{
					buf[0] = g_byR1buf[7];
					buf[1] = g_byR1buf[8];
					buf[2] = g_byR1buf[9];
					buf[3] = g_byR1buf[10];
					DealWriteRegister(g_byR1buf[3],buf,2);  //这里肯定是写一个寄存器，我们就简单处理吧。
				}
				break; 
			default:	//其余命令，我们就不做处理了，触摸屏应该不会发。
				break; 
			}
			snd_flag = 0;  //上次命令已经处理好了。
		}

		if(rcv_flag2 > 0)  //如果收到
		{
			if (Receive2[rcv_flag2-1] == 0xfe)
				rcv_handle2(Receive2[0]);
			if (t2_rcvtimeout == 0)
			{
				memset(Receive2,0,20);
				rcv_flag2 = 0;
			}
		 }
		
		if(snd_flag2 == 1)	   //收到下位机的解包处理完	 存放在g_byR2buf
		{
			if(g_byR2buf[0]!=0x09)
			{
				if (g_byR2buf[0] != 0x00)
					UartACK2(g_byR2buf[0]);//先回复ACK,再处理数据;
				t2_flag=0;
				if(g_byR2buf[0]==0x08)//返回发送产量
				{
					if(command[0][5]==0)
						temp_cl1=((uint32)g_byR2buf[1]<<24)|((uint32)g_byR2buf[2]<<16)|((uint32)g_byR2buf[3]<<8)|(uint32)g_byR2buf[4];
		  			else
				   	   temp_cl2=((uint32)g_byR2buf[1]<<24)|((uint32)g_byR2buf[2]<<16)|((uint32)g_byR2buf[3]<<8)|(uint32)g_byR2buf[4];
					if (g_usRegister[30] == 0x01) //甲班
					{
						cl_j = temp_cl1 + temp_cl2; //累计产量
						Jiaohuan(14,cl_j);
					}
					else if (g_usRegister[30] == 0x02)//乙班
					{
						cl_y = temp_cl1 + temp_cl2; //累计产量
						Jiaohuan(16,cl_j);
					}
					else if (g_usRegister[30] == 0x03)
					{
						cl_b = temp_cl1 + temp_cl2; //累计产量
						Jiaohuan(18,cl_j);
					}
					else if (g_usRegister[30] == 0x04)
					{
						cl_d = temp_cl1 + temp_cl2; //累计产量
						Jiaohuan(20,cl_j);
					}	
						temp_clz = cl_j+cl_y+cl_b+cl_d;
						Jiaohuan(22,cl_j);
						//temp_cl1 = 0;
					//	temp_cl2 = 0;
					}
					if(g_byR2buf[0]==0x05)   //返回地址设定
					{
			 			if(command[0][5]==0)
						{
							g_usRegister[26] = g_byR2buf[1];
							if(g_usRegister[26]!=g_usRegister[31])
							g_usRegister[28]=0x0001;
							else 
							g_usRegister[28]=0x0000;
						}
						else
						{
							g_usRegister[33] = g_byR2buf[1];
							if(g_usRegister[33]!=g_usRegister[32])
							g_usRegister[29]=0x0001;
							else 
							g_usRegister[29]=0x0000;
						}
					}
					memset(command[0],0,6);//处理完毕,清除命令行;
						for(i=0;i<10;i++)
						{
							memcpy(command[i],command[i+1],6);
						}
						snd_flag3=0;
					
				}	
				if(g_byR2buf[0]==0x09)//收到ACK
				{
					//if(g_byR2buf[1]==command[0][0]) //////////////////////////////////////////////////////////
					t2_flag=0;	
					memset(g_byR2buf,0,30);			
				}
		   	snd_flag2=0;
			}	 
	}
}
void UartACK2(BYTE status)
{
	BYTE ack[2];
	ack[0]=0x09;
	ack[1]=status;
    Send2(ack,2,command[0][5]);
}
void Jiaohuan(u16 addr,uint32 in)
{
	u16 in1=0,in2=0;
	in2=(u16)(in&0x0000ffff);
	in1=(u16)in>>16;
	in=(uint32)((((uint32)in2)<<16)+in1);
	memcpy((char*)(&g_usRegister[addr]),(char*)&in,4);
}
void  savecombuf(BYTE* buf)
{
 	BYTE i=0, j=0;
	for(i=0;i<10;i++)
	{
	
		if((command[i][0]==0x00)||((buf[0]==command[i][0])&&(buf[5]==command[i][5]))) //如果前面已经有该指令，则覆盖，没有的话放最后
		{	
			for(j=0;j<6;j++)
			{
				command[i][j]=buf[j];	
			}
		  break;
		}
	}
}