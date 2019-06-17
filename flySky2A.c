#include "STC15.h"
#include <string.h>
#include <intrins.h>
#include <stdio.h>



/////////////////////////////////////////////////////
//引脚
/////////////////////////////////////////////////////
sbit LED = P3^2;
sbit KEY = P3^3;
sbit PPM_OUT = P3^0;


//不可更改
//sbit A7105_SCK	= P1^5;
//sbit A7105_SDA 	= P1^3;			//MOSI
//sbit A7105_GIO2 = P1^4;			//MISO
//sbit A7105_GIO1 = 	P1^1;

sbit A7105_CS		= 	P1^2;


/////////////////////////////////////////////////////
//数据类型
/////////////////////////////////////////////////////
typedef unsigned char uint8_t;
typedef unsigned int uint16_t; 
typedef unsigned long uint32_t;
typedef enum
{
	false = 0,
	true = 1
}bool;

typedef uint16_t tick_t ;

typedef union
{
	uint16_t raw;
	struct
	{
		uint8_t h;			//高8位
		uint8_t l;			//低8位,51里大端模式
	}parse;
}halfword_t;



/////////////////////////////////////////////////////
//宏
/////////////////////////////////////////////////////
//#define FOSC 12000000			//晶振频率(M)
//#define FOSC 11059200			//晶振频率(M)
//#define FOSC 30000000			//晶振频率(M)
//#define FOSC 20000000			//晶振频率(M)
#define FOSC 24000000			//晶振频率(M)


//
#define _BIT(_bit) (1<<_bit)
#define READ_BIT(_byte,_bit) (_byte&(1<<_bit))
#define CLR_BIT(_byte,_bit) (_byte &= ~(1<<_bit))
#define SET_BIT(_byte,_bit) (_byte |= (1<<_bit))
#define gpio_setMode_IO(_port,_pin) 	do{ CLR_BIT(P##_port##M0,_pin); CLR_BIT(P##_port##M1,_pin);}while(0)	//准双向口
#define gpio_setMode_PP(_port,_pin) 	do{ SET_BIT(P##_port##M0,_pin); CLR_BIT(P##_port##M1,_pin);}while(0)	//推挽输出
#define gpio_setMode_FL(_port,_pin) 	do{ CLR_BIT(P##_port##M0,_pin); SET_BIT(P##_port##M1,_pin);}while(0)	//高阻输入
#define gpio_setMode_OD(_port,_pin) 	do{ SET_BIT(P##_port##M0,_pin); SET_BIT(P##_port##M1,_pin);}while(0)	//开漏输入


#define ID_ADDR_RAM	0xF1


#define MAX_FLYSKY_2A_CHANNEL_COUNT 								14
#define FLYSKY_FREQUENCY_COUNT      								16
#define FLYSKY_2A_PAYLOAD_SIZE      								37

#define FLYSKY_2A_CHANNEL_COUNT    									 8//MAX_FLYSKY_2A_CHANNEL_COUNT

#define TX_DELAY        (500*2)



/*********A7105****************/
/**
 * A7105 states for strobe
 */
typedef enum
{
    A7105_SLEEP     = 0x80,
    A7105_IDLE      = 0x90,
    A7105_STANDBY   = 0xA0,
    A7105_PLL       = 0xB0,
    A7105_RX        = 0xC0,
    A7105_TX        = 0xD0,
    A7105_RST_WRPTR = 0xE0,
    A7105_RST_RDPTR = 0xF0
} a7105_state_t;


/**
 * Register addresses
 */
typedef enum
{
    A7105_00_MODE         = 0x00,
    A7105_01_MODE_CONTROL = 0x01,
    A7105_02_CALC         = 0x02,
    A7105_03_FIFOI        = 0x03,
    A7105_04_FIFOII       = 0x04,
    A7105_05_FIFO_DATA    = 0x05,
    A7105_06_ID_DATA      = 0x06,
    A7105_07_RC_OSC_I     = 0x07,
    A7105_08_RC_OSC_II    = 0x08,
    A7105_09_RC_OSC_III   = 0x09,
    A7105_0A_CK0_PIN      = 0x0A,
    A7105_0B_GPIO1_PIN_I  = 0x0B,
    A7105_0C_GPIO2_PIN_II = 0x0C,
    A7105_0D_CLOCK        = 0x0D,
    A7105_0E_DATA_RATE    = 0x0E,
    A7105_0F_PLL_I        = 0x0F,
		A7105_0F_CHANNEL	  = 0x0F,
    A7105_10_PLL_II       = 0x10,
    A7105_11_PLL_III      = 0x11,
    A7105_12_PLL_IV       = 0x12,
    A7105_13_PLL_V        = 0x13,
    A7105_14_TX_I         = 0x14,
    A7105_15_TX_II        = 0x15,
    A7105_16_DELAY_I      = 0x16,
    A7105_17_DELAY_II     = 0x17,
    A7105_18_RX           = 0x18,
    A7105_19_RX_GAIN_I    = 0x19,
    A7105_1A_RX_GAIN_II   = 0x1A,
    A7105_1B_RX_GAIN_III  = 0x1B,
    A7105_1C_RX_GAIN_IV   = 0x1C,
    A7105_1D_RSSI_THOLD   = 0x1D,
    A7105_1E_ADC          = 0x1E,
    A7105_1F_CODE_I       = 0x1F,
    A7105_20_CODE_II      = 0x20,
    A7105_21_CODE_III     = 0x21,
    A7105_22_IF_CALIB_I   = 0x22,
    A7105_23_IF_CALIB_II  = 0x23,
    A7105_24_VCO_CURCAL   = 0x24,
    A7105_25_VCO_SBCAL_I  = 0x25,
    A7105_26_VCO_SBCAL_II = 0x26,
    A7105_27_BATTERY_DET  = 0x27,
    A7105_28_TX_TEST      = 0x28,
    A7105_29_RX_DEM_TEST_I  = 0x29,
    A7105_2A_RX_DEM_TEST_II = 0x2A,
    A7105_2B_CPC          = 0x2B,
    A7105_2C_XTAL_TEST    = 0x2C,
    A7105_2D_PLL_TEST     = 0x2D,
    A7105_2E_VCO_TEST_I   = 0x2E,
    A7105_2F_VCO_TEST_II  = 0x2F,
    A7105_30_IFAT         = 0x30,
    A7105_31_RSCALE       = 0x31,
    A7105_32_FILTER_TEST  = 0x32,
} a7105_reg_t;
/* Register: A7105_00_MODE */
#define A7105_MODE_FECF        0x40    // [0]: FEC pass. [1]: FEC error. (FECF is read only, it is updated internally while receiving every packet.)
#define A7105_MODE_CRCF        0x20    // [0]: CRC pass. [1]: CRC error. (CRCF is read only, it is updated internally while receiving every packet.)
#define A7105_MODE_CER         0x10    // [0]: RF chip is disabled. [1]: RF chip is enabled.
#define A7105_MODE_XER         0x08    // [0]: Crystal oscillator is disabled. [1]: Crystal oscillator is enabled.
#define A7105_MODE_PLLER       0x04    // [0]: PLL is disabled. [1]: PLL is enabled.
#define A7105_MODE_TRSR        0x02    // [0]: RX state. [1]: TX state. Serviceable if TRER=1 (TRX is enable).
#define A7105_MODE_TRER        0x01    // [0]: TRX is disabled. [1]: TRX is enabled.


/////////////////////////////////////////////////////
//flySky2A相关数据类型
/////////////////////////////////////////////////////
typedef struct {
    uint8_t type;
    uint8_t number;
    uint8_t valueL;
    uint8_t valueH;
} flySky2ASens_t;

typedef struct {
    uint8_t type;
    uint32_t txId;
    uint32_t rxId;
    flySky2ASens_t sens[7];
} flySky2ATelemetryPkt_t;

typedef struct {
    uint8_t type;			//1
    uint32_t txId;		//5
    uint32_t rxId;		//9
    uint8_t state;		//10
    uint8_t reserved1;	//11
    uint8_t rfChannelMap[16];		//
    uint8_t reserved2[10];
} flySky2ABindPkt_t;

typedef struct {
    uint8_t type;		//1
    uint32_t txId;	//5
    uint32_t rxId;	//9
    uint8_t dat[28];
} flySky2ARcDataPkt_t;

typedef struct {
    uint8_t type;
    uint32_t txId;
    uint8_t dat[16];
} flySkyRcDataPkt_t;

enum {
    SENSOR_INT_V = 0x00,
    SENSOR_TEMP = 0x01,
    SENSOR_MOT_RPM = 0x02,
    SENSOR_EXT_V = 0x03,
    SENSOR_RSSI = 0xFC,
    SENSOR_ERR_RATE = 0xFE
};

enum {
    FLYSKY_2A_PACKET_RC_DATA = 0x58,
    FLYSKY_2A_PACKET_BIND1 = 0xBB,
    FLYSKY_2A_PACKET_BIND2 = 0xBC,
    FLYSKY_2A_PACKET_FS_SETTINGS = 0x56,
    FLYSKY_2A_PACKET_SETTINGS = 0xAA,
    FLYSKY_2A_PACKET_TELEMETRY = 0xAA,
    FLYSKY_PACKET_RC_DATA = 0x55,
    FLYSKY_PACKET_BIND = 0xAA
};

typedef struct {
    tick_t packet;
    tick_t firstPacket;
    tick_t syncPacket;
    tick_t telemetry;
} timings_t;

/////////////////////////////////////////////////////
//变量
/////////////////////////////////////////////////////
code uint8_t flySky2ARegs[] = {
    0xff, 0x62, 0x00, 0x25, 0x00, 0xff, 0xff, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x19, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f,
    0x62, 0x80, 0xff, 0xff, 0x2a, 0x32, 0xc3, 0x1f,
    0x1e, 0xff, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00,
    0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f
};
code uint8_t flySky2ABindChannels[] = {
    0x0D, 0x8C
};

//code timings_t flySky2ATimings = {3850, 4850, 5775, 57000};



//对频参数
data uint32_t txId = 0;
data uint32_t rxId = 0;
data uint8_t rfChannelMap[FLYSKY_FREQUENCY_COUNT];


data uint8_t packet[FLYSKY_2A_PAYLOAD_SIZE];
//通道数据,1000-2000
//[173->998],[1812->2012],[993->1500]
halfword_t rcData[FLYSKY_2A_CHANNEL_COUNT] = {
1500,
1500,
1000,
1500,
	
1000,
1000,
1000,
1000,		
};


bit sbus_busy = 0;
data uint8_t sbus_cnt = 0;
uint8_t sbus_buffer[22];			//22个通道数据
halfword_t sbusData[FLYSKY_2A_CHANNEL_COUNT];
data tick_t timeLastSBUSPacket;



data uint8_t cnt33ms = 0;					//32.768ms
data uint8_t cnt33ms_led = 0;			//32.768ms


bit bound = false;
data uint8_t timeBindRequest = 0;


data tick_t timeout = 0;					//下个包的超时时间
data tick_t timeLastPacket = 0;		//上一次收到包的时间
data uint8_t countTimeout = 0;		//接收超时次数
data uint8_t countPacket = 0;			//接收包计数

bit waitTx = false;								//等待发送
data tick_t timeTxRequest;				//请求发送的时间

bit sendTelemetry = false;				//发送回传包请求

data uint8_t errorRate = 0;				//误码率
//data uint16_t rssi_dBm = 0;				//

data tick_t txrx_finished_time;
//bit txrx_finished = false;

data uint8_t adc_voltage = 0;	

/////////////////////////////////////////////////////
//common
/////////////////////////////////////////////////////

tick_t micros()
{
	register uint8_t th;
	register uint8_t tl;
	do
	{
		th = CH;
		tl = CL;
	}while(th != CH);	
	return (((tick_t)th<<8) + tl);
}

/*********************EEPROM**************************/
//每个扇区512字节
//STC15W408AS的EEPROM:10个扇区,0000H-13FFH
//IAP15W408AS:				16个扇区,0000H-1FFFH
//IAP15W4K61S4:				122个扇区,0000H-F3FFH

#if FOSC < 1000000
#define ENABLE_IAP 0x87		//SYSCLK<1 MHz
#elif FOSC < 2000000
#define ENABLE_IAP 0x86		//SYSCLK<2 MHz
#elif FOSC < 3000000
#define ENABLE_IAP 0x85		//SYSCLK<3 MHz
#elif FOSC < 6000000
#define ENABLE_IAP 0x84		//SYSCLK<6 MHz
#elif FOSC < 12000000
#define ENABLE_IAP 0x83		//SYSCLK<12 MHz
#elif FOSC < 20000000
#define ENABLE_IAP 0x82		//SYSCLK<20 MHz
#elif FOSC < 24000000
#define ENABLE_IAP 0x81		//SYSCLK<24 MHz
#elif FOSC < 30000000
#define ENABLE_IAP 0x80		//SYSCLK<30 MHz
#else
#define ENABLE_IAP 0x80		//???
#endif


#define	CMD_IDLE		0			//空闲模式
#define	CMD_READ 		1 		//字节读取
#define	CMD_PROGRAM 2 		//字节编程
#define CMD_ERASE 	3 		//扇区擦除

#define IAP_SECTOR_SIZE			512													//扇区大小,删除时会删除整个扇区
#define EEPROM_ADDRESS			0*IAP_SECTOR_SIZE					//用于EEPROM的起始地址

void IapIdle()
{
	IAP_CONTR = 0;				//关闭IAP功能
	IAP_CMD = 0;
	IAP_TRIG = 0;
	IAP_ADDRH = 0x80;
	IAP_ADDRL = 0;
}
void IapEraseSector(uint16_t addr)
{
	   IAP_CONTR = ENABLE_IAP;         //使能IAP
    IAP_CMD = CMD_ERASE;            //设置IAP命令
    IAP_ADDRL = addr;               //设置IAP低地址
    IAP_ADDRH = addr >> 8;          //设置IAP高地址
    IAP_TRIG = 0x5a;                //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                //写触发命令(0xa5)
    _nop_();                        //等待ISP/IAP/EEPROM操作完成
    IapIdle();
}

void eeprom_write_byte(uint16_t addr,uint8_t dat)
{
	IAP_CONTR = ENABLE_IAP;        
	IAP_CMD = CMD_PROGRAM;         
	IAP_ADDRL = addr;            
	IAP_ADDRH = addr >> 8;         
	IAP_DATA = dat;                 
	IAP_TRIG = 0x5a;               
	IAP_TRIG = 0xa5;                
	_nop_();                       
	IapIdle();
}

uint8_t eeprom_read_byte(uint16_t addr)
{
	uint8_t dat;
	IAP_CONTR = ENABLE_IAP;
	IAP_CMD = CMD_READ;
	IAP_ADDRL = addr;
	IAP_ADDRH = addr>>8;
	
	IAP_TRIG = 0x5A;
	IAP_TRIG = 0xA5;
	
	_nop_();
	dat = IAP_DATA;
	IapIdle();
	
	return dat;
}
void eeprom_write(uint16_t addr,const void *buffer,uint16_t len)
{
	uint8_t *p = (uint8_t*)buffer;
	while(len--)
	{
		eeprom_write_byte(addr++,*p++);
	}
}

void eeprom_read(uint16_t addr,void *buffer,uint16_t len)
{
	uint8_t *p = (uint8_t*)buffer;
	while(len--)
	{
		*p++ = eeprom_read_byte(addr++);
	}
}

//addr为扇区首地址
void eeprom_erase(uint16_t addr,uint16_t len)
{
	uint8_t num = (len+IAP_SECTOR_SIZE-1)/IAP_SECTOR_SIZE;
	while(num--)
	{
		IapEraseSector(addr);
		addr += IAP_SECTOR_SIZE;	
	}
}

/*********************SPI**************************/
//定义寄存器位
#define SSIG        0x80        //SPCTL.7                                 
#define SPEN        0x40        //SPCTL.6                                 
#define DORD        0x20        //SPCTL.5                                 
#define MSTR        0x10        //SPCTL.4                                 
#define CPOL        0x08        //SPCTL.3                                 
#define CPHA        0x04        //SPCTL.2  
#define SPIF        0x80        //SPSTAT.7                                
#define WCOL        0x40        //SPSTAT.6   
#define SPI_LSB 		0		//定义为1时低位先行,为0时高位先行
#define SPI_CPOL		0		//定义空闲时的电平

//定义时钟相位
//0 前时钟沿采样，后时钟沿输出
//1 前时钟沿输出，后时钟沿采样
//SPI总线速率@15W/15F
#define SPI_SPEED_4 0		//CPU_CLK/4
#define SPI_SPEED_8 1		//CPU_CLK/8
#define SPI_SPEED_16 2	//CPU_CLK/16
#define SPI_SPEED_32 3 	//CPU_CLK/32
#define SPI_CPHA    0
#define SPI_WOK (SPSTAT & SPIF)	//发送完成

void SPI_Init()
{
	SPSTAT = SPIF | WCOL;
	//SPI使能，主机，方向，空闲电平，时序，速度
	SPCTL = (SPI_LSB<<5)|(SPI_CPOL<<3)|(SPI_CPHA<<2) | SPI_SPEED_4; 		//忽略ss脚,主模式,
	SPCTL |= SSIG|SPEN| MSTR; 		//忽略ss脚,主模式,
}
#define SPI_RW(_v)	do{SPDAT = (_v);	while(!SPI_WOK);SPSTAT = SPIF | WCOL;}while(0)
#define SPI_DATA		(SPDAT)


/********************UART*****************/
//timer2作为波特率发生器
void uart_init_sbus()			//UART1初始化
{
	SCON = 0xda;                //9位可变波特率,校验位初始为1
	RI=0;
	TI = 0;
	//设置波特率发生器（T2 1T）
	T2L = 65536-FOSC/4/100000;				//100kbps
	T2H = (65536-FOSC/4/100000)>>8;
	AUXR = 0x14; 			           
	AUXR |= 0x01;     
		
	REN = 0;
	
	ES = 1;		//允许串口中断
	EA = 1;		//打开总中断

}

#define SBUS_PUTC(_c)	do{ACC = (_c);if (P)TB8 = 1;else TB8 = 0;SBUF = ACC;}while(0)
		 										//获取校验位
			   	//偶校验 
			

/********************TIMER0*****************/
/**********************
16BIT 	最大值 	65536/F(us)
F(M)		1T(ms)	12T(ms)
8				8.192 	98.304 
12			5.461 	65.536 
24			2.731 	32.768 
35			1.872 	22.469 
***********************/
#define TIMER_16BIT_12T_US(_) (uint16_t)(65536 - (_)*(FOSC/12000000))
#define TIMER_16BIT_1T_US(_) 	(uint16_t)(65536 - _*(FOSC/1000000))

#define	AUXR_T0x12			0x80
#define	AUXR_T1x12			0x20

void timer0_init()			
{
	//定时器0
	AUXR &= ~AUXR_T0x12;				//12T
	//AUXR |= AUXR_T0x12;				//1T
	TMOD &= 0xF0;						//16位自动重载
	TL0 = TIMER_16BIT_12T_US(400);
	TH0 = TIMER_16BIT_12T_US(400)>>8;		//4ms
	
	ET0 = 1;
	TR0 = 1;	
	EA = 1;				//打开总中断
}

/********************PCA*****************/
//系统时间,下降沿捕获
void pca_init()
{
	CCON = 0;                       //初始化PCA控制寄存器
																	//PCA定时器停止
																	//清除CF标志
																	//清除模块中断标志
	CL = 0;                         //复位PCA寄存器
	CH = 0;
	//CMOD = 0x00;                    //设置PCA时钟源，禁止PCA定时器溢出中断
	CMOD = 0x01;											//设置PCA时钟源,使能PCA定时器溢出中断
	
	CR = 1;                         //PCA定时器开始工作
	
	CCAPM0 = 0x10;                  //PCA模块0为16位捕获模式(下降沿捕获,可测从低电平开始的整个周期),禁止捕获中断
	
}
/********************ADC*****************/
#define ADC_POWER   0x80            //ADC电源控制位
#define ADC_FLAG    0x10            //ADC完成标志
#define ADC_START   0x08            //ADC起始控制位
#define ADC_SPEEDLL 0x00            //540个时钟
#define ADC_SPEEDL  0x20            //360个时钟
#define ADC_SPEEDH  0x40            //180个时钟
#define ADC_SPEEDHH 0x60            //90个时钟

#define ADC_BEGIN(_ch)	do{ADC_CONTR = ADC_POWER | ADC_SPEEDLL | _ch | ADC_START;}while(0)
void adc_init()
{
	P1ASF = 0x01;                   	//设置P1口为AD口
	ADC_RES = 0;                    	//清除结果寄存器
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL;	
	ADC_BEGIN(0);
}



/////////////////////////////////////////////////////
//A7105
/////////////////////////////////////////////////////
void A7105SoftReset(void)
{
	A7105_CS = 0;
	SPI_RW(A7105_00_MODE);
	SPI_RW(0);
	A7105_CS = 1;
}


#define A7105ReadReg_D(_reg,_ret)	do{A7105_CS = 0;SPI_RW((_reg) | 0x40);SPI_RW(0xFF);A7105_CS = 1;_ret = SPI_DATA;}while(0)
uint8_t A7105ReadReg(uint8_t reg)
{
	A7105_CS = 0;
	SPI_RW(reg | 0x40);
	SPI_RW(0xFF);
	A7105_CS = 1;
	return SPI_DATA;
}

#define A7105WriteReg_D(_reg,_dat)	do{A7105_CS = 0;SPI_RW((_reg));SPI_RW(_dat);A7105_CS = 1;}while(0)
void A7105WriteReg(uint8_t reg, uint8_t dat)
{
  A7105_CS = 0;
	SPI_RW(reg);
	SPI_RW(dat);
	A7105_CS = 1;
}

#define A7105Strobe(_s)	do{ A7105_CS = 0;SPI_RW(_s);A7105_CS = 1;  }while(0)
//void A7105Strobe(uint8_t state)
//{	
//  A7105_CS = 0;
//	SPI_RW(state);
//	A7105_CS = 1;  
//}


void A7105WriteID(uint32_t id)
{
		A7105_CS = 0;
		SPI_RW(A7105_06_ID_DATA);
		SPI_RW((id >> 24));
		SPI_RW((id >> 16));
		SPI_RW((id >> 8));
		SPI_RW((id >> 0));
		A7105_CS = 1;  
}
uint32_t A7105ReadID()
{
	uint32_t id = 0;
	A7105_CS = 0;
	SPI_RW(A7105_06_ID_DATA | 0x40);
	SPI_RW(0xFF); id |= SPI_DATA;id<<=8;
	SPI_RW(0xFF); id |= SPI_DATA;id<<=8;
	SPI_RW(0xFF); id |= SPI_DATA;id<<=8;
	SPI_RW(0xFF); id |= SPI_DATA;
	A7105_CS = 1;  
	return id;
}

void A7105WriteFIFO (uint8_t *dat, uint8_t num)
{
	A7105Strobe(A7105_RST_WRPTR);	// reset write pointer
	A7105_CS = 0;
	SPI_RW(A7105_05_FIFO_DATA);
	while(num--)
	{
		SPI_RW(*dat);
		dat++;
	}
	A7105_CS = 1;  
}
void A7105ReadFIFO (uint8_t *dat, uint8_t num)
{
	A7105Strobe(A7105_RST_RDPTR);	// reset read pointer
	A7105_CS = 0;
	SPI_RW(A7105_05_FIFO_DATA | 0x40);
	while(num--)
	{
		SPI_RW(0xFF);
		*dat++=SPI_DATA;
	}
	A7105_CS = 1;
}

void A7105Init(uint32_t id)
{
	A7105SoftReset();
	A7105WriteID(id);	
}

void A7105Config(const uint8_t *regsTable, uint8_t len)
{
		uint8_t i;
    if (regsTable) {
        uint16_t t = 1000;

        for (i = 0; i < len; i++) {
            if (regsTable[i] != 0xFF) {
                A7105WriteReg (i, regsTable[i]);
            }
        }

        A7105Strobe(A7105_STANDBY);

        A7105WriteReg(A7105_02_CALC, 0x01);

        while ((A7105ReadReg(A7105_02_CALC) != 0) && t--) {}

        A7105ReadReg(A7105_22_IF_CALIB_I);

        A7105WriteReg(A7105_24_VCO_CURCAL, 0x13);
        A7105WriteReg(A7105_25_VCO_SBCAL_I, 0x09);
        A7105Strobe(A7105_STANDBY);
    }
}
/////////////////////////////////////////////////////
//flySky2A
/////////////////////////////////////////////////////
uint8_t getNextChannel(uint8_t step)
{
    static data uint8_t channel = 0;
    channel = (channel + step) & 0x0F;
    return rfChannelMap[channel];
}


#define resetTimeout(_ts)	do{timeLastPacket = (_ts);timeout = 4850*2;countTimeout = 0;countPacket++;}while(0)
//void resetTimeout(tick_t timeStamp)
//{
//    timeLastPacket = timeStamp;
//		timeout = 4850*2;
//    countTimeout = 0;
//    countPacket++;
//}

//txId和rxId校验
//bool isValidPacket(const uint8_t *packet) 
//{
//    //const flySky2ARcDataPkt_t *rcPacket = (const flySky2ARcDataPkt_t*) packet;
//    //return (rcPacket->rxId == rxId && rcPacket->txId == txId);
//		return true;
//}

void buildAndWriteTelemetry(uint8_t *packet)
{
    if (packet) {
        static data uint8_t bytesToWrite = FLYSKY_2A_PAYLOAD_SIZE; // first time write full packet to buffer a7105
        flySky2ATelemetryPkt_t *telemertyPacket = (flySky2ATelemetryPkt_t*) packet;
				//FIXME:BAT_ADC
				halfword_t voltage;
				voltage.raw = (uint32_t)adc_voltage*330*11/255*1;
        telemertyPacket->type = FLYSKY_2A_PACKET_TELEMETRY;
        telemertyPacket->sens[0].type = SENSOR_INT_V;
        telemertyPacket->sens[0].number = 0;
        telemertyPacket->sens[0].valueL = voltage.parse.l;
        telemertyPacket->sens[0].valueH = voltage.parse.h;

				//rssi_dBm = 0;
        telemertyPacket->sens[1].type = SENSOR_RSSI;
        telemertyPacket->sens[1].number = 0;
        telemertyPacket->sens[1].valueL = 0;//rssi_dBm & 0xFF;
        telemertyPacket->sens[1].valueH = 0;//(rssi_dBm >> 8) & 0xFF;

        telemertyPacket->sens[2].type = SENSOR_ERR_RATE;
        telemertyPacket->sens[2].number = 0;
        telemertyPacket->sens[2].valueL = errorRate & 0xFF;
        telemertyPacket->sens[2].valueH = (errorRate >> 8) & 0xFF;

        memset (&telemertyPacket->sens[3], 0xFF, 4 * sizeof(flySky2ASens_t));

        A7105WriteFIFO(packet, bytesToWrite);
				
        bytesToWrite = 9 + 3 * sizeof(flySky2ASens_t);// next time write only bytes that could change, the others are already set as 0xFF in buffer a7105
    }
}

bool flySky2AInit()
{
	uint8_t startRxChannel;
	uint32_t id;
	
	rxId = *((uint32_t*)ID_ADDR_RAM);					//使用STC的ID号前4个字节作为rxID
	startRxChannel = flySky2ABindChannels[0];
	A7105Init(0x5475c52A);
	A7105Config(flySky2ARegs, sizeof(flySky2ARegs));
	
	
	id = A7105ReadID();
	if(id != 0x5475c52A)return false;		//无线模块异常
    
	if (txId == 0) 
	{
		bound = false;											//进入对码模式
	} 
	else 
	{
		bound = true;
		startRxChannel = getNextChannel(0);
	}

	A7105WriteReg(A7105_0F_CHANNEL, startRxChannel);
	A7105Strobe(A7105_RX); // start listening
	resetTimeout(0);
	return true;
}


void config_setup()
{
	
	uint8_t temp1 = 0;
	uint8_t temp2 = 0;
	uint8_t offset;
	temp1 = eeprom_read_byte(EEPROM_ADDRESS);
	temp2 = eeprom_read_byte(EEPROM_ADDRESS+1);
	
	if(temp1 == 0x55 && temp2 == 0xAA)
	{
		offset = 2;
		eeprom_read(EEPROM_ADDRESS+offset,&txId,sizeof(txId));	offset += sizeof(txId);
		eeprom_read(EEPROM_ADDRESS+offset,rfChannelMap,FLYSKY_FREQUENCY_COUNT);offset += FLYSKY_FREQUENCY_COUNT;
	}
	else
	{
		txId = 0;
	}
}
////////////////////////////////////////////////
//
////////////////////////////////////////////////
void config_save()
{
	uint8_t offset;
	eeprom_erase(EEPROM_ADDRESS,512);
	eeprom_write_byte(EEPROM_ADDRESS,0x55);
	eeprom_write_byte(EEPROM_ADDRESS+1,0xAA);
	offset = 2;
	eeprom_write(EEPROM_ADDRESS+offset,&txId,sizeof(txId));	offset += sizeof(txId);
	eeprom_write(EEPROM_ADDRESS+offset,rfChannelMap,FLYSKY_FREQUENCY_COUNT);offset += FLYSKY_FREQUENCY_COUNT;
}


void Delay500ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 46;
	j = 153;
	k = 245;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}

void Delay100ms()		//@24.000MHz
{
	unsigned char i, j, k;

	_nop_();
	_nop_();
	i = 10;
	j = 31;
	k = 147;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}


void main()
{
	Delay500ms();
	config_setup();
	uart_init_sbus();
	adc_init();
	pca_init();
	timer0_init();	
	SPI_Init();
	A7105_CS = 1;
	KEY = 1;
	
	

	
	
	
	if(!flySky2AInit())		//无线错误,双闪
	{
		while(1)
		{
			LED = 0;
			Delay100ms();
			LED = 1;
			Delay100ms();
			
			LED = 0;
			Delay100ms();
			LED = 1;
			
			Delay500ms();
			Delay500ms();
		}	
	}
	timeLastSBUSPacket = micros();
	while(1)
	{
		static data tick_t tick_now;
		bit signalReceived = false;			//收到通道数据

		data uint8_t modeReg;
		
		
		if(CCF0)			//下降沿捕获标志
		{
			CCF0 = 0;			
			txrx_finished_time = (CCAP0H<<8)+CCAP0L;		//捕获时间				
			A7105ReadReg_D(A7105_00_MODE,modeReg);
			if (((modeReg & A7105_MODE_TRSR) != 0) && ((modeReg & A7105_MODE_TRER) == 0))  // TX complete
			{
					if (bound) 
					{
							A7105WriteReg_D(A7105_0F_CHANNEL, getNextChannel(1));
					}
					A7105Strobe(A7105_RX);
			}
			else if ((modeReg & (A7105_MODE_CRCF|A7105_MODE_TRER)) == 0)  // RX complete, CRC pass
			{		
					
					data uint8_t bytesToRead = (bound) ? (9 + 2*FLYSKY_2A_CHANNEL_COUNT) : (11 + FLYSKY_FREQUENCY_COUNT);				
					A7105ReadFIFO(packet, bytesToRead);
					switch (packet[0]) 
					{
						case FLYSKY_2A_PACKET_SETTINGS: 		// receiver settings							
						case FLYSKY_2A_PACKET_FS_SETTINGS: 	// failsafe settings
						case FLYSKY_2A_PACKET_RC_DATA:
								//if (isValidPacket(packet)) 		//包校验
								{
										resetTimeout(txrx_finished_time);
										rcData[0].parse.h = packet[9+1], rcData[0].parse.l = packet[9+0];
										rcData[1].parse.h = packet[11+1],rcData[1].parse.l = packet[11+0];
										rcData[2].parse.h = packet[13+1],rcData[2].parse.l = packet[13+0];
										rcData[3].parse.h = packet[15+1],rcData[3].parse.l = packet[15+0];
										rcData[4].parse.h = packet[17+1],rcData[4].parse.l = packet[17+0];
										rcData[5].parse.h = packet[19+1],rcData[5].parse.l = packet[19+0];
										rcData[6].parse.h = packet[21+1],rcData[6].parse.l = packet[21+0];
										rcData[7].parse.h = packet[23+1],rcData[7].parse.l = packet[23+0];
									
										if(packet[0] == FLYSKY_2A_PACKET_RC_DATA);
										{												
												if (sendTelemetry) 
												{
													
																		
														buildAndWriteTelemetry(packet);
														timeTxRequest = txrx_finished_time;
														waitTx = true;
														sendTelemetry = false;			
												}
												
												signalReceived = true;							
										}
										
										if (!waitTx) 
										{
												A7105WriteReg_D(A7105_0F_CHANNEL, getNextChannel(1));
										}
								}
								break;
						case FLYSKY_2A_PACKET_BIND1:
						case FLYSKY_2A_PACKET_BIND2:
								if (!bound) 
								{
										data flySky2ABindPkt_t *bindPacket;									
										resetTimeout(txrx_finished_time);								
										bindPacket = (flySky2ABindPkt_t*) packet;									
										if (bindPacket->rfChannelMap[0] != 0xFF) 
										{
												memcpy(rfChannelMap, bindPacket->rfChannelMap, FLYSKY_FREQUENCY_COUNT); // get TX channels
										}
										txId = bindPacket->txId;
										bindPacket->rxId = rxId;
										memset(bindPacket->rfChannelMap, 0xFF, 26); // erase channelMap and 10 bytes after it

										waitTx = true;
										timeTxRequest = txrx_finished_time;		
										cnt33ms = 0;					//200ms后对频成功
										A7105WriteFIFO(packet, FLYSKY_2A_PAYLOAD_SIZE);
								}
								break;
							default:
									break;
					}
					
					if (!waitTx)
					{
						A7105Strobe(A7105_RX);
					}
			}
			else 
			{
				A7105Strobe(A7105_RX);
			}
		}
		
		
		if (waitTx && (micros() - timeTxRequest) > TX_DELAY) 	//请求之后500us发一个包
		{
				A7105Strobe(A7105_TX);
				waitTx = false;
		}	
		
		
		//对频按键
		if(KEY == 0)
		{	
			if(timeBindRequest)
			{
				if((cnt33ms - timeBindRequest) >= 61)	//长按2s
				{
					bound = false;
					txId = 0;
					memset(rfChannelMap, 0, FLYSKY_FREQUENCY_COUNT);
					A7105WriteReg(A7105_0F_CHANNEL, flySky2ABindChannels[0]);
					
				}
			}
			else 
			{
				timeBindRequest = cnt33ms;
			}
		}
		else
		{
			timeBindRequest = 0;
		}
		
		
		tick_now = micros();
		if (bound) 
		{
			static data uint8_t cntLastCal = 0;
				
			//接收包超时处理
			if ((tick_now - timeLastPacket) > timeout)
			{
				uint16_t stepOver = (tick_now - timeLastPacket) / (3850*2);//flySky2ATimings.packet;
        timeLastPacket = (stepOver > 1) ? (tick_now) : (timeLastPacket + timeout);

				
        A7105Strobe(A7105_STANDBY);
        A7105WriteReg(A7105_0F_CHANNEL, getNextChannel(stepOver%FLYSKY_FREQUENCY_COUNT));
        A7105Strobe(A7105_RX);

        if(countTimeout > 31) {							//连续丢了32个包
            timeout = 5775*2;//flySky2ATimings.syncPacket;		
        } 
				else 
				{
            timeout = 3850*2;//flySky2ATimings.packet;
            countTimeout++;
        }
			}
			
			//101包的时间,统计误码率,并启动发送回传包
			if((cnt33ms - cntLastCal) >= 12)//3.85*101=388.85/32.768=11.867
			{
				cntLastCal = cnt33ms;
	
				//误码率统计
				errorRate = (countPacket >= 100) ? (0) : (100 - countPacket);
				countPacket = 0;
				
				//获取一次电压
				adc_voltage = ADC_RES;
				ADC_BEGIN(0);
				//发送回传包
				sendTelemetry = true;
			}
			
			//
			if(errorRate > 80)				//错误率达80%
			{
				if(cnt33ms_led >= 400/33)		//250ms慢闪，信号丢失
				{
					cnt33ms_led = 0;
					LED = !LED;
				}
			}
			else								//通信正常，常亮
			{
				LED = 0;
			}
    } 
		else //正在对码
		{				
			if (cnt33ms > 6 && rfChannelMap[0] != 0 && txId != 0) 		//对码成功,200ms
			{
				bound = true;
				config_save();
				cnt33ms = 0;
				A7105WriteReg_D(A7105_0F_CHANNEL, getNextChannel(1));
				resetTimeout(micros());
				
			}
			
			
			if(cnt33ms_led >= 3)		//100ms快闪
			{
				cnt33ms_led = 0;
				LED = !LED;
			}
    }
		
		//14ms发一次sbus数据
		if((tick_now - timeLastSBUSPacket) > 14000*2)	
		{
				timeLastSBUSPacket = tick_now;
				
				if(signalReceived)
				{
					sbusData[0].raw = (rcData[0].raw*16)/10 - 1408;
					sbusData[1].raw = (rcData[1].raw*16)/10 - 1408;
					sbusData[2].raw = (rcData[2].raw*16)/10 - 1408;
					sbusData[3].raw = (rcData[3].raw*16)/10 - 1408;
					sbusData[4].raw = (rcData[4].raw*16)/10 - 1408;
					sbusData[5].raw = (rcData[5].raw*16)/10 - 1408;
					sbusData[6].raw = (rcData[6].raw*16)/10 - 1408;
					sbusData[7].raw = (rcData[7].raw*16)/10 - 1408;			


					sbus_buffer[0] = sbusData[0].parse.l;
					sbus_buffer[1] = sbusData[0].parse.h | (sbusData[1].parse.l)<<3;
					sbus_buffer[2] = ((sbusData[1].raw)>>5) | (sbusData[2].parse.l<<6);
					sbus_buffer[3] = ((sbusData[2].raw)>>2);
					sbus_buffer[4] = ((sbusData[2].raw)>>10)| (sbusData[3].parse.l<<1);
					sbus_buffer[5] = ((sbusData[3].raw)>>7) | (sbusData[4].parse.l<<4);
					sbus_buffer[6] = ((sbusData[4].raw)>>4) | (sbusData[5].parse.l<<7);
					sbus_buffer[7] = ((sbusData[5].raw)>>1);
					sbus_buffer[8] = ((sbusData[5].raw)>>9) | (sbusData[6].parse.l<<2);
					sbus_buffer[9] = ((sbusData[6].raw)>>6) | (sbusData[7].parse.l<<5);
					sbus_buffer[10] = ((sbusData[7].raw)>>3);		
				
				}

				
			
				
				//启动SBUS第一个字节
				SBUS_PUTC(0x0F);
				sbus_cnt = 0;
				sbus_busy = 1;		
			}
		
	}
	
	while(1)
	{
		Delay100ms();
	}
}
/////////////////////////////////////////////////////////////////////////////////
//中断
/////////////////////////////////////////////////////////////////////////////////
void uart_isr() interrupt 4
{
  if(TI)			//发送完成
	{
	 TI = 0;
	 if(sbus_cnt < 22)
	 {
			SBUS_PUTC(sbus_buffer[sbus_cnt]);
			sbus_cnt++;
	 }
	 else if(sbus_cnt == 22)						//
	 {
			SBUS_PUTC(0x00);
			sbus_cnt++;	 
	 }
	 else if(sbus_cnt == 23)						//endbyte
	 {
			SBUS_PUTC(0x00);
			sbus_cnt++;			 
	 }
	 else
	 {
			sbus_busy = 0;
	 }
	 
	}	
	
	if(RI)
	{
		RI = 0;
	}
	
}

//@24M,12T
#define GET_CHANNEL_HT(_us)	(65536 - (((_us)<<1)- 800))		
#define GET_CHANNEL_LT(_us)	(65536 - ((_us)<<1))
void tim0_isr() interrupt 1
{
	data halfword_t ppm_ht;
	static bit next_level = 0;
	static data uint8_t ppm_cnt = 0;
	PPM_OUT = next_level;
	
	if(next_level)		//当前输出1
	{
		TL0 = GET_CHANNEL_LT(400);
		TH0 = GET_CHANNEL_LT(400)>>8;		
		next_level = 0;
	}
	else		//当前输出0
	{
		if(ppm_cnt < 8)
		{
			ppm_ht.raw =  GET_CHANNEL_HT(rcData[ppm_cnt].raw);	
			ppm_cnt ++;
		}
		else
		{
			ppm_ht.raw = GET_CHANNEL_LT(3000);	//3ms同步电平
			ppm_cnt = 0;
		}
		TL0 = ppm_ht.parse.l,TH0 = ppm_ht.parse.h;
		next_level = 1;
	}
}


void PCA_isr() interrupt 7
{
	CF = 0;			//清除中断标志
	cnt33ms++;
	cnt33ms_led++;
}
