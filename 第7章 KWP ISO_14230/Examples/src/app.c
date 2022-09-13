

/*
	KWP ISO14230 测试，OBD7
  
	
	siyuan 2022-09-07
*/

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

#include	<stdio.h>
#include "can.h"
#include "bsp.h"
#include "timer.h"
#include "usart.h"	 
//#include "vpwm.h"

usb_core_driver cdc_acm;

extern  uint16_t iRxUsbLen;//接收Usb数据长度
extern uint16_t iRxUsbFlag;//接收完成标记  0x80接收完成

void GetKLineTime(void);

void EcuSendData(void);
u8 RecAdd(void);
u8 Wait5BpsAdd(void);
/*!
    \brief      main routine will construct a USB mass storage device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{

		//uint8_t SendData1[100]={0x09,0X02,0X10,0x03,0x00,0x00,0x00,0x00,0x00};
		
//		uint8_t SendData2[100]={0x0F,0X02,0X10,0x01,0x02,0x03,0x04,0x05,0x00};
//		uint8_t i=0;
		iRxUsbLen=0;
		iRxUsbFlag=0;
		
    usb_rcu_config();

    usb_timer_init();

    usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);

    usb_intr_config();
    
#ifdef USE_IRC48M
    /* CTC peripheral clock enable */
    rcu_periph_clock_enable(RCU_CTC);

    /* CTC configure */
    ctc_config();

    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET) {
    }
#endif

		//
	//StCanInitTest();//CAN test
	//	FdCanInitTest();//FD CAN
		
		TIM5_config();//用于定时
		TIM6_config();//用于测量时间
		Led_Init();//初始化 普通IO

		
		iHBitSum=0;//
		iLBitSum=0;//
		//初始化变量
		iKDataMode=0;
		iPartValue=0;
		iEcuFlag=0;//采数模式
		
		Adc_Init();//初始化电压采样
		
//		CanFD_config(can_500k,Data_1M);//CAN FD 500k  4M 80%
//		//CAN1_Config16BitFilter(0xFC00,0xFD00);//设置过滤ID 
//		CAN_setAllfit();//设置不过滤ID
		
		uint8_t SendData[10]={0xC1,0x33,0xF1,0x81,0x66};
		uint8_t SendData1[10]={0x83,0xF1,0x11,0xC1,0xEF,0x8F,0xC4};
		gpio_bit_set(GPIOB,GPIO_PIN_11);	//PB11=1 开启 1027 
		//UART1_Init(10416); //波特率
		InitKinSys(0,0,10416);//25/25 拉低拉高激活
    /* main loop */
  while (1) 
	{
		SendKwp14230Frame(SendData);//CX 开头的帧
		Delay_ms(500);
		SendKwp14230Frame(SendData1);//8X 开头的帧
		Delay_ms(500);
//		//TEST
//	 if (USBD_CONFIGURED == cdc_acm.dev.cur_status) 
//	 {
//      if (0U == cdc_acm_check_ready(&cdc_acm)) 
//			{
//         cdc_acm_data_receive(&cdc_acm);		
//					// 准备接收数据 							           
//			}
//			else 
//			{							//发送数据
//        //cdc_acm_data_send(&cdc_acm);
//      }
//		} 
//		Delay_us(10);
//		continue;				
	
    }
}



