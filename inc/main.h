/*
 * main.h
 *
 *  Created on: 2018年4月14日
 *      Author: weihaochen
 */

#ifndef MAIN_H_
#define MAIN_H_


/***************** LED操作函数宏定义(低电平点亮) *********************/
#define LED_ON(x)      x=0    //打开LED
#define LED_OFF(x)     x=1    //关闭LED
#define LED_TOGGLE(x)  x^=1   //翻转LED
/****************************** end *********************************/



/***************** LED所在GPIO口时钟宏定义 ***************************/
#define LED0_GPIO_RCC_CLK  RCC_APB2Periph_GPIOB//LED0 GPIO RCC时钟
#define LED1_GPIO_RCC_CLK  RCC_APB2Periph_GPIOE//LED1 GPIO RCC时钟
/****************************** end *********************************/


/********************* LED所在GPIO口宏定义 ***************************/
#define LED0_GPIO  GPIOB
#define LED1_GPIO  GPIOE

#define LED0_PIN   GPIO_Pin_5
#define LED1_PIN   GPIO_Pin_5

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5

#define LED_GREEN    LED0   //绿色
#define LED_BLUE     LED1   //蓝色
/****************************** end *********************************/


/*************************** 调用说明 *******************************/
//  1、引用led.h头文件
//  2、调用初始化函数
//  3、使用宏定义操作LED
//  示例:
//      LED_ON(LED0);//打开LED0
//      LED_OFF(LED1);//关闭LED1
//      LED_TOGGLE(LED_GREEN);//翻转绿色LED
/****************************** end *********************************/



//按键定义
#define KEY0  0
#define KEY1  1
#define KEY2  2
#define KEY3  3

//key0 PE4
//key1 PE3
//key2 PE2
//key3/key_up PA0


/********************* 按键所在GPIO口宏定义 ***************************/
#define KEY0_GPIO GPIOE
#define KEY1_GPIO GPIOE
#define KEY2_GPIO GPIOE
#define KEY3_GPIO GPIOA

#define KEY0_PIN GPIO_Pin_4
#define KEY1_PIN GPIO_Pin_3
#define KEY2_PIN GPIO_Pin_2
#define KEY3_PIN GPIO_Pin_0
/****************************** end *********************************/


/********************* 按键所在GPIO口时钟宏定义 *********************/
#define KEY0_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY1_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY2_RCC_CLK  RCC_APB2Periph_GPIOE
#define KEY3_RCC_CLK  RCC_APB2Periph_GPIOA
/****************************** end *********************************/



/********************* 按键硬件状态获取宏定义 ***********************/
#define KEY0_HARD_STA  ((KEY0_GPIO->IDR & (uint16_t)KEY0_PIN)==0)
#define KEY1_HARD_STA  ((KEY1_GPIO->IDR & (uint16_t)KEY1_PIN)==0)
#define KEY2_HARD_STA  ((KEY2_GPIO->IDR & (uint16_t)KEY2_PIN)==0)
#define KEY3_HARD_STA  ((KEY3_GPIO->IDR & (uint16_t)KEY3_PIN)!=0)//这个按键是高电平有效
/****************************** end *********************************/



/********************* 按键按下返回值宏定义 *************************/
#define KEY0_PRES   1   //KEY0按下
#define KEY1_PRES   2   //KEY1按下
#define KEY2_PRES   3   //KEY2按下
#define WKUP_PRES   4   //KEY_UP按下(即WK_UP/KEY_UP)
/****************************** end *********************************/
#include "timer.h"


/********************* 按键函数声明 *********************************/

void Key_Init(void);   //IO初始化
u8 Key_Scan(u8);       //按键扫描函数
u8 Key_GetSta(u8 key); //按键当前状态获取  1:已被按下  0:未被按下
/************************* 函数声明 *********************************/
void LED_Init(void);//初始化

void    DHT11_INIT( DHT11 *p_dev );
void    DS18B20_INIT( DS18B20 *dev_p );
void    UART1_INIT( UART* self );
void    UART2_INIT( UART* self );
void    UART3_INIT( UART* self );
void    ADC_INIT( ADC *this );
void    RCC_Configuration(void);
ErrorStatus    system_send_message( void );
void    system_send_all_datas( void );
#endif /* MAIN_H_ */
